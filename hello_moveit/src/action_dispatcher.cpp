#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/int8.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <Eigen/Geometry>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// ============================================================
// intent 编码：与 foundationpose_node.py 的 INTENT_CODES 保持一致
// ============================================================
enum
{
  INTENT_APPROACH = 0,   // 靠近：只移位置到物体正上方
  INTENT_GRIND = 1,      // 打磨：连续视觉伺服跟随
  INTENT_GRASP = 2,      // 抓取：6D 姿态 + 抓取偏置
};

// 执行确认后要衔接的动作：离散轨迹执行完之后是否自动启动连续伺服
enum class PendingAfter
{
  NONE,          // approach/grasp：执行完即止
  START_SERVO,   // grind：初始位姿对齐执行完 -> 自动启动连续伺服
};

// approach 参数：物体正上方 offset（米）
constexpr double APPROACH_HEIGHT = 0.15;
constexpr double kPi = 3.14159265358979323846;

// ============================================================
// 抓取偏置 T_OBJ_TCP（与 hello_moveit.cpp 一致，针对红三角块/夹爪标定）
// ============================================================
Eigen::Isometry3d buildTcpOffset()
{
  Eigen::Matrix3d R;
  R <<  0.7071,  0.0, -0.7071,
       -0.7071,  0.0, -0.7071,
        0.0,     1.0,  0.0;
  Eigen::Vector3d t(-0.0072, 0.04958, 0.01725);
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = R;
  T.translation() = t;
  return T;
}

// ============================================================
// approach 末端姿态：垂直向下（绕基座 X 轴转 180°，使末端 Z 轴朝下）。
// 若末端工具的「垂直向下」不是这个旋转，改这里；也可改成保持当前 Tool_Link 姿态。
// ============================================================
Eigen::Isometry3d approachOrientation()
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::AngleAxisd(kPi, Eigen::Vector3d::UnitX()).toRotationMatrix();
  return T;
}

// ============================================================
// controller 管理 helper（复用 hello_moveit.cpp / visual_servo.cpp 逻辑）
// ============================================================
namespace
{
std::string getControllerState(const rclcpp::Node::SharedPtr& node,
                               const std::string& controller_name)
{
  auto client = node->create_client<controller_manager_msgs::srv::ListControllers>(
      "/controller_manager/list_controllers");
  if (!client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node->get_logger(), "Controller list service not available.");
    return "";
  }
  auto req = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
  auto future = client->async_send_request(req);
  if (future.wait_for(std::chrono::seconds(6)) != std::future_status::ready) {
    RCLCPP_ERROR(node->get_logger(), "Timed out listing controllers.");
    return "";
  }
  const auto response = future.get();
  for (const auto& controller : response->controller) {
    if (controller.name == controller_name) {
      return controller.state;
    }
  }
  return "";
}

bool switchControllers(const rclcpp::Node::SharedPtr& node,
                       const std::vector<std::string>& activate,
                       const std::vector<std::string>& deactivate)
{
  auto logger = node->get_logger();
  auto client = node->create_client<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controller");
  if (!client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(logger, "Controller switch service not available.");
    return false;
  }

  bool already_in_target_state = true;
  for (const auto& c : activate) {
    if (getControllerState(node, c) != "active") {
      already_in_target_state = false;
      break;
    }
  }
  if (already_in_target_state) {
    for (const auto& c : deactivate) {
      const auto state = getControllerState(node, c);
      if (!state.empty() && state != "inactive") {
        already_in_target_state = false;
        break;
      }
    }
  }
  if (already_in_target_state) {
    RCLCPP_INFO(logger, "Controllers already in desired state, skipping switch.");
    return true;
  }

  auto req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  req->activate_controllers = activate;
  req->deactivate_controllers = deactivate;
  req->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
  req->activate_asap = true;
  req->timeout.sec = 5;

  auto future = client->async_send_request(req);
  if (future.wait_for(std::chrono::seconds(6)) != std::future_status::ready) {
    RCLCPP_ERROR(logger, "Timed out switching controllers.");
    return false;
  }
  const auto response = future.get();
  if (!response->ok) {
    RCLCPP_ERROR(logger, "Controller switch failed.");
    return false;
  }
  RCLCPP_INFO(logger, "Controller switch complete: activate=[%s] deactivate=[%s]",
              activate.empty() ? "" : activate.front().c_str(),
              deactivate.empty() ? "" : deactivate.front().c_str());
  return true;
}
}  // namespace

// ============================================================
// 统一动作调度节点
// ============================================================
class ActionDispatcher : public rclcpp::Node
{
public:
  ActionDispatcher() : Node("action_dispatcher")
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    intent_sub_ = create_subscription<std_msgs::msg::Int8>(
        "/object_intent", 10,
        [this](const std_msgs::msg::Int8::SharedPtr msg) { intentCallback(msg); });
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/object_pose", 10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { poseCallback(msg); });

    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
        "/servo_node/delta_twist_cmds", 10);
    servo_start_client_ = create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
    controller_switch_client_ = create_client<controller_manager_msgs::srv::SwitchController>(
        "/controller_manager/switch_controller");

    // 执行确认服务：规划完成后，等用户调用此服务才真正执行机械臂
    execute_srv_ = create_service<std_srvs::srv::Trigger>(
        "/action_dispatcher/execute",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
          executeCallback(resp);
        });

    RCLCPP_INFO(get_logger(), "ActionDispatcher ready. Waiting for /object_intent ...");
  }

  ~ActionDispatcher() override
  {
    if (grind_active_) {
      publishZeroTwistBurst();
    }
  }

  // 由主循环调用：有 pending intent 就同步执行（approach/grasp 阻塞规划，grind 启动 timer）
  void processPendingAction()
  {
    int code = pending_intent_.exchange(-1);
    if (code == -1) {
      return;
    }
    switch (code) {
      case INTENT_APPROACH:
        doApproach();
        break;
      case INTENT_GRASP:
        doGrasp();
        break;
      case INTENT_GRIND:
        doGrind();
        break;
      default:
        RCLCPP_WARN(get_logger(), "Unknown intent code %d, ignored.", code);
    }
  }

private:
  // ---------- 回调 ----------
  void intentCallback(const std_msgs::msg::Int8::SharedPtr msg)
  {
    if (msg->data == last_intent_) {
      return;  // foundationpose_node 已去重，这里双保险
    }
    last_intent_ = msg->data;
    RCLCPP_INFO(get_logger(), "Received intent code %d, scheduling action.", msg->data);
    pending_intent_.store(msg->data);
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    latest_pose_ = msg;
  }

  // ---------- 通用 ----------
  void ensureMoveGroup()
  {
    if (move_group_) {
      return;
    }
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "arm");
    move_group_->setPlanningTime(50);
    move_group_->setNumPlanningAttempts(30);
    move_group_->setGoalPositionTolerance(0.005);
    move_group_->setGoalOrientationTolerance(0.2);

    // RViz 可视化：显示目标位姿轴 + 规划轨迹线
    visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
        shared_from_this(), "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
        move_group_->getRobotModel());
    visual_tools_->deleteAllMarkers();
  }

  geometry_msgs::msg::PoseStamped::SharedPtr waitForPose(double timeout_s)
  {
    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(static_cast<int>(timeout_s * 1000));
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        if (latest_pose_) {
          return latest_pose_;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return nullptr;
  }

  bool transformPoseToBase(const geometry_msgs::msg::PoseStamped& obj,
                           geometry_msgs::msg::PoseStamped& out)
  {
    if (!tf_buffer_->canTransform("base_link", obj.header.frame_id, obj.header.stamp,
                                  rclcpp::Duration::from_seconds(1.0))) {
      RCLCPP_ERROR(get_logger(), "Cannot transform from '%s' to base_link",
                   obj.header.frame_id.c_str());
      return false;
    }
    try {
      tf_buffer_->transform(obj, out, "base_link");
      return true;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_ERROR(get_logger(), "TF transform failed: %s", ex.what());
      return false;
    }
  }

  bool planAndShow(const geometry_msgs::msg::Pose& pose)
  {
    // 在 RViz 里显示目标位姿轴（先于规划，方便确认目标对不对）
    if (visual_tools_) {
      Eigen::Isometry3d target;
      tf2::fromMsg(pose, target);
      visual_tools_->publishAxisLabeled(target, "TCP Target");
      visual_tools_->trigger();
    }

    move_group_->setPoseTarget(pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (!static_cast<bool>(move_group_->plan(plan))) {
      RCLCPP_ERROR(get_logger(), "MoveIt planning failed.");
      return false;
    }

    // 在 RViz 里显示规划轨迹线
    if (visual_tools_) {
      const auto* ee = move_group_->getRobotModel()->getLinkModel("Tool_Link");
      const auto* jmg = move_group_->getRobotModel()->getJointModelGroup("arm");
      visual_tools_->publishTrajectoryLine(plan.trajectory_, ee, jmg);
      visual_tools_->trigger();
    }

    // 存下规划，等人工确认后再执行
    {
      std::lock_guard<std::mutex> lock(plan_mutex_);
      pending_plan_ = plan;
      plan_ready_ = true;
    }
    RCLCPP_WARN(get_logger(),
                "规划完成，已显示在 RViz。确认执行请调用："
                "ros2 service call /action_dispatcher/execute std_srvs/srv/Trigger {}");
    return true;
  }

  void executeCallback(std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
  {
    // 用户可能比规划更快地回答「是否启动」：这里等规划就绪，最多等 30s，
    // 避免 execute 命令早于规划到达而报「没有待执行的规划」。
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(plan_mutex_);
        if (plan_ready_) {
          break;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    {
      std::lock_guard<std::mutex> lock(plan_mutex_);
      if (!plan_ready_) {
        resp->success = false;
        resp->message = "没有待执行的规划（等待 30s 规划仍未完成）";
        return;
      }
      plan = pending_plan_;
      plan_ready_ = false;
    }
    RCLCPP_INFO(get_logger(), "收到执行确认，开始执行机械臂 ...");
    move_group_->execute(plan);
    RCLCPP_INFO(get_logger(), "Motion execution finished.");

    // grind：对齐执行完 -> 自动切速度控制器并启动连续伺服
    if (pending_after_ == PendingAfter::START_SERVO) {
      pending_after_ = PendingAfter::NONE;
      RCLCPP_INFO(get_logger(), "对齐完成，启动连续视觉伺服 ...");
      startServo();
    }

    resp->success = true;
    resp->message = "执行完成";
  }

  // ---------- approach：物体正上方 + 垂直向下 ----------
  void doApproach()
  {
    pending_after_ = PendingAfter::NONE;
    ensureMoveGroup();
    auto pose = waitForPose(3.0);
    if (!pose) {
      RCLCPP_ERROR(get_logger(), "Approach: no /object_pose received in time.");
      return;
    }
    geometry_msgs::msg::PoseStamped obj_base;
    if (!transformPoseToBase(*pose, obj_base)) {
      return;
    }
    if (!switchControllers(shared_from_this(), { "arm_controller" },
                           { "arm_velocity_controller" })) {
      RCLCPP_ERROR(get_logger(), "Approach: failed to switch to trajectory controller.");
      return;
    }

    Eigen::Isometry3d target = approachOrientation();
    target.translation() = Eigen::Vector3d(
        obj_base.pose.position.x,
        obj_base.pose.position.y,
        obj_base.pose.position.z + APPROACH_HEIGHT);

    RCLCPP_INFO(get_logger(), "Approach target: [%.3f, %.3f, %.3f]",
                target.translation().x(), target.translation().y(),
                target.translation().z());
    if (!planAndShow(tf2::toMsg(target))) {
      RCLCPP_ERROR(get_logger(), "Approach planning failed.");
    }
  }

  // ---------- grasp：T_BASE_TCP = T_BASE_OBJ * T_OBJ_TCP ----------
  void doGrasp()
  {
    pending_after_ = PendingAfter::NONE;
    ensureMoveGroup();
    auto pose = waitForPose(3.0);
    if (!pose) {
      RCLCPP_ERROR(get_logger(), "Grasp: no /object_pose received in time.");
      return;
    }
    geometry_msgs::msg::PoseStamped obj_base;
    if (!transformPoseToBase(*pose, obj_base)) {
      return;
    }
    if (!switchControllers(shared_from_this(), { "arm_controller" },
                           { "arm_velocity_controller" })) {
      RCLCPP_ERROR(get_logger(), "Grasp: failed to switch to trajectory controller.");
      return;
    }

    Eigen::Isometry3d T_BASE_OBJ;
    tf2::fromMsg(obj_base.pose, T_BASE_OBJ);
    const Eigen::Isometry3d T_BASE_TCP = T_BASE_OBJ * buildTcpOffset();

    RCLCPP_INFO(get_logger(), "Grasp TCP target: [%.3f, %.3f, %.3f]",
                T_BASE_TCP.translation().x(), T_BASE_TCP.translation().y(),
                T_BASE_TCP.translation().z());
    if (!planAndShow(tf2::toMsg(T_BASE_TCP))) {
      RCLCPP_ERROR(get_logger(), "Grasp planning failed.");
    }
  }

  // ---------- grind：先初始位姿对齐（离散规划），对齐执行完再启动连续伺服 ----------
  void doGrind()
  {
    ensureMoveGroup();
    auto pose = waitForPose(3.0);
    if (!pose) {
      RCLCPP_ERROR(get_logger(), "Grind: no /object_pose received in time.");
      return;
    }
    geometry_msgs::msg::PoseStamped obj_base;
    if (!transformPoseToBase(*pose, obj_base)) {
      return;
    }
    if (!switchControllers(shared_from_this(), { "arm_controller" },
                           { "arm_velocity_controller" })) {
      RCLCPP_ERROR(get_logger(),
                   "Grind: failed to switch to trajectory controller for alignment.");
      return;
    }

    Eigen::Isometry3d T_BASE_OBJ;
    tf2::fromMsg(obj_base.pose, T_BASE_OBJ);
    const Eigen::Isometry3d T_BASE_TCP = T_BASE_OBJ * buildTcpOffset();

    RCLCPP_INFO(get_logger(), "Grind alignment target: [%.3f, %.3f, %.3f]",
                T_BASE_TCP.translation().x(), T_BASE_TCP.translation().y(),
                T_BASE_TCP.translation().z());
    if (!planAndShow(tf2::toMsg(T_BASE_TCP))) {
      RCLCPP_ERROR(get_logger(), "Grind alignment planning failed.");
      return;
    }
    // 对齐执行完 -> 自动启动伺服
    pending_after_ = PendingAfter::START_SERVO;
    RCLCPP_INFO(get_logger(), "Grind alignment planned. 执行确认后将自动启动连续伺服。");
  }

  // ---------- 启动连续视觉伺服：切 velocity controller + start_servo + 控制循环 ----------
  void startServo()
  {
    if (grind_active_) {
      RCLCPP_WARN(get_logger(), "Servo already active.");
      return;
    }
    if (!switchControllers(shared_from_this(), { "arm_velocity_controller" },
                           { "arm_controller" })) {
      RCLCPP_ERROR(get_logger(), "Servo: failed to switch to velocity controller.");
      return;
    }
    if (!servo_start_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "Servo: /servo_node/start_servo service unavailable.");
      return;
    }
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = servo_start_client_->async_send_request(req);
    if (future.wait_for(std::chrono::seconds(6)) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "Servo: timed out starting servo.");
      return;
    }
    if (!future.get()->success) {
      RCLCPP_ERROR(get_logger(), "Servo: servo start failed: %s",
                   future.get()->message.c_str());
      return;
    }

    grind_active_ = true;
    grind_timer_ = create_wall_timer(std::chrono::milliseconds(100),
                                     [this]() { grindControlLoop(); });
    RCLCPP_INFO(get_logger(), "Servo started, continuous tracking loop running.");
  }

  void grindControlLoop()
  {
    // 1. 当前 Tool_Link 位姿
    geometry_msgs::msg::TransformStamped t_base_tool;
    try {
      t_base_tool = tf_buffer_->lookupTransform("base_link", "Tool_Link", tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "TF error (Tool_Link): %s", ex.what());
      return;
    }

    // 2. 最新物体位姿
    geometry_msgs::msg::PoseStamped::SharedPtr obj;
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      obj = latest_pose_;
    }
    if (!obj) {
      return;
    }
    geometry_msgs::msg::PoseStamped obj_base;
    if (!transformPoseToBase(*obj, obj_base)) {
      return;
    }

    // 3. 期望 TCP = T_BASE_OBJ * T_OBJ_TCP；当前 TCP = T_BASE_TCP
    Eigen::Isometry3d T_BASE_OBJ;
    tf2::fromMsg(obj_base.pose, T_BASE_OBJ);
    const Eigen::Isometry3d T_des = T_BASE_OBJ * buildTcpOffset();
    const Eigen::Isometry3d T_curr = tf2::transformToEigen(t_base_tool);

    // 4. 误差（照搬 visual_servo.cpp 的 P 控制）
    Eigen::Vector3d pos_error = T_des.translation() - T_curr.translation();
    Eigen::Matrix3d R_error = T_des.linear() * T_curr.linear().transpose();
    Eigen::AngleAxisd angle_axis(R_error);
    Eigen::Vector3d rot_error = Eigen::Vector3d::Zero();
    if (std::abs(angle_axis.angle()) > 1e-5) {
      rot_error = angle_axis.axis() * angle_axis.angle();
    }

    auto twist_msg = geometry_msgs::msg::TwistStamped();
    twist_msg.header.stamp = now();
    twist_msg.header.frame_id = "base_link";

    // 截止误差带：够小就停
    if (pos_error.norm() < 0.004 && rot_error.norm() < 0.08) {
      twist_pub_->publish(twist_msg);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Target reached, holding.");
      return;
    }

    const double kp_pos = 0.9;
    const double kp_rot = 0.6;
    auto clip = [](double v, double m) { return std::max(-m, std::min(m, v)); };

    twist_msg.twist.linear.x = clip(kp_pos * pos_error.x(), 0.08);
    twist_msg.twist.linear.y = clip(kp_pos * pos_error.y(), 0.08);
    twist_msg.twist.linear.z = clip(kp_pos * pos_error.z(), 0.08);
    twist_msg.twist.angular.x = clip(kp_rot * rot_error.x(), 0.18);
    twist_msg.twist.angular.y = clip(kp_rot * rot_error.y(), 0.18);
    twist_msg.twist.angular.z = clip(kp_rot * rot_error.z(), 0.18);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                         "Tracking... Pos err %.3fm Rot err %.3frad",
                         pos_error.norm(), rot_error.norm());
    twist_pub_->publish(twist_msg);
  }

  void publishZeroTwistBurst()
  {
    auto stop_msg = geometry_msgs::msg::TwistStamped();
    stop_msg.header.frame_id = "base_link";
    for (int i = 0; i < 5; ++i) {
      stop_msg.header.stamp = now();
      twist_pub_->publish(stop_msg);
      rclcpp::sleep_for(std::chrono::milliseconds(20));
    }
  }

  // ---------- 成员 ----------
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr intent_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr controller_switch_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr execute_srv_;
  rclcpp::TimerBase::SharedPtr grind_timer_;

  std::mutex pose_mutex_;
  geometry_msgs::msg::PoseStamped::SharedPtr latest_pose_;
  std::mutex plan_mutex_;
  moveit::planning_interface::MoveGroupInterface::Plan pending_plan_;
  bool plan_ready_{ false };
  std::atomic<int> pending_intent_{ -1 };
  int last_intent_{ -1 };
  bool grind_active_{ false };
  PendingAfter pending_after_{ PendingAfter::NONE };
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ActionDispatcher>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  // 独立线程持续 spin：处理订阅回调 + service 响应。
  // processPendingAction 里的同步 service 调用（列/切控制器）依赖这个线程才能拿到响应，
  // 否则 future.wait_for 会一直超时（与 hello_moveit.cpp 的做法一致）。
  std::thread spinner([&executor]() { executor.spin(); });

  while (rclcpp::ok()) {
    node->processPendingAction();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
