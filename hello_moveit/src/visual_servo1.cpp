#include <atomic>
#include <csignal>

#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int8.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <Eigen/Geometry>

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

namespace
{
std::atomic_bool g_visual_servo_stop_requested{false};

void handleSignal(int signum)
{
  if (signum == SIGINT) {
    g_visual_servo_stop_requested.store(true);
  }
}
}  // namespace

class VisualServoNode : public rclcpp::Node
{
public:
  VisualServoNode() : Node("visual_servo")
  {
    // 向 MoveIt Servo 发布速度指令 (TwistStamped 消息)
    // 注意：这里的 topic 需要根据你实际 launch 的 servo 节点配置来调整，默认可能是 ~/delta_twist_cmds
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/servo_node/delta_twist_cmds", 10);
    
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 订阅目标位姿
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/object_pose", 10,
      std::bind(&VisualServoNode::poseCallback, this, std::placeholders::_1));

    status_sub_ = this->create_subscription<std_msgs::msg::Int8>(
      "/servo_node/status", 10,
      std::bind(&VisualServoNode::statusCallback, this, std::placeholders::_1));

    // 实机串口链路较慢，控制循环先降到 10Hz，减少连续小步命令堆积
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&VisualServoNode::controlLoop, this));

    servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
    controller_switch_client_ = this->create_client<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controller");
    start_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&VisualServoNode::ensureServoStarted, this));

    RCLCPP_INFO(this->get_logger(), "Visual Servo Node initialized. Waiting for /object_pose and TF...");
  }

  ~VisualServoNode() override
  {
    safeStopAndRestoreController();
  }

  void safeStopAndRestoreController()
  {
    if (shutdown_handled_) {
      return;
    }
    shutdown_handled_ = true;

    RCLCPP_WARN(this->get_logger(), "Visual servo is stopping. Publishing zero twist and restoring trajectory controller.");

    publishZeroTwistBurst();
    switchBackToTrajectoryController();
  }

private:
  void publishZeroTwistBurst()
  {
    auto stop_msg = geometry_msgs::msg::TwistStamped();
    stop_msg.header.frame_id = "base_link";

    for (int i = 0; i < 5; ++i) {
      stop_msg.header.stamp = this->now();
      twist_pub_->publish(stop_msg);
      rclcpp::sleep_for(std::chrono::milliseconds(20));
    }
  }

  void switchBackToTrajectoryController()
  {
    if (!controller_switch_client_->wait_for_service(std::chrono::milliseconds(500))) {
      RCLCPP_WARN(this->get_logger(),
                  "Controller switch service unavailable during shutdown, cannot restore trajectory controller.");
      return;
    }

    auto req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    req->activate_controllers = { "arm_controller" };
    req->deactivate_controllers = { "arm_velocity_controller" };
    req->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
    req->activate_asap = true;
    req->timeout.sec = 5;

    auto future = controller_switch_client_->async_send_request(req);
    if (future.wait_for(std::chrono::seconds(6)) != std::future_status::ready) {
      RCLCPP_WARN(this->get_logger(), "Timed out while restoring trajectory controller during shutdown.");
      return;
    }

    const auto response = future.get();
    if (response->ok) {
      velocity_controller_active_ = false;
      RCLCPP_INFO(this->get_logger(), "Trajectory controller restored, velocity controller deactivated.");
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to restore trajectory controller during shutdown.");
    }
  }

  void ensureServoStarted()
  {
    if (servo_started_ || start_request_pending_ || switch_request_pending_) {
      return;
    }

    if (!velocity_controller_active_) {
      ensureVelocityControllerActive();
      return;
    }

    if (!servo_start_client_->wait_for_service(std::chrono::milliseconds(100))) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                           "Waiting for /servo_node/start_servo service...");
      return;
    }

    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    start_request_pending_ = true;
    servo_start_client_->async_send_request(
      req,
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        start_request_pending_ = false;
        auto resp = future.get();
        if (resp->success) {
          servo_started_ = true;
          start_timer_->cancel();
          RCLCPP_INFO(this->get_logger(), "MoveIt Servo started.");
        } else {
          RCLCPP_WARN(this->get_logger(), "Failed to start MoveIt Servo: %s", resp->message.c_str());
        }
      });
  }

  void ensureVelocityControllerActive()
  {
    if (velocity_controller_active_ || switch_request_pending_) {
      return;
    }

    if (!controller_switch_client_->wait_for_service(std::chrono::milliseconds(100))) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                           "Waiting for /controller_manager/switch_controller service...");
      return;
    }

    auto req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    req->activate_controllers = { "arm_velocity_controller" };
    req->deactivate_controllers = { "arm_controller" };
    req->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
    req->activate_asap = true;
    req->timeout.sec = 5;

    switch_request_pending_ = true;
    controller_switch_client_->async_send_request(
      req,
      [this](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future) {
        switch_request_pending_ = false;
        auto resp = future.get();
        if (resp->ok) {
          velocity_controller_active_ = true;
          RCLCPP_INFO(this->get_logger(), "Velocity controller activated, trajectory controller deactivated.");
        } else {
          RCLCPP_WARN(this->get_logger(), "Failed to switch to arm_velocity_controller.");
        }
      });
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!latest_obj_pose_) {
      RCLCPP_INFO(this->get_logger(), "✅ First /object_pose received! Tracking started.");
    }
    latest_obj_pose_ = msg;
  }

  void statusCallback(const std_msgs::msg::Int8::SharedPtr msg)
  {
    if (msg->data == last_servo_status_) {
      return;
    }
    last_servo_status_ = msg->data;

    switch (msg->data) {
      case 0:
        RCLCPP_INFO(this->get_logger(), "Servo status: 0 (No warnings)");
        break;
      case 1:
        RCLCPP_WARN(this->get_logger(), "Servo status: 1 (Approaching singularity, decelerating)");
        break;
      case 2:
        RCLCPP_ERROR(this->get_logger(), "Servo status: 2 (Singularity halt)");
        break;
      case 3:
        RCLCPP_WARN(this->get_logger(), "Servo status: 3 (Approaching collision, decelerating)");
        break;
      case 4:
        RCLCPP_ERROR(this->get_logger(), "Servo status: 4 (Collision halt)");
        break;
      case 5:
        RCLCPP_ERROR(this->get_logger(), "Servo status: 5 (Joint bound halt)");
        break;
      case 6:
        RCLCPP_WARN(this->get_logger(), "Servo status: 6 (Leaving singularity, decelerating)");
        break;
      default:
        RCLCPP_WARN(this->get_logger(), "Servo status: %d (Unknown code)", msg->data);
        break;
    }
  }

  void controlLoop()
  {
    if (!latest_obj_pose_) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "⏳ Still waiting for /object_pose message...");
      return;
    }

    if (twist_pub_->get_subscription_count() == 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                           "No subscriber on /servo_node/delta_twist_cmds yet.");
    }

    // 1. 获取当前 TCP (Tool_Link) 在 base_link 下的真实位姿
    geometry_msgs::msg::TransformStamped t_base_tool;
    try {
      t_base_tool = tf_buffer_->lookupTransform("base_link", "Tool_Link", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "⚠️ TF Error (Tool_Link): %s", ex.what());
      return; // TF 还未准备好或连接中断，跳过这个周期
    }

    // 2. 将 object_pose 转换到 base_link
    geometry_msgs::msg::PoseStamped object_pose_base;
    try {
      tf_buffer_->transform(*latest_obj_pose_, object_pose_base, "base_link");
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "⚠️ TF Error (object_pose): %s", ex.what());
      return;
    }

    // 3. 计算期望的 TCP 目标位姿: T_BASE_TCP_des = T_BASE_OBJ * T_OBJ_TCP
    Eigen::Isometry3d T_BASE_OBJ;
    tf2::fromMsg(object_pose_base.pose, T_BASE_OBJ);
    Eigen::Isometry3d T_BASE_TCP_des = T_BASE_OBJ * buildTcpOffset();

    // 4. 获取当前 TCP 位姿: T_BASE_TCP_curr
    Eigen::Isometry3d T_BASE_TCP_curr = tf2::transformToEigen(t_base_tool);

    // 计算位置误差: 位置向量差
    Eigen::Vector3d pos_error = T_BASE_TCP_des.translation() - T_BASE_TCP_curr.translation();
    
    // 计算姿态误差: 旋转差矩阵 -> 轴角表示
    Eigen::Matrix3d R_error = T_BASE_TCP_des.linear() * T_BASE_TCP_curr.linear().transpose();
    Eigen::AngleAxisd angle_axis(R_error);
    
    // 防止极其微小的0角度导致轴计算除以0出现nan
    Eigen::Vector3d rot_error = Eigen::Vector3d::Zero();
    if (std::abs(angle_axis.angle()) > 1e-5) {
      rot_error = angle_axis.axis() * angle_axis.angle();
    }

    // --------- 截止误差带（防抖动）---------
    if (pos_error.norm() < 0.004 && rot_error.norm() < 0.08) {
      // 误差够小就发 0 速度停下
      auto stop_msg = geometry_msgs::msg::TwistStamped();
      stop_msg.header.stamp = this->now();
      stop_msg.header.frame_id = "base_link";
      twist_pub_->publish(stop_msg);
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "🎯 Target reached within tolerance. Holding position.");
      return;
    }

    // 5. 比例控制 (P控制) —— 适度提高跟踪速度，避免被过低限速切成一段一段
    double kp_pos = 0.9;
    double kp_rot = 0.6;

    auto twist_msg = geometry_msgs::msg::TwistStamped();
    twist_msg.header.stamp = this->now();
    twist_msg.header.frame_id = "base_link"; // Servo 要求指令参照的坐标系

    // 限制最大平动速度(m/s)和角速度(rad/s)，防止刚启动误差很大时速度过快
    auto clip_speed = [](double val, double max_val) {
      return std::max(-max_val, std::min(max_val, val));
    };

    Eigen::Vector3d linear_cmd(
      clip_speed(kp_pos * pos_error.x(), 0.08),
      clip_speed(kp_pos * pos_error.y(), 0.08),
      clip_speed(kp_pos * pos_error.z(), 0.08));

    Eigen::Vector3d angular_cmd(
      clip_speed(kp_rot * rot_error.x(), 0.18),
      clip_speed(kp_rot * rot_error.y(), 0.18),
      clip_speed(kp_rot * rot_error.z(), 0.18));

    twist_msg.twist.linear.x = linear_cmd.x();
    twist_msg.twist.linear.y = linear_cmd.y();
    twist_msg.twist.linear.z = linear_cmd.z();

    twist_msg.twist.angular.x = angular_cmd.x();
    twist_msg.twist.angular.y = angular_cmd.y();
    twist_msg.twist.angular.z = angular_cmd.z();

    // 6. 发布控制指令给 MoveIt Servo
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
      "🚀 Tracking... Error Norm [Pos: %.3fm, Rot: %.3frad]", pos_error.norm(), rot_error.norm());
    twist_pub_->publish(twist_msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr start_timer_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr controller_switch_client_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  geometry_msgs::msg::PoseStamped::SharedPtr latest_obj_pose_;
  bool servo_started_{ false };
  bool start_request_pending_{ false };
  bool switch_request_pending_{ false };
  bool velocity_controller_active_{ false };
  bool shutdown_handled_{ false };
  int8_t last_servo_status_{ -1 };
};

int main(int argc, char **argv)
{
  rclcpp::InitOptions init_options;
  rclcpp::init(argc, argv, init_options, rclcpp::SignalHandlerOptions::None);
  auto node = std::make_shared<VisualServoNode>();

  const auto previous_sigint_handler = std::signal(SIGINT, handleSignal);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  while (rclcpp::ok() && !g_visual_servo_stop_requested.load()) {
    executor.spin_some();
    rclcpp::sleep_for(std::chrono::milliseconds(20));
  }

  node->safeStopAndRestoreController();
  executor.remove_node(node);
  std::signal(SIGINT, previous_sigint_handler);
  rclcpp::shutdown();
  return 0;
}
