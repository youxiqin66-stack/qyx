#include <memory>
#include <future>
#include <thread>

#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <Eigen/Geometry>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

/**
 * @brief 订阅 /object_pose，根据 T_OBJ_TCP 偏置计算 TCP 目标位姿，然后规划并执行
 *
 * 数学关系：
 *   T_BASE_TCP = T_BASE_OBJ × T_OBJ_TCP
 *
 * T_OBJ_TCP（TCP 相对于物体坐标系的齐次变换矩阵，单位：米）：
 *   R = [ 0.7071   0  -0.7071 ]    t = [-0.0072 ]
 *       [-0.7071   0  -0.7071 ]        [ 0.04958]
 *       [ 0        1   0      ]        [ 0.01725]
 */

// ============================================================
// 全局：T_OBJ_TCP 偏置矩阵（固定不变）
// ============================================================
Eigen::Isometry3d buildTcpOffset()
{
  // 旋转矩阵部分（列主序赋值）
  Eigen::Matrix3d R;
  R <<  0.7071,  0.0, -0.7071,
       -0.7071,  0.0, -0.7071,
        0.0,     1.0,  0.0;

  // 平移部分（单位：米）
  Eigen::Vector3d t(-0.0072, 0.04958, 0.01725);

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = R;
  T.translation() = t;
  return T;
}

// ============================================================
// 主函数
// ============================================================
namespace
{
std::string getControllerState(
    const rclcpp::Node::SharedPtr& node,
    const std::string& controller_name)
{
  auto client = node->create_client<controller_manager_msgs::srv::ListControllers>(
      "/controller_manager/list_controllers");

  if (!client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node->get_logger(), "Controller list service /controller_manager/list_controllers not available.");
    return "";
  }

  auto req = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
  auto future = client->async_send_request(req);
  if (future.wait_for(std::chrono::seconds(6)) != std::future_status::ready) {
    RCLCPP_ERROR(node->get_logger(), "Timed out while listing controllers.");
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
    RCLCPP_ERROR(logger, "Controller switch service /controller_manager/switch_controller not available.");
    return false;
  }

  bool already_in_target_state = true;
  for (const auto& controller : activate) {
    if (getControllerState(node, controller) != "active") {
      already_in_target_state = false;
      break;
    }
  }
  if (already_in_target_state) {
    for (const auto& controller : deactivate) {
      const auto state = getControllerState(node, controller);
      if (!state.empty() && state != "inactive") {
        already_in_target_state = false;
        break;
      }
    }
  }
  if (already_in_target_state) {
    RCLCPP_INFO(logger, "Controllers already in desired state, skipping switch request.");
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
    RCLCPP_ERROR(logger, "Timed out while switching controllers.");
    return false;
  }

  const auto response = future.get();
  if (!response->ok) {
    bool reached_target_state = true;
    for (const auto& controller : activate) {
      if (getControllerState(node, controller) != "active") {
        reached_target_state = false;
        break;
      }
    }
    if (reached_target_state) {
      for (const auto& controller : deactivate) {
        const auto state = getControllerState(node, controller);
        if (!state.empty() && state != "inactive") {
          reached_target_state = false;
          break;
        }
      }
    }

    if (!reached_target_state) {
      RCLCPP_ERROR(logger, "Controller switch failed.");
      return false;
    }

    RCLCPP_WARN(logger, "Controller switch response was not ok, but controllers reached the desired state.");
  }

  RCLCPP_INFO(logger, "Controller switch complete. Activated: [%s], deactivated: [%s]",
              activate.empty() ? "" : activate.front().c_str(),
              deactivate.empty() ? "" : deactivate.front().c_str());
  return true;
}
}  // namespace

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto const logger = rclcpp::get_logger("hello_moveit");

  // 使用多线程执行器：同时处理 MoveIt 回调和话题订阅
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  if (!switchControllers(node, { "arm_controller" }, { "arm_velocity_controller" })) {
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  // ============================================================
  // MoveIt 接口初始化
  // ============================================================
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group = MoveGroupInterface(node, "arm");
  move_group.setPlanningTime(50);              // 增加规划超时时间到 30 秒
  move_group.setNumPlanningAttempts(30);           // 增加重试次数到 30 次，大幅提高随机采样规划器的成功率
  move_group.setGoalPositionTolerance(0.005);    // 放宽位置容差：10 mm (1 cm)
  move_group.setGoalOrientationTolerance(0.2);   // 放宽姿态容差：约 28°

  // 可视化工具
  auto visual_tools = moveit_visual_tools::MoveItVisualTools{
      node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
      move_group.getRobotModel()};
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  auto const draw_title = [&visual_tools](const std::string& text) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation().z() = 1.0;
    visual_tools.publishText(pose, text, rviz_visual_tools::WHITE,
                             rviz_visual_tools::XLARGE);
  };
  auto const prompt = [&visual_tools](const std::string& text) {
    visual_tools.prompt(text);
  };
  auto const draw_trajectory = [&visual_tools, &move_group,
      jmg = move_group.getRobotModel()->getJointModelGroup("arm")](auto const& traj) {
    auto const* ee = move_group.getRobotModel()->getLinkModel("Tool_Link");
    visual_tools.publishTrajectoryLine(traj, ee, jmg);
  };

  // ============================================================
  // TF2 - 用于将 /object_pose 变换到 base_link 坐标系
  // ============================================================
  auto tf_buffer   = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  // ============================================================
  // 固定的 T_OBJ_TCP 偏置
  // ============================================================
  const Eigen::Isometry3d T_OBJ_TCP = buildTcpOffset();

  // ============================================================
  // 订阅 /object_pose，接收一次后进行规划执行
  // ============================================================
  RCLCPP_INFO(logger, "Waiting for /object_pose message...");
  draw_title("Waiting for object pose");
  visual_tools.trigger();

  // 使用 promise/future 接收第一条消息
  std::promise<geometry_msgs::msg::PoseStamped> pose_promise;
  auto pose_future = pose_promise.get_future();
  bool received = false;

  auto subscription = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/object_pose", 10,
      [&](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (!received) {
          received = true;
          pose_promise.set_value(*msg);
        }
      });

  // 等待消息（最多 10 秒）
  if (pose_future.wait_for(std::chrono::seconds(10)) == std::future_status::timeout) {
    RCLCPP_ERROR(logger, "Timeout: no /object_pose received within 10 seconds!");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  const auto object_pose_msg = pose_future.get();
  RCLCPP_INFO(logger, "Received object pose in frame: %s",
              object_pose_msg.header.frame_id.c_str());

  // ============================================================
  // 将物体位姿变换到 base_link 坐标系
  // ============================================================
  const std::string planning_frame = move_group.getPlanningFrame();  // 通常是 base_footprint 或 base_link 

  geometry_msgs::msg::PoseStamped object_pose_base;
  try {
    // 等待 TF 可用（最多等 1 秒）
    if (!tf_buffer->canTransform(planning_frame,
                                  object_pose_msg.header.frame_id,
                                  object_pose_msg.header.stamp,
                                  rclcpp::Duration::from_seconds(1.0))) {
      RCLCPP_ERROR(logger, "Cannot transform from '%s' to '%s'",
                   object_pose_msg.header.frame_id.c_str(), planning_frame.c_str());
      rclcpp::shutdown();
      spinner.join();
      return 1;
    }
    tf_buffer->transform(object_pose_msg, object_pose_base, planning_frame);
    RCLCPP_INFO(logger, "Object pose in '%s': [%.3f, %.3f, %.3f]",
                planning_frame.c_str(),
                object_pose_base.pose.position.x,
                object_pose_base.pose.position.y,
                object_pose_base.pose.position.z);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(logger, "TF transform failed: %s", ex.what());
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  // ============================================================
  // 计算 TCP 目标位姿：T_BASE_TCP = T_BASE_OBJ × T_OBJ_TCP
  // ============================================================

  // 将 ROS Pose 消息转换为 Eigen::Isometry3d
  Eigen::Isometry3d T_BASE_OBJ;
  tf2::fromMsg(object_pose_base.pose, T_BASE_OBJ);

  // 核心计算
  const Eigen::Isometry3d T_BASE_TCP = T_BASE_OBJ * T_OBJ_TCP;

  // 将结果转换回 ROS Pose 消息
  const geometry_msgs::msg::Pose tcp_target_pose = tf2::toMsg(T_BASE_TCP);

  RCLCPP_INFO(logger,
              "TCP target pose: pos=[%.3f, %.3f, %.3f]  quat=[%.3f, %.3f, %.3f, %.3f]",
              tcp_target_pose.position.x,
              tcp_target_pose.position.y,
              tcp_target_pose.position.z,
              tcp_target_pose.orientation.x,
              tcp_target_pose.orientation.y,
              tcp_target_pose.orientation.z,
              tcp_target_pose.orientation.w);

  // ============================================================
  // 可视化目标位姿（在 RViz 中显示坐标轴）
  // ============================================================
  visual_tools.publishAxisLabeled(T_BASE_TCP, "TCP Target");
  draw_title("Planning to TCP target");
  visual_tools.trigger();

  // ============================================================
  // MoveIt 规划
  // ============================================================
  move_group.setPoseTarget(tcp_target_pose);

  prompt("Press 'Next' in RvizVisualToolsGui to start planning");
  draw_title("Planning...");
  visual_tools.trigger();

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const bool plan_ok = static_cast<bool>(move_group.plan(plan));

  if (!plan_ok) {
    draw_title("Planning Failed!");
    visual_tools.trigger();
    RCLCPP_ERROR(logger, "MoveIt planning failed!");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  // 显示轨迹预览
  draw_trajectory(plan.trajectory_);
  visual_tools.trigger();

  // ============================================================
  // 执行
  // ============================================================
  prompt("Press 'Next' in RvizVisualToolsGui to execute");
  draw_title("Executing...");
  visual_tools.trigger();

  move_group.execute(plan);

  draw_title("Done!");
  visual_tools.trigger();
  RCLCPP_INFO(logger, "Motion execution complete.");

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
