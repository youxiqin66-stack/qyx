/**
 * @file bottle_insert.cpp
 * @brief 红瓶插孔节点：两端式「先到瓶口上方，再直线下插」
 *
 * 数学关系（与 hello_moveit.cpp 一致）：
 *   T_BASE_TCP = T_BASE_OBJ × T_OBJ_TCP
 * 其中 T_OBJ_TCP 用 bottle_offsets.h 里的 approachOffset() / insertOffset()。
 *
 * 流程（两端式，每段都先在 RViz 显示规划轨迹，等你点 Next 才执行）：
 *   阶段 1  订阅 /object_pose -> 转 base_link -> 计算上方对齐位姿
 *           -> 规划 -> RViz 显示轨迹 -> 确认 -> 执行（杆子移到瓶口正上方）
 *   阶段 2  移除瓶子碰撞体 -> 计算插入位姿 -> 规划 -> RViz 显示轨迹
 *           -> 确认 -> 执行（杆子直线插入瓶口）
 *
 * 动臂前两道确认都在 RVizVisualToolsGui 的 Next 按钮完成，与本包 hello_moveit 一致。
 */

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

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

#include "hello_moveit/bottle_offsets.h"

namespace
{
std::string getControllerState(const rclcpp::Node::SharedPtr& node,
                               const std::string& controller_name)
{
  auto client = node->create_client<controller_manager_msgs::srv::ListControllers>(
      "/controller_manager/list_controllers");
  if (!client->wait_for_service(std::chrono::seconds(5))) {
    return "";
  }
  auto req = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
  auto future = client->async_send_request(req);
  if (future.wait_for(std::chrono::seconds(6)) != std::future_status::ready) {
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

  bool already = true;
  for (const auto& c : activate) {
    if (getControllerState(node, c) != "active") {
      already = false;
      break;
    }
  }
  if (already) {
    for (const auto& c : deactivate) {
      const auto state = getControllerState(node, c);
      if (!state.empty() && state != "inactive") {
        already = false;
        break;
      }
    }
  }
  if (already) {
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
  if (!future.get()->ok) {
    RCLCPP_ERROR(logger, "Controller switch failed.");
    return false;
  }
  RCLCPP_INFO(logger, "Controller switch complete: activate=[%s] deactivate=[%s]",
              activate.empty() ? "" : activate.front().c_str(),
              deactivate.empty() ? "" : deactivate.front().c_str());
  return true;
}
}  // namespace

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto const node = std::make_shared<rclcpp::Node>(
      "bottle_insert",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto const logger = rclcpp::get_logger("bottle_insert");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // 离散规划用 trajectory controller（arm_controller），先切过去
  if (!switchControllers(node, { "arm_controller" }, { "arm_velocity_controller" })) {
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  // ==================== MoveIt 接口 ====================
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group = MoveGroupInterface(node, "arm");
  move_group.setPlanningTime(50);
  move_group.setNumPlanningAttempts(30);
  move_group.setGoalPositionTolerance(0.005);
  move_group.setGoalOrientationTolerance(0.2);

  // 用于阶段 2 前移除瓶子碰撞体
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // ==================== 可视化 ====================
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

  // ==================== TF2 ====================
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  // ==================== 订阅 /object_pose ====================
  RCLCPP_INFO(logger, "Waiting for /object_pose message...");
  draw_title("Bottle insert: waiting for object pose");
  visual_tools.trigger();

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

  if (pose_future.wait_for(std::chrono::seconds(30)) == std::future_status::timeout) {
    RCLCPP_ERROR(logger, "Timeout: no /object_pose received within 30 seconds!");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  const auto object_pose_msg = pose_future.get();
  RCLCPP_INFO(logger, "Received object pose in frame: %s",
              object_pose_msg.header.frame_id.c_str());

  // ==================== 变换到 base_link ====================
  const std::string planning_frame = move_group.getPlanningFrame();
  geometry_msgs::msg::PoseStamped object_pose_base;
  try {
    if (!tf_buffer->canTransform(planning_frame, object_pose_msg.header.frame_id,
                                 object_pose_msg.header.stamp,
                                 rclcpp::Duration::from_seconds(1.0))) {
      RCLCPP_ERROR(logger, "Cannot transform from '%s' to '%s'",
                   object_pose_msg.header.frame_id.c_str(), planning_frame.c_str());
      rclcpp::shutdown();
      spinner.join();
      return 1;
    }
    tf_buffer->transform(object_pose_msg, object_pose_base, planning_frame);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(logger, "TF transform failed: %s", ex.what());
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  Eigen::Isometry3d T_BASE_OBJ;
  tf2::fromMsg(object_pose_base.pose, T_BASE_OBJ);
  const Eigen::Isometry3d T_APPROACH = T_BASE_OBJ * bottle_offset::approachOffset();
  const Eigen::Isometry3d T_INSERT = T_BASE_OBJ * bottle_offset::insertOffset();

  RCLCPP_INFO(logger, "Approach target: [%.3f, %.3f, %.3f]",
              T_APPROACH.translation().x(), T_APPROACH.translation().y(),
              T_APPROACH.translation().z());
  RCLCPP_INFO(logger, "Insert   target: [%.3f, %.3f, %.3f]",
              T_INSERT.translation().x(), T_INSERT.translation().y(),
              T_INSERT.translation().z());

  // 只约束杆子 X 轴对准瓶口轴线，绕杆轴（roll）取离当前姿态最近的解。
  // 杆子是回转体，roll 无关紧要；固定 roll 反而会让 IK/规划因关节限位失败。
  auto const align_roll_to_current = [&move_group](const Eigen::Isometry3d& nominal) {
    const auto cur = move_group.getCurrentPose("Tool_Link");
    Eigen::Isometry3d T_cur;
    tf2::fromMsg(cur.pose, T_cur);

    const Eigen::Vector3d x_tgt = nominal.linear().col(0);  // 目标 X（瓶口轴线方向）
    const Eigen::Vector3d x_cur = T_cur.linear().col(0);    // 当前杆 X 方向
    Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(x_cur, x_tgt);

    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = (q * T_cur.linear());   // X 对齐目标，Y/Z（roll）取离当前最近
    T.translation() = nominal.translation();
    return T;
  };

  // ==================== 阶段 1：上方对齐 ====================
  const Eigen::Isometry3d T_APPROACH_ALIGNED = align_roll_to_current(T_APPROACH);

  visual_tools.publishAxisLabeled(T_APPROACH_ALIGNED, "Approach");
  draw_title("Stage 1/2: approach above opening");
  visual_tools.trigger();

  prompt("Press 'Next' to PLAN the approach (above opening)");
  move_group.setPoseTarget(tf2::toMsg(T_APPROACH_ALIGNED));
  moveit::planning_interface::MoveGroupInterface::Plan plan_approach;
  const bool approach_ok = static_cast<bool>(move_group.plan(plan_approach));
  if (!approach_ok) {
    draw_title("Approach planning failed!");
    visual_tools.trigger();
    RCLCPP_ERROR(logger, "Approach planning failed.");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }
  draw_trajectory(plan_approach.trajectory_);
  visual_tools.trigger();

  prompt("Press 'Next' to EXECUTE the approach");
  move_group.execute(plan_approach);
  RCLCPP_INFO(logger, "Approach executed. Rod is now above the opening.");

  // ==================== 阶段 2：直线下插 ====================
  // 移除瓶子碰撞体，否则杆子进入瓶口会被判为碰撞
  planning_scene_interface.removeCollisionObjects({ "bottle" });
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  const Eigen::Isometry3d T_INSERT_ALIGNED = align_roll_to_current(T_INSERT);

  visual_tools.publishAxisLabeled(T_INSERT_ALIGNED, "Insert");
  draw_title("Stage 2/2: insert into opening");
  visual_tools.trigger();

  prompt("Press 'Next' to PLAN the insert (into opening)");
  move_group.setPoseTarget(tf2::toMsg(T_INSERT_ALIGNED));
  moveit::planning_interface::MoveGroupInterface::Plan plan_insert;
  const bool insert_ok = static_cast<bool>(move_group.plan(plan_insert));
  if (!insert_ok) {
    draw_title("Insert planning failed!");
    visual_tools.trigger();
    RCLCPP_ERROR(logger, "Insert planning failed.");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }
  draw_trajectory(plan_insert.trajectory_);
  visual_tools.trigger();

  prompt("Press 'Next' to EXECUTE the insert");
  move_group.execute(plan_insert);
  RCLCPP_INFO(logger, "Insert executed. Done.");

  draw_title("Done!");
  visual_tools.trigger();

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
