#include <memory>
#include <future>
#include <thread>
#include <string>
#include <vector>
#include <cmath>
#include <unordered_map>

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
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include "hello_moveit/bottle_offsets.h"

/**
 * @brief 【对齐节点】订阅一次 /object_pose，算出 TCP 目标位姿，规划并执行。
 *
 * 有两个 target_mode，和目标模型对应：
 *   - bottle（默认，红瓶跟随用）：目标 = 瓶口中心沿瓶口法线抬高
 *       follow_standoff，**Tool_Link 原点就在该点**，**+X 指向瓶子的 -Z**
 *       （瓶子斜放时法线一起倾斜）。
 *     这个位姿与 visual_servo 里 target_mode=bottle 算出的位姿完全一致，
 *     所以对齐完成后直接起 visual_servo，误差≈0，不会被伺服拽着跳。
 *   - triangle（原三角块）：T_BASE_TCP = T_BASE_OBJ × T_OBJ_TCP，
 *     与 hello_moveit1 完全等价，保留在这里方便用同一入口切换。
 *
 * 用法：
 *   ros2 run hello_moveit hello_moveit                    # 红瓶，沿瓶口法线（默认）
 *   ros2 run hello_moveit hello_moveit -p vertical_mode:=world   # 瓶子正放时用世界竖直
 *   ros2 run hello_moveit hello_moveit -p target_mode:=triangle
 *   ros2 run hello_moveit hello_moveit1                   # 原三角块节点
 */

// ============================================================
// 三角块偏置（原 hello_moveit 的 T_OBJ_TCP，原样保留）
// ============================================================
Eigen::Isometry3d buildTcpOffsetTriangle()
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
// 控制器辅助函数
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

  // 与 visual_servo 保持同名同义的参数：对齐完直接起伺服，目标位姿一致。
  // 注意：NodeOptions 开了 automatically_declare_parameters_from_overrides，
  // 命令行 -p 传进来的参数在构造时就已声明，这里不能再 declare_parameter
  // （会抛 ParameterAlreadyDeclaredException），所以用 has_parameter 判断。
  auto get_string_param = [&node](const std::string& name, const std::string& def) {
    return node->has_parameter(name) ? node->get_parameter(name).as_string() : def;
  };
  auto get_double_param = [&node](const std::string& name, double def) {
    return node->has_parameter(name) ? node->get_parameter(name).as_double() : def;
  };

  const std::string target_mode = get_string_param("target_mode", "bottle");
  // 默认跟随瓶子自身轴向：瓶子斜放时，Tool_Link +X 要朝瓶子 -Z 插入，
  // 而不是世界竖直（vertical_mode=world 只适合瓶子正放的情况）。
  const std::string vertical_mode = get_string_param("vertical_mode", "bottle");
  const double follow_standoff = get_double_param("follow_standoff", 0.020);
  // roll_mode=free（默认）：只锁杆轴向，绕杆轴的 roll 取离当前最近的解；
  // roll_mode=fixed：用完整 R_rod，roll 也定死（joint5 限位时可能 IK 无解）。
  const std::string roll_mode = get_string_param("roll_mode", "free");

  RCLCPP_INFO(logger, "target_mode=%s vertical_mode=%s follow_standoff=%.0fmm roll_mode=%s",
              target_mode.c_str(), vertical_mode.c_str(),
              follow_standoff * 1000.0, roll_mode.c_str());

  // 使用多线程执行器：同时处理 MoveIt 回调和话题订阅
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // 对齐是离散规划，必须用轨迹控制器
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

  // 规划时长参数化。原来 50s × 30 次会把 CPU 打满、RViz 看起来像卡死，
  // 失败也要等很久才返回。默认改成 5s × 10 次，失败能快速反馈。
  const double planning_time = get_double_param("planning_time", 5.0);
  const int planning_attempts =
      node->has_parameter("planning_attempts")
          ? static_cast<int>(node->get_parameter("planning_attempts").as_int())
          : 5;
  move_group.setPlanningTime(planning_time);
  move_group.setNumPlanningAttempts(planning_attempts);
  move_group.setGoalPositionTolerance(0.005);   // 5 mm
  move_group.setGoalOrientationTolerance(0.2);  // 约 11.5°
  RCLCPP_INFO(logger, "planning_time=%.1fs attempts=%d",
              planning_time, planning_attempts);

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
  // 杆是回转体：只约束 Tool_Link 的 X 轴对准名义 X（瓶子 -Z），绕杆轴的 roll
  // 取离当前姿态最近的解。固定 roll 会因关节限位导致 IK/规划直接失败。
  // ============================================================
  auto const align_roll_to_current = [&move_group](const Eigen::Isometry3d& nominal) {
    const auto cur = move_group.getCurrentPose("Tool_Link");
    Eigen::Isometry3d T_cur;
    tf2::fromMsg(cur.pose, T_cur);

    const Eigen::Vector3d x_tgt = nominal.linear().col(0);  // 目标 X = 瓶口轴线
    const Eigen::Vector3d x_cur = T_cur.linear().col(0);    // 当前杆 X 方向
    const Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(x_cur, x_tgt);

    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = q * T_cur.linear();   // X 对准目标，Y/Z（roll）离当前最近
    T.translation() = nominal.translation();
    return T;
  };

  // ============================================================
  // TF2 - 用于将 /object_pose 变换到规划坐标系
  // ============================================================
  auto tf_buffer   = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  // ============================================================
  // 订阅 /object_pose，接收一次后进行规划执行
  // ============================================================
  RCLCPP_INFO(logger, "Waiting for /object_pose message...");
  draw_title("Waiting for object pose");
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
  // 将物体位姿变换到规划坐标系
  // ============================================================
  const std::string planning_frame = move_group.getPlanningFrame();

  geometry_msgs::msg::PoseStamped object_pose_base;
  try {
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
  // 计算 TCP 目标位姿
  // ============================================================
  Eigen::Isometry3d T_BASE_OBJ;
  tf2::fromMsg(object_pose_base.pose, T_BASE_OBJ);

  Eigen::Isometry3d T_BASE_TCP = Eigen::Isometry3d::Identity();
  if (target_mode == "bottle") {
    // 与 visual_servo.cpp 中 target_mode=bottle 的算法逐行一致：
    //   瓶口中心 p_mouth = T_BASE_OBJ * (kOpeningX, kOpeningY, kRimZ)
    //   Tool_Link 原点 = p_mouth + axis*follow_standoff
    //   axis = 世界 +Z（vertical_mode=world）或瓶子自身 +Z（vertical_mode=bottle）
    //   姿态 = Tool_Link +X 指向瓶子 -Z（= -axis；bottle 模式下随瓶子倾斜）
    const Eigen::Vector3d p_mouth = T_BASE_OBJ * Eigen::Vector3d(
        bottle_offset::kOpeningX, bottle_offset::kOpeningY, bottle_offset::kRimZ);

    const bool follow_bottle_axis = (vertical_mode == "bottle");
    const Eigen::Vector3d axis = follow_bottle_axis
        ? Eigen::Vector3d(T_BASE_OBJ.linear().col(2))
        : Eigen::Vector3d::UnitZ();

    // Tool_Link 原点 = 瓶口沿法线方向 standoff 处（默认 2cm）。
    const Eigen::Matrix3d R_rod = bottle_offset::buildRodVerticalDown().linear();
    const Eigen::Matrix3d R_nominal = follow_bottle_axis
        ? Eigen::Matrix3d(T_BASE_OBJ.linear() * R_rod)
        : R_rod;
    T_BASE_TCP.linear() = R_nominal;
    T_BASE_TCP.translation() = p_mouth + axis * follow_standoff;

    // roll_mode=free：只锁杆的轴向（X 对准 axis），roll 取离当前最近。
    // roll_mode=fixed：保留 R_rod 的完整 6D 姿态（roll 也定死）。
    if (roll_mode != "fixed") {
      T_BASE_TCP = align_roll_to_current(T_BASE_TCP);
    }

    RCLCPP_INFO(logger,
                "Bottle mouth in '%s': [%.3f, %.3f, %.3f], rod root target: [%.3f, %.3f, %.3f]",
                planning_frame.c_str(),
                p_mouth.x(), p_mouth.y(), p_mouth.z(),
                T_BASE_TCP.translation().x(),
                T_BASE_TCP.translation().y(),
                T_BASE_TCP.translation().z());
  } else {
    // triangle：与原 hello_moveit / hello_moveit1 完全一致
    T_BASE_TCP = T_BASE_OBJ * buildTcpOffsetTriangle();
  }

  // ============================================================
  // IK 预检（走 move_group 的 /compute_ik 服务）
  //
  // 为什么不用本节点的 RobotState::setFromIK：
  //   `ros2 run` 启动的节点上没有 robot_description_kinematics 参数，
  //   MoveGroupInterface 在本进程加载的 RobotModel 没有 IK 插件
  //   （日志 "No kinematics solver instantiated for group 'arm'"）。
  //   真正的 pick_ik 在 move_group 进程里，直接调它的服务即可。
  // ============================================================
  const bool ik_check_enabled = node->has_parameter("ik_check")
      ? node->get_parameter("ik_check").as_bool()
      : true;
  std::vector<double> q_goal;   // IK 解出来的关节角；有值时按关节空间目标规划
  if (ik_check_enabled) {
    const auto* jmg = move_group.getRobotModel()->getJointModelGroup("arm");
    auto ik_client = node->create_client<moveit_msgs::srv::GetPositionIK>("/compute_ik");

    if (!ik_client->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_WARN(logger,
                  "/compute_ik 服务不可用，跳过 IK 预检，直接按位姿目标规划。");
    } else {
      auto solve_ik = [&](const Eigen::Isometry3d& T, int timeout_sec) {
        std::vector<double> q;
        auto req = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
        req->ik_request.group_name = "arm";
        req->ik_request.ik_link_name = "Tool_Link";
        req->ik_request.pose_stamped.header.frame_id = planning_frame;
        req->ik_request.pose_stamped.header.stamp = node->now();
        req->ik_request.pose_stamped.pose = tf2::toMsg(T);
        // 让 move_group 在自己的规划场景里检查碰撞（瓶子是 object_visualizer 加的
        // 碰撞体）。这样 IK 有解基本就能规划过去，不会"有解但目标碰撞"。
        req->ik_request.avoid_collisions = true;
        req->ik_request.timeout = rclcpp::Duration(timeout_sec, 0);

        auto future = ik_client->async_send_request(req);
        if (future.wait_for(std::chrono::seconds(timeout_sec + 2)) != std::future_status::ready) {
          RCLCPP_WARN(logger, "/compute_ik 调用超时");
          return q;
        }
        const auto resp = future.get();
        if (resp->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
          return q;
        }

        // 按本组变量顺序取出关节角
        std::unordered_map<std::string, double> by_name;
        const auto& js = resp->solution.joint_state;
        for (size_t i = 0; i < js.name.size() && i < js.position.size(); ++i) {
          by_name[js.name[i]] = js.position[i];
        }
        for (const auto& name : jmg->getVariableNames()) {
          auto it = by_name.find(name);
          if (it == by_name.end()) {
            RCLCPP_WARN(logger, "/compute_ik 返回缺少关节 %s", name.c_str());
            q.clear();
            return q;
          }
          q.push_back(it->second);
        }
        return q;
      };

      // pick_ik 的 global 模式需要一点时间，给 2s
      q_goal = solve_ik(T_BASE_TCP, 2);
      if (q_goal.empty() && roll_mode != "fixed") {
        // 最近的 roll 无解时，绕杆轴按 30° 步长换 roll 再试（joint 限位可能只放过某些 roll）
        const Eigen::Vector3d rod_x = T_BASE_TCP.linear().col(0);
        for (int i = 1; i <= 12 && q_goal.empty(); ++i) {
          const double th = i * M_PI / 6.0;
          Eigen::Isometry3d T = T_BASE_TCP;
          T.linear() = Eigen::AngleAxisd(th, rod_x).toRotationMatrix() * T_BASE_TCP.linear();
          q_goal = solve_ik(T, 1);
          if (!q_goal.empty()) {
            T_BASE_TCP = T;
          }
        }
      }

      if (q_goal.empty()) {
        RCLCPP_ERROR(logger,
            "IK pre-check: NO solution for this target (position + 瓶口法线方向, roll_mode=%s). "
            "说明瓶子所在的位置/高度，当前臂以这个插入方向够不到。",
            roll_mode.c_str());
        RCLCPP_ERROR(logger,
            "本目标 Tool_Link 原点为 [%.3f, %.3f, %.3f]。"
            "+X 朝瓶子 -Z 时可用的经验范围约为 x ∈ [-0.26, 0.45]，"
            "z ∈ [-0.22, 0.27]（base_link 下，FK 采样）；超出则挪瓶子/换瓶位。",
            T_BASE_TCP.translation().x(), T_BASE_TCP.translation().y(),
            T_BASE_TCP.translation().z());
        rclcpp::shutdown();
        spinner.join();
        return 1;
      }
    }
  }

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
  move_group.setStartStateToCurrentState();
  if (!q_goal.empty()) {
    // 用 IK 解做关节空间目标（单点目标），比按位姿目标规划更容易成功。
    move_group.setJointValueTarget(q_goal);
    RCLCPP_INFO(logger, "Goal set: joint-space IK solution of the target pose.");
  } else {
    move_group.setPoseTarget(tcp_target_pose);
    RCLCPP_INFO(logger, "Goal set: pose target (IK pre-check disabled).");
  }

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
