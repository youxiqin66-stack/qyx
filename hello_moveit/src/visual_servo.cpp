#include <atomic>
#include <chrono>
#include <csignal>
#include <string>

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

#include "hello_moveit/bottle_offsets.h"

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

// 单轴匀速模型（CV）卡尔曼滤波器：从带噪声的离散位置测量，估计位置 + 速度。
struct AxisKalman
{
  double pos = 0.0;
  double vel = 0.0;
  double p00 = 1.0, p01 = 0.0, p11 = 1.0;  // 状态协方差
  bool initialized = false;

  // 匀速模型预测 dt 秒（每个控制周期调用；CV 模型只推进位置，速度不变）
  void predict(double dt, double q)
  {
    pos += vel * dt;
    // P = F P F^T + Q，Q 采用离散白噪声加速度模型
    p00 += 2.0 * dt * p01 + dt * dt * p11 + q * dt * dt * dt / 3.0;
    p01 += dt * p11 + q * dt * dt / 2.0;
    p11 += q * dt;
  }

  // 用测量 z（噪声方差 R）修正（收到新位姿时调用）
  void update(double z, double R)
  {
    if (!initialized) {
      pos = z;
      vel = 0.0;
      p00 = R;
      p01 = 0.0;
      p11 = 1.0;
      initialized = true;
      return;
    }
    // 先保存更新前的协方差：下面三式全部基于 P⁻，不能使用已被修改过的元素
    const double p00_prev = p00;
    const double p01_prev = p01;
    const double p11_prev = p11;

    const double S = p00_prev + R;     // 新息协方差
    const double k0 = p00_prev / S;    // 位置卡尔曼增益
    const double k1 = p01_prev / S;    // 速度卡尔曼增益
    const double y = z - pos;          // 新息

    pos += k0 * y;
    vel += k1 * y;

    // P⁺ = (I - K H) P⁻
    p00 = (1.0 - k0) * p00_prev;
    p01 = (1.0 - k0) * p01_prev;
    p11 = p11_prev - k1 * p01_prev;
  }
};

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

    // 红瓶跟随（默认）：Tool_Link 原点停在瓶口沿法线方向 follow_standoff_ 处，
    // 只跟随瓶子的平移、不下插。target_mode=triangle 退回原三角块抓取偏置。
    // vertical_mode=bottle（默认）跟随瓶子自身轴向：Tool_Link +X 朝瓶子 -Z 插入；
    // vertical_mode=world 固定世界竖直（只适合瓶子正放，姿态不随位姿噪声抖）。
    target_mode_ = this->declare_parameter<std::string>("target_mode", "triangle");
    vertical_mode_ = this->declare_parameter<std::string>("vertical_mode", "bottle");
    follow_standoff_ = this->declare_parameter<double>("follow_standoff", 0.020);
    // roll_mode=free（默认）：只锁杆轴向，roll 取离当前最近；
    // roll_mode=fixed：用完整 R_rod（roll 也定死）。
    roll_mode_ = this->declare_parameter<std::string>("roll_mode", "free");
    RCLCPP_INFO(this->get_logger(),
                "target_mode=%s vertical_mode=%s follow_standoff=%.0fmm roll_mode=%s",
                target_mode_.c_str(), vertical_mode_.c_str(),
                follow_standoff_ * 1000.0, roll_mode_.c_str());

    // 控制周期 10Hz（100ms），与 Servo 的 publish_period 0.10s 匹配
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
    new_pose_pending_ = true;
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
    // 测量本次与上次控制循环的真实时间间隔，供卡尔曼滤波预测和 D 项使用
    const auto now = std::chrono::steady_clock::now();
    const double dt = has_last_control_time_
                        ? std::chrono::duration<double>(now - last_control_time_).count()
                        : 0.10;
    last_control_time_ = now;
    has_last_control_time_ = true;

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

    // 2.5 卡尔曼滤波：估计物体在 base_link 下的速度（前馈用）
    const double kf_R = 1e-5;   // 测量噪声方差 [m^2]，≈3mm 标准差，对应视觉位姿噪声
    const double kf_q = 1.0;    // 过程噪声（加速度方差），越大越信测量、响应越快
    const double kf_dt = std::max(0.001, std::min(dt, 0.5));
    kf_x_.predict(kf_dt, kf_q);
    kf_y_.predict(kf_dt, kf_q);
    kf_z_.predict(kf_dt, kf_q);
    if (new_pose_pending_) {
      kf_x_.update(object_pose_base.pose.position.x, kf_R);
      kf_y_.update(object_pose_base.pose.position.y, kf_R);
      kf_z_.update(object_pose_base.pose.position.z, kf_R);
      new_pose_pending_ = false;
    }
    const Eigen::Vector3d obj_vel_est(kf_x_.vel, kf_y_.vel, kf_z_.vel);

    // 3. 计算期望的 TCP 目标位姿
    //    物体原点位置用 KF 估计值（与测量同处 base_link），姿态仍取原始测量位姿。
    Eigen::Isometry3d T_BASE_OBJ;
    tf2::fromMsg(object_pose_base.pose, T_BASE_OBJ);
    T_BASE_OBJ.translation() = Eigen::Vector3d(kf_x_.pos, kf_y_.pos, kf_z_.pos);

    Eigen::Isometry3d T_BASE_TCP_des = Eigen::Isometry3d::Identity();
    if (target_mode_ == "bottle") {
      // 瓶口中心（建模系 (25,25,130)mm）在 base_link 下的位置
      const Eigen::Vector3d p_mouth = T_BASE_OBJ * Eigen::Vector3d(
          bottle_offset::kOpeningX, bottle_offset::kOpeningY, bottle_offset::kRimZ);

      const bool follow_bottle_axis = (vertical_mode_ == "bottle");
      const Eigen::Vector3d axis = follow_bottle_axis
          ? Eigen::Vector3d(T_BASE_OBJ.linear().col(2))
          : Eigen::Vector3d::UnitZ();

      // Tool_Link 原点 = 瓶口沿法线方向 follow_standoff 处（默认 2cm）。
      const Eigen::Matrix3d R_rod = bottle_offset::buildRodVerticalDown().linear();
      const Eigen::Matrix3d R_nominal = follow_bottle_axis
          ? Eigen::Matrix3d(T_BASE_OBJ.linear() * R_rod)
          : R_rod;
      T_BASE_TCP_des.linear() = R_nominal;   // roll 初值，后面按最近 roll 重算
      T_BASE_TCP_des.translation() = p_mouth + axis * follow_standoff_;
    } else {
      // triangle：原三角块打磨/抓取偏置 T_BASE_TCP_des = T_BASE_OBJ * T_OBJ_TCP
      T_BASE_TCP_des = T_BASE_OBJ * buildTcpOffset();
    }

    // 4. 获取当前 TCP 位姿: T_BASE_TCP_curr
    Eigen::Isometry3d T_BASE_TCP_curr = tf2::transformToEigen(t_base_tool);

    if (target_mode_ == "bottle" && roll_mode_ != "fixed") {
      // roll_mode=free：只锁名义姿态的 X 轴方向（Tool_Link +X = 瓶子 -Z），
      // 绕杆轴的 roll 取离当前姿态最近的解；roll_mode=fixed 时保留完整 R_rod。
      // 注意别用瓶子 +Z——+X 要朝瓶子 -Z，差 180°。
      const Eigen::Vector3d x_nominal = T_BASE_TCP_des.linear().col(0);
      const Eigen::Vector3d x_cur = T_BASE_TCP_curr.linear().col(0);
      const Eigen::Quaterniond q =
          Eigen::Quaterniond::FromTwoVectors(x_cur, x_nominal);
      T_BASE_TCP_des.linear() = Eigen::Matrix3d(q * T_BASE_TCP_curr.linear());
    }

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

    // 5. PD 控制：P 项为主，D 项暂置 0 备用（后续调参直接改 kd_*）
    const double kp_pos = 1.0;
    const double kp_rot = 0.6;
    const double kd_pos = 0.0;
    const double kd_rot = 0.0;

    // --------- 截止误差带（防抖动）---------
    if (pos_error.norm() < 0.004 && rot_error.norm() < 0.08) {
      // 误差够小就发 0 速度停下；同时清掉上一周期误差，避免目标再次移动时微分项用旧值跳变
      has_prev_error_ = false;
      auto stop_msg = geometry_msgs::msg::TwistStamped();
      stop_msg.header.stamp = this->now();
      stop_msg.header.frame_id = "base_link";
      twist_pub_->publish(stop_msg);
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "🎯 Target reached within tolerance. Holding position.");
      return;
    }

    auto twist_msg = geometry_msgs::msg::TwistStamped();
    twist_msg.header.stamp = this->now();
    twist_msg.header.frame_id = "base_link"; // Servo 要求指令参照的坐标系

    // 限制最大平动速度(m/s)和角速度(rad/s)，防止刚启动误差很大时速度过快
    auto clip_speed = [](double val, double max_val) {
      return std::max(-max_val, std::min(max_val, val));
    };

    // 速度前馈：直接叠加物体速度，臂不用等误差涨起来才提速；前馈项单独限幅防噪声尖峰
    const double vel_ff_gain = 1.0;
    const Eigen::Vector3d v_ff(
      clip_speed(vel_ff_gain * obj_vel_est.x(), 0.15),
      clip_speed(vel_ff_gain * obj_vel_est.y(), 0.15),
      clip_speed(vel_ff_gain * obj_vel_est.z(), 0.15));

    Eigen::Vector3d linear_cmd = kp_pos * pos_error + v_ff;
    Eigen::Vector3d angular_cmd = kp_rot * rot_error;

    // D 项：kd_* 目前为 0，代码保留供后续使用
    if (has_prev_error_) {
      const double dt_clamped = std::max(dt, 0.001);
      linear_cmd += kd_pos * (pos_error - prev_pos_error_) / dt_clamped;
      angular_cmd += kd_rot * (rot_error - prev_rot_error_) / dt_clamped;
    }
    prev_pos_error_ = pos_error;
    prev_rot_error_ = rot_error;
    has_prev_error_ = true;

    Eigen::Vector3d linear_cmd_clipped(
      clip_speed(linear_cmd.x(), 0.15),
      clip_speed(linear_cmd.y(), 0.15),
      clip_speed(linear_cmd.z(), 0.15));

    // 角速度上限 0.35 rad/s
    Eigen::Vector3d angular_cmd_clipped(
      clip_speed(angular_cmd.x(), 0.35),
      clip_speed(angular_cmd.y(), 0.35),
      clip_speed(angular_cmd.z(), 0.35));

    twist_msg.twist.linear.x = linear_cmd_clipped.x();
    twist_msg.twist.linear.y = linear_cmd_clipped.y();
    twist_msg.twist.linear.z = linear_cmd_clipped.z();

    twist_msg.twist.angular.x = angular_cmd_clipped.x();
    twist_msg.twist.angular.y = angular_cmd_clipped.y();
    twist_msg.twist.angular.z = angular_cmd_clipped.z();

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
  std::string target_mode_{ "bottle" };
  std::string vertical_mode_{ "bottle" };
  std::string roll_mode_{ "free" };
  double follow_standoff_{ 0.020 };
  Eigen::Vector3d prev_pos_error_{ Eigen::Vector3d::Zero() };
  Eigen::Vector3d prev_rot_error_{ Eigen::Vector3d::Zero() };
  bool has_prev_error_{ false };
  std::chrono::steady_clock::time_point last_control_time_{};
  bool has_last_control_time_{ false };
  AxisKalman kf_x_, kf_y_, kf_z_;
  bool new_pose_pending_{ false };
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
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }

  node->safeStopAndRestoreController();
  executor.remove_node(node);
  std::signal(SIGINT, previous_sigint_handler);
  rclcpp::shutdown();
  return 0;
}
