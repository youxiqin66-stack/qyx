/**
 * @file bottle_visualizer.cpp
 * @brief 红瓶插入任务的「虚拟映射」可视化节点
 *
 * 订阅 /object_pose，做两件事：
 *   1. 把红瓶 STL 作为碰撞物体加进 MoveIt 规划场景（RViz 里显示瓶子在
 *      跟踪位姿处的虚拟映射，同时为 approach 阶段提供避障）；
 *   2. 计算「上方对齐」「插入」两个 TCP 目标位姿，用 RViz 坐标轴标出来，
 *      让你在真正执行前先看清整条插入链路的虚拟映射。
 *
 * 与 object_visualizer.cpp 的区别：额外发布两个目标位姿轴。
 * 本节点只显示、不动机械臂。
 */

#include <array>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <Eigen/Geometry>

#include "hello_moveit/bottle_offsets.h"

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

class BottleVisualizer : public rclcpp::Node
{
public:
  BottleVisualizer() : Node("bottle_visualizer")
  {
    this->declare_parameter<std::string>("mesh_file", "");
    this->declare_parameter<std::string>("object_id", "bottle");
    this->declare_parameter<std::string>("planning_frame", "base_link");
    this->declare_parameter<double>("mesh_scale", 0.001);

    mesh_file_ = this->get_parameter("mesh_file").as_string();
    object_id_ = this->get_parameter("object_id").as_string();
    planning_frame_ = this->get_parameter("planning_frame").as_string();
    mesh_scale_ = this->get_parameter("mesh_scale").as_double();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    planning_scene_interface_ =
        std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/bottle_visualizer/target_axes", 10);

    pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/object_pose", 10,
        std::bind(&BottleVisualizer::poseCallback, this, std::placeholders::_1));

    if (mesh_file_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "mesh_file parameter is required!");
      throw std::runtime_error("mesh_file parameter is required");
    }
    loadMesh();

    RCLCPP_INFO(this->get_logger(), "Bottle Visualizer started. Object ID: %s, mesh: %s",
                object_id_.c_str(), mesh_file_.c_str());
  }

private:
  void loadMesh()
  {
    shapes::Mesh* mesh = shapes::createMeshFromResource("file://" + mesh_file_);
    if (mesh == nullptr) {
      throw std::runtime_error("Failed to load mesh file: " + mesh_file_);
    }
    if (mesh->vertex_count == 0 || mesh->triangle_count == 0) {
      delete mesh;
      throw std::runtime_error("Mesh file is empty or invalid: " + mesh_file_);
    }
    if (mesh_scale_ != 1.0) {
      for (unsigned int i = 0; i < mesh->vertex_count * 3; ++i) {
        mesh->vertices[i] *= mesh_scale_;
      }
    }
    shape_msgs::msg::Mesh mesh_msg;
    shapes::ShapeMsg shape_msg;
    if (!shapes::constructMsgFromShape(mesh, shape_msg)) {
      delete mesh;
      throw std::runtime_error("Failed to convert mesh to ROS message");
    }
    mesh_msg_ = boost::get<shape_msgs::msg::Mesh>(shape_msg);
    delete mesh;
    RCLCPP_INFO(this->get_logger(), "Loaded mesh with %zu triangles", mesh_msg_.triangles.size());
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped base_pose;
    try {
      if (!tf_buffer_->canTransform(planning_frame_, msg->header.frame_id,
                                    msg->header.stamp, rclcpp::Duration::from_seconds(0.1))) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Waiting for TF from %s to %s",
                             msg->header.frame_id.c_str(), planning_frame_.c_str());
        return;
      }
      tf_buffer_->transform(*msg, base_pose, planning_frame_);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "TF transform failed: %s", ex.what());
      return;
    }

    updateCollisionObject(base_pose);
    publishTargetAxes(base_pose);
  }

  void updateCollisionObject(const geometry_msgs::msg::PoseStamped& base_pose)
  {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = planning_frame_;
    collision_object.header.stamp = this->now();
    collision_object.id = object_id_;
    collision_object.meshes.push_back(mesh_msg_);
    collision_object.mesh_poses.push_back(base_pose.pose);
    collision_object.operation = collision_object.ADD;
    planning_scene_interface_->applyCollisionObject(collision_object);
  }

  void publishTargetAxes(const geometry_msgs::msg::PoseStamped& base_pose)
  {
    Eigen::Isometry3d T_BASE_OBJ;
    tf2::fromMsg(base_pose.pose, T_BASE_OBJ);

    const Eigen::Isometry3d T_APPROACH = T_BASE_OBJ * bottle_offset::approachOffset();
    const Eigen::Isometry3d T_INSERT = T_BASE_OBJ * bottle_offset::insertOffset();

    publishFrame("approach", T_APPROACH, { 1.0f, 0.6f, 0.0f, 1.0f });  // 橙色：上方对齐
    publishFrame("insert", T_INSERT, { 0.0f, 0.8f, 1.0f, 1.0f });      // 青蓝：插入到位
  }

  // 在某个位姿处画一组 X/Y/Z 坐标轴（三个箭头），便于观察目标姿态
  void publishFrame(const std::string& ns, const Eigen::Isometry3d& frame_pose,
                    const std::array<float, 4>& color)
  {
    // +X 轴（红）——箭头默认沿自身 +X 方向
    publishArrow(ns, 0, frame_pose, Eigen::Isometry3d::Identity(),
                 { 1.0f, 0.2f, 0.2f, 1.0f });
    // +Y 轴（绿）——绕自身 +Z 转 90°
    Eigen::Isometry3d rot_y = Eigen::Isometry3d::Identity();
    rot_y.linear() = Eigen::AngleAxisd(kPi / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    publishArrow(ns, 1, frame_pose, rot_y, { 0.2f, 1.0f, 0.2f, 1.0f });
    // +Z 轴（蓝）——绕自身 +Y 转 -90°
    Eigen::Isometry3d rot_z = Eigen::Isometry3d::Identity();
    rot_z.linear() = Eigen::AngleAxisd(-kPi / 2.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
    publishArrow(ns, 2, frame_pose, rot_z, { 0.3f, 0.4f, 1.0f, 1.0f });

    // 目标原点处再画一个小球
    visualization_msgs::msg::Marker sphere;
    sphere.header.frame_id = planning_frame_;
    sphere.header.stamp = this->now();
    sphere.ns = ns;
    sphere.id = 3;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.pose = tf2::toMsg(frame_pose);
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.01;
    sphere.color.r = color[0];
    sphere.color.g = color[1];
    sphere.color.b = color[2];
    sphere.color.a = color[3];
    marker_pub_->publish(sphere);
  }

  void publishArrow(const std::string& ns, int id, const Eigen::Isometry3d& frame_pose,
                    const Eigen::Isometry3d& axis_rot, const std::array<float, 4>& rgba)
  {
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = planning_frame_;
    arrow.header.stamp = this->now();
    arrow.ns = ns;
    arrow.id = id;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.pose = tf2::toMsg(frame_pose * axis_rot);
    arrow.scale.x = 0.004;  // 杆直径
    arrow.scale.y = 0.009;  // 箭头直径
    arrow.scale.z = 0.02;   // 箭头长度
    arrow.color.r = rgba[0];
    arrow.color.g = rgba[1];
    arrow.color.b = rgba[2];
    arrow.color.a = rgba[3];
    marker_pub_->publish(arrow);
  }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  std::string mesh_file_;
  std::string object_id_;
  std::string planning_frame_;
  double mesh_scale_;
  shape_msgs::msg::Mesh mesh_msg_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BottleVisualizer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
