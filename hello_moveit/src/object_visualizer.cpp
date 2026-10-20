/**
 * @file object_visualizer.cpp
 * @brief 订阅 /object_pose 话题并通过 PlanningSceneInterface 在 RViz 中显示物体
 *
 * 功能概述：
 * 1. 从 /object_pose 话题接收物体的位姿信息（geometry_msgs::msg::PoseStamped）
 * 2. 使用 TF2 将位姿从相机坐标系转换到 MoveIt 的规划坐标系（如 base_link）
 * 3. 通过 MoveIt 的 PlanningSceneInterface 将物体添加到碰撞检测场景中
 * 4. 支持加载 STL mesh 文件或使用简单的盒子形状作为物体的视觉表示
 *
 * 适用场景：
 * - 实时物体跟踪与避障
 * - 机器人抓取规划前的场景重建
 * - 相机检测到的物体在 RViz 中的可视化
 */

#include <memory>                                                     // std::shared_ptr, std::make_shared
#include <rclcpp/rclcpp.hpp>                                          // ROS 2 C++ 客户端库（节点、日志、时间等）
#include <geometry_msgs/msg/pose_stamped.hpp>                         // PoseStamped 消息类型
#include <moveit/planning_scene_interface/planning_scene_interface.h> // MoveIt 场景接口
#include <moveit_msgs/msg/collision_object.hpp>                       // 碰撞物体消息定义
#include <shape_msgs/msg/mesh.hpp>                                    // Mesh 消息类型
#include <geometric_shapes/shape_operations.h>                        // 形状操作工具（加载 mesh）
#include <geometric_shapes/mesh_operations.h>                         // Mesh 操作工具
#include <tf2_ros/transform_listener.h>                               // TF2 监听器（订阅 /tf 和 /tf_static）
#include <tf2_ros/buffer.h>                                           // TF2 缓冲区（存储变换信息）
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>                    // TF2 与 geometry_msgs 的转换支持

/**
 * @class ObjectVisualizer
 * @brief 物体可视化节点类
 *
 * 该类实现了一个 ROS 2 节点，负责：
 * - 订阅物体位姿话题 /object_pose
 * - 执行坐标系变换（从相机坐标系到规划坐标系）
 * - 更新 MoveIt 规划场景中的碰撞物体
 *
 * 继承自 rclcpp::Node，是一个标准的 ROS 2 节点
 */
class ObjectVisualizer : public rclcpp::Node
{
public:
  /**
   * @brief 构造函数 - 初始化节点并设置所有必要的组件
   *
   * 初始化流程：
   * 1. 声明并读取 ROS 参数（mesh 文件路径、物体 ID、规划坐标系、缩放比例）
   * 2. 初始化 TF2 变换监听器（用于坐标系转换）
   * 3. 初始化 MoveIt PlanningSceneInterface（用于更新场景）
   * 4. 创建 /object_pose 话题的订阅器
   * 5. 如果指定了 mesh 文件，则加载 mesh；
   */
  ObjectVisualizer() : Node("object_visualizer")
  {
    // ==================== 参数声明 ====================
    // 这些参数可以通过 launch 文件或命令行传递

    // mesh_file: STL 文件的绝对路径，如 "/path/to/model.stl"
    // 留空则使用默认的盒子形状
    this->declare_parameter<std::string>("mesh_file", "");

    // object_id: 物体在 MoveIt 场景中的唯一标识符
    // 用于更新或删除特定物体
    this->declare_parameter<std::string>("object_id", "tracked_object");

    // planning_frame: MoveIt 规划坐标系的名称
    // 必须与 MoveIt 配置中的 planning_frame 一致（通常是 base_footprint 或 base_link）
    this->declare_parameter<std::string>("planning_frame", "base_link");

    // mesh_scale: mesh 模型的缩放比例
    // 例如 0.001 可将毫米单位的模型转换为米单位
    this->declare_parameter<double>("mesh_scale", 0.001);

    // ==================== 读取参数值 ====================
    mesh_file_ = this->get_parameter("mesh_file").as_string();
    object_id_ = this->get_parameter("object_id").as_string();
    planning_frame_ = this->get_parameter("planning_frame").as_string();
    mesh_scale_ = this->get_parameter("mesh_scale").as_double();

    // ==================== 初始化 TF2 组件 ====================
    // TF Buffer: 存储最近几秒内的所有坐标变换信息
    // 需要传入节点的时钟以保持时间同步
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

    // TF Listener: 自动订阅 /tf 和 /tf_static 话题
    // 并将接收到的变换信息填充到 tf_buffer_ 中
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ==================== 初始化 MoveIt 接口 ====================
    // PlanningSceneInterface: MoveIt 提供的场景管理接口
    // 用于添加、更新、删除碰撞物体
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    // ==================== 创建订阅器 ====================
    // 订阅 /object_pose 话题（类型：geometry_msgs::msg::PoseStamped）
    // 队列大小为 10，使用 poseCallback 作为回调函数
    pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/object_pose", 10,
        std::bind(&ObjectVisualizer::poseCallback, this, std::placeholders::_1));

    // ==================== 加载 Mesh（必须） ====================
    // mesh 文件是必需的，未指定则报错退出
    if (mesh_file_.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "Mesh file parameter is required but not specified!");
      RCLCPP_ERROR(this->get_logger(), "Please set 'mesh_file' parameter to a valid STL file path");
      throw std::runtime_error("mesh_file parameter is required");
    }

    // 加载 STL 文件
    loadMesh();

    // ==================== 打印启动信息 ====================
    RCLCPP_INFO(this->get_logger(), "Object Visualizer Node Started!");
    RCLCPP_INFO(this->get_logger(), "  Object ID: %s", object_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Planning Frame: %s", planning_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Mesh File: %s", mesh_file_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Mesh Scale: %.3f", mesh_scale_);
  }

private:
  /**
   * @brief 加载 STL mesh 文件并转换为 ROS 消息格式
   *
   * 工作流程：
   * 1. 使用 shapes::createMeshFromResource 加载 STL 文件（file:// 协议）
   * 2. 检查加载是否成功，失败则抛出异常
   * 3. 应用缩放比例（mesh_scale_）到所有顶点坐标
   * 4. 将 C++ shapes::Mesh 对象转换为 ROS shape_msgs::msg::Mesh 消息
   * 5. 存储转换后的消息并释放原始 mesh 内存
   *
   * 注意事项：
   * - createMeshFromResource 返回裸指针，需要手动 delete
   * - 支持 file:// 和 package:// 协议
   * - 顶点数据是连续存储的数组：[x1, y1, z1, x2, y2, z2, ...]
   * - 如果 mesh 加载失败，将抛出 runtime_error 异常
   */
  void loadMesh()
  {
    // 使用 file:// 协议加载本地 STL 文件
    // 也支持 package://package_name/path/to/file.stl 格式
    shapes::Mesh *mesh = shapes::createMeshFromResource("file://" + mesh_file_);

    // 检查加载是否成功
    if (mesh == nullptr)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to load mesh from: %s", mesh_file_.c_str());
      RCLCPP_ERROR(this->get_logger(), "Please check if the file exists and is a valid STL file");
      throw std::runtime_error("Failed to load mesh file: " + mesh_file_);
    }

    // 检查 mesh 是否有效（有顶点和三角形）
    if (mesh->vertex_count == 0 || mesh->triangle_count == 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Loaded mesh is empty (0 vertices or triangles)");
      delete mesh;
      throw std::runtime_error("Mesh file is empty or invalid: " + mesh_file_);
    }

    // 打印 mesh 信息用于调试
    // vertex_count: 顶点数量
    // triangle_count: 三角面片数量
    RCLCPP_INFO(this->get_logger(), "Successfully loaded mesh with %d vertices and %d triangles",
                mesh->vertex_count, mesh->triangle_count);

    // ==================== 应用缩放比例 ====================
    // 如果缩放比例不为 1.0，则对所有顶点坐标进行缩放
    // 这在单位转换时很有用（例如：毫米 -> 米，需要 scale = 0.001）
    if (mesh_scale_ != 1.0)
    {
      // 检查 vertices 指针是否有效
      if (mesh->vertices == nullptr)
      {
        RCLCPP_ERROR(this->get_logger(), "Mesh vertices pointer is null");
        delete mesh;
        throw std::runtime_error("Invalid mesh data: vertices pointer is null");
      }
      
      // mesh->vertices 是连续数组：[x1, y1, z1, x2, y2, z2, ...]
      // 因此总共有 vertex_count * 3 个浮点数
      for (unsigned int i = 0; i < mesh->vertex_count * 3; ++i)
      {
        mesh->vertices[i] *= mesh_scale_;
      }
      RCLCPP_INFO(this->get_logger(), "Applied mesh scale: %.3f", mesh_scale_);
    }

    // ==================== 转换为 ROS 消息格式 ====================
    // shapes::constructMsgFromShape 将 C++ 形状对象转换为 ROS 消息
    // ShapeMsg 是一个 variant 类型，可以存储多种形状消息（Box, Sphere, Mesh 等）
    shape_msgs::msg::Mesh mesh_msg;
    shapes::ShapeMsg shape_msg; // variant 类型容器
    
    if (!shapes::constructMsgFromShape(mesh, shape_msg))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to construct message from mesh shape");
      delete mesh;
      throw std::runtime_error("Failed to convert mesh to ROS message");
    }

    // 使用 boost::get 从 variant 中提取 shape_msgs::msg::Mesh 类型
    // 如果类型不匹配会抛出异常（但这里保证是 Mesh 类型）
    mesh_msg = boost::get<shape_msgs::msg::Mesh>(shape_msg);

    // 存储到成员变量，供后续 updateObject 使用
    mesh_msg_ = mesh_msg;

    // ==================== 清理内存 ====================
    // createMeshFromResource 返回的是裸指针，需要手动释放
    // 此时 mesh_msg_ 已经包含了所有必要数据的副本
    delete mesh;
  }

  /**
   * @brief /object_pose 话题的回调函数 - 处理接收到的物体位姿
   *
   * @param msg 接收到的 PoseStamped 消息，包含：
   *            - header.frame_id: 位姿所在的坐标系（通常是相机坐标系）
   *            - header.stamp: 位姿的时间戳
   *            - pose: 位姿信息（position + orientation）
   *
   * 工作流程：
   * 1. 检查从消息坐标系到规划坐标系的 TF 变换是否可用
   * 2. 执行坐标变换，将位姿转换到规划坐标系
   * 3. 调用 updateObject 更新 MoveIt 场景
   * 4. 定期打印位置信息（用于调试）
   *
   * 错误处理：
   * - 如果 TF 变换不可用，打印警告并跳过当前消息
   * - 如果变换失败（异常），打印错误信息并跳过
   * - 使用 RCLCPP_WARN_THROTTLE 限制日志频率（每 1000ms 最多一次）
   */
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // 用于存储转换后的位姿（在 planning_frame_ 坐标系中）
    geometry_msgs::msg::PoseStamped transformed_pose;

    try
    {
      // ==================== 检查 TF 变换是否可用 ====================
      // canTransform 检查：从 source frame（msg->header.frame_id）
      //                  到 target frame（planning_frame_）的变换是否存在

      if (!tf_buffer_->canTransform(planning_frame_, msg->header.frame_id,
                                    msg->header.stamp, rclcpp::Duration::from_seconds(0.1)))
      {
        // 使用 THROTTLE 宏限制日志频率，避免刷屏
        // 参数：logger, clock, period_ms（毫秒）, message
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Waiting for transform from %s to %s",
                             msg->header.frame_id.c_str(), planning_frame_.c_str());
        return; // 跳过当前消息，等待下一条
      }

      // ==================== 执行坐标变换 ====================
      // transform 函数执行实际的坐标变换
      // 将输入的 PoseStamped 从其 frame_id 转换到目标 frame
      //
      // 参数说明：
      // - *msg: 输入的 PoseStamped（const reference）
      // - transformed_pose: 输出的 PoseStamped（转换后的结果）
      // - planning_frame_: 目标坐标系名称
    
      tf_buffer_->transform(*msg, transformed_pose, planning_frame_);
    }
    catch (tf2::TransformException &ex)
    {

      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Transform failed: %s", ex.what());
      return;
    }

    // ==================== 更新 MoveIt 场景 ====================
    // 将转换后的位姿应用到 MoveIt 的碰撞检测场景中
    updateObject(transformed_pose);

    // ==================== 定期打印调试信息 ====================
    // 使用静态计数器实现简单的日志节流
    // 假设话题频率为 30Hz，每 30 条消息打印一次（即每秒一次）
    static int count = 0;
    if (++count % 30 == 0)
    {
      RCLCPP_INFO(this->get_logger(),
                  "Updated object '%s' at position [%.3f, %.3f, %.3f]",
                  object_id_.c_str(),
                  transformed_pose.pose.position.x,
                  transformed_pose.pose.position.y,
                  transformed_pose.pose.position.z);
    }
  }

  /**
   * @brief 更新 MoveIt 场景中的碰撞物体
   *
   * @param pose 物体在规划坐标系中的位姿（已经过 TF 转换）
   *
   * 功能说明：
   * 1. 构造 moveit_msgs::msg::CollisionObject 消息
   * 2. 设置物体的 mesh 形状和位姿
   * 3. 通过 PlanningSceneInterface 将物体应用到场景
   *
   * 操作类型：
   * - ADD: 添加新物体，或更新已存在的同 ID 物体
   * - 其他可选操作：REMOVE（删除）、APPEND（追加）、MOVE（移动）
   *
   * 注意事项：
   * - 频繁调用此函数会有性能影响，尤其是使用复杂 mesh 时
   * - MoveIt 的 PlanningSceneMonitor 会异步处理这些更新
   * - 物体会自动显示在 RViz 的 PlanningScene 插件中（需启用 Scene Robot）
   */
  void updateObject(const geometry_msgs::msg::PoseStamped &pose)
  {
    // ==================== 构造碰撞物体消息 ====================
    moveit_msgs::msg::CollisionObject collision_object;

    // 设置消息头
    // frame_id: 物体位姿所在的坐标系（必须与 pose 中的位姿对应）
    collision_object.header.frame_id = planning_frame_;

    // stamp: 时间戳，使用当前时间
    // 注意：也可以使用 pose.header.stamp 保持与输入消息一致
    collision_object.header.stamp = this->now();

    // id: 物体的唯一标识符
    // 如果场景中已存在相同 ID 的物体，则会被更新（覆盖）
    collision_object.id = object_id_;

    // ==================== 设置物体形状 ====================
    // 使用 mesh 形状（从 STL 文件加载）
    // meshes 和 mesh_poses 必须一一对应（索引匹配）

    // 添加 mesh 几何形状（顶点、三角面片等数据）
    collision_object.meshes.push_back(mesh_msg_);

    // 添加 mesh 的位姿（相对于 header.frame_id 的变换）
    // 这里使用转换后的 pose，已经在 planning_frame_ 坐标系中
    collision_object.mesh_poses.push_back(pose.pose);

    // ==================== 设置操作类型 ====================
    // ADD: 添加物体到场景，如果已存在则更新
    // 其他选项：
    // - REMOVE: 从场景中删除物体
    // - APPEND: 追加到现有物体（合并几何形状）
    // - MOVE: 移动物体（仅更新位姿）
    collision_object.operation = collision_object.ADD;

    // ==================== 应用到 MoveIt 场景 ====================
    // 调用 PlanningSceneInterface 的方法将物体添加到场景
    //
    // 内部工作流程：
    // 1. 发布消息到 /planning_scene 话题
    // 2. MoveIt 的 PlanningSceneMonitor 订阅该话题
    // 3. 场景更新会自动同步到 RViz 和 MoveGroup 节点
    // 4. 更新后的场景用于碰撞检测和运动规划
    planning_scene_interface_->applyCollisionObject(collision_object);
  }

  // ==================== 成员变量 ====================

  // ROS 订阅器：用于接收 /object_pose 话题
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;

  // MoveIt 场景接口：用于添加、更新、删除场景中的碰撞物体
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;

  // TF2 Buffer：存储最近的坐标变换信息（默认缓存约 10 秒）
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  // TF2 Listener：订阅 /tf 和 /tf_static 话题，并填充 tf_buffer_
  // 注意：listener 必须持续存在，否则无法接收 TF 更新
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ==================== 配置参数 ====================

  // mesh 文件的绝对路径（STL 格式）
  // 例如："/home/user/models/object.stl"
  std::string mesh_file_;

  // 物体在 MoveIt 场景中的唯一 ID
  // 用于更新或删除特定物体
  std::string object_id_;

  // MoveIt 规划坐标系的名称
  // 必须与 MoveIt 配置一致（如 base_footprint 或 base_link）
  std::string planning_frame_;

  // mesh 缩放比例
  // 用于单位转换或调整模型大小（如 0.001 将毫米转为米）
  double mesh_scale_;

  // ==================== Mesh 数据 ====================

  // 从 STL 文件加载并转换后的 mesh 消息
  // 包含顶点、三角面片、法向量等几何数据
  shape_msgs::msg::Mesh mesh_msg_;
};

/**
 * @brief 主函数 - 程序入口点
 *
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return int 退出状态码（0 表示正常退出）
 *
 * 执行流程：
 * 1. 初始化 ROS 2 系统（rclcpp::init）
 * 2. 创建 ObjectVisualizer 节点实例
 * 3. 进入事件循环（rclcpp::spin）- 阻塞执行，处理回调
 * 4. 退出时清理资源（rclcpp::shutdown）
 *
 * 运行方式：
 * - 直接运行：ros2 run hello_moveit object_visualizer
 * - 通过 launch 文件：ros2 launch hello_moveit object_visualizer.launch.py
 * - 带参数：ros2 run hello_moveit object_visualizer --ros-args -p mesh_file:=/path/to/model.stl
 *
 * 退出条件：
 * - Ctrl+C 信号（SIGINT）
 * - 节点被外部关闭（如 launch 文件停止）
 * - 程序内部调用 rclcpp::shutdown()
 */
int main(int argc, char **argv)
{
  // ==================== 初始化 ROS 2 ====================
  // 必须在创建任何 ROS 2 对象之前调用
  // 解析命令行参数，设置信号处理器等
  rclcpp::init(argc, argv);

  // ==================== 创建节点 ====================
  // 使用智能指针管理节点生命周期
  // 构造函数会完成所有初始化工作（参数、订阅器、TF、mesh 加载等）
  auto node = std::make_shared<ObjectVisualizer>();

  // ==================== 事件循环 ====================
  // spin 会阻塞当前线程，持续处理以下事件：
  // - 话题回调（/object_pose）
  // - 定时器回调
  // - 服务请求/响应
  // - 参数变更
  //
  // 内部使用单线程执行器（SingleThreadedExecutor）
  // 回调函数按顺序串行执行，不会并发
  rclcpp::spin(node);

  // ==================== 清理资源 ====================
  // spin 返回后（通常是收到 Ctrl+C），清理 ROS 2 资源
  // 包括：关闭节点、停止订阅器、释放 DDS 资源等
  rclcpp::shutdown();

  return 0; // 正常退出
}
