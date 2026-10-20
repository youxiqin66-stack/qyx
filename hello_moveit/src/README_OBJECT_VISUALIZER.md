# Object Visualizer - 实时物体可视化节点

## 概述
这个节点订阅 `/object_pose` 话题（由 FoundationPose 节点发布），并通过 MoveIt 的 `PlanningSceneInterface` 在 RViz 中实时显示跟踪的物体。

## 功能特点
- 订阅 `/object_pose` 话题获取物体位姿
- 支持加载 STL mesh 文件来精确显示物体形状
- 自动进行 TF 坐标变换（camera_frame → base_link）
- 通过 PlanningSceneInterface 将物体添加到 MoveIt 场景中
- 在 RViz 中实时更新物体位置和姿态

## 文件说明
- `src/object_visualizer.cpp` - 主节点源代码
- `launch/object_visualizer.launch.py` - 独立启动文件
- 已集成到 `arm_config/launch/demo.launch.py`

## 编译
```bash
cd ~/ws_moveit
colcon build --packages-select hello_moveit
source install/setup.bash
```

## 使用方法

### 方法1：通过 demo.launch.py 启动（推荐）
这个方法会同时启动 MoveIt、RViz 和 object_visualizer：

```bash
cd ~/ws_moveit
source install/setup.bash

# 默认使用 ~/red_triangle.stl
ros2 launch arm_config demo.launch.py

# 或指定自定义 mesh 文件
ros2 launch arm_config demo.launch.py mesh_file:=/path/to/your/mesh.stl
```

### 方法2：单独启动 object_visualizer
```bash
ros2 launch hello_moveit object_visualizer.launch.py \
    mesh_file:=/path/to/red_triangle.stl \
    object_id:=tracked_object \
    planning_frame:=base_link \
    mesh_scale:=1.0
```

## 参数说明
- `mesh_file`: STL mesh 文件的完整路径（默认：`~/red_triangle.stl`）
- `object_id`: 物体在规划场景中的ID（默认：`tracked_object`）
- `planning_frame`: MoveIt 的规划坐标系（默认：`base_link`）
- `mesh_scale`: mesh 缩放因子（默认：`1.0`）

## TF 变换链
```
base_link (MoveIt planning frame)
    ↓ [静态TF - 手眼标定]
camera_frame
    ↓ [动态TF - FoundationPose]
object (tracked_object)
```

## 完整系统启动流程

### 1. 启动 MoveIt 和 RViz（包含 object_visualizer）
```bash
# Terminal 1
cd ~/ws_moveit
source install/setup.bash
ros2 launch arm_config demo.launch.py mesh_file:=~/red_triangle.stl
```

### 2. 启动 FoundationPose 节点（发布 /object_pose）
```bash
# Terminal 2
cd ~/FoundationPose/FoundationPose/ros
source install/setup.bash
ros2 run foundationpose_ros2 foundationpose_node
```

### 3. 启动 FoundationPose 主程序（ZeroMQ 服务器）
```bash
# Terminal 3
cd ~/FoundationPose/FoundationPose
# 运行你的 FoundationPose 检测程序（发送位姿到 ZeroMQ）
```

## 在 RViz 中查看物体

1. 确保 RViz 中的 **PlanningScene** 显示已启用
2. 在左侧面板中找到 **Scene Robot** 部分
3. 勾选 **Show Scene Geometry** 
4. 物体将显示为绿色（碰撞物体）或灰色

如果看不到物体：
- 检查 `/object_pose` 话题是否有数据：`ros2 topic echo /object_pose`
- 检查 TF 树：`ros2 run tf2_tools view_frames`
- 查看节点日志：`ros2 node list` 和 `ros2 node info /object_visualizer`

## 调试命令

查看话题：
```bash
ros2 topic list
ros2 topic echo /object_pose
ros2 topic hz /object_pose
```

查看 TF 变换：
```bash
ros2 run tf2_ros tf2_echo base_link camera_frame
ros2 run tf2_ros tf2_echo camera_frame object
ros2 run tf2_tools view_frames
```

查看节点信息：
```bash
ros2 node list
ros2 node info /object_visualizer
```

## 故障排除

### 问题1：找不到 mesh 文件
**错误**: `Failed to load mesh from: ...`
**解决**: 
- 确保 STL 文件存在：`ls -lh ~/red_triangle.stl`
- 使用绝对路径
- 检查文件权限

### 问题2：TF 变换失败
**错误**: `Waiting for transform from camera_frame to base_link`
**解决**:
- 确保静态 TF 已发布：`ros2 topic echo /tf_static`
- 检查坐标系名称是否匹配
- 确认 FoundationPose 节点正在运行

### 问题3：RViz 中看不到物体
**解决**:
- 在 RViz 中启用 **PlanningScene** 显示
- 勾选 **Show Scene Geometry**
- 检查物体是否在视野范围内（可能太远或太小）
- 尝试在 RViz 中重置视角

### 问题4：物体不更新
**解决**:
- 检查 `/object_pose` 话题频率：`ros2 topic hz /object_pose`
- 查看节点日志：`ros2 node info /object_visualizer`
- 确认 FoundationPose 正在发布位姿

## 代码说明

### 主要功能
1. **loadMesh()**: 从 STL 文件加载 mesh 并转换为 ROS 消息格式
2. **poseCallback()**: 接收位姿，执行 TF 变换到 base_link
3. **updateObject()**: 更新 PlanningScene 中的物体位置

### 关键依赖
- `moveit_ros_planning_interface`: PlanningSceneInterface
- `geometric_shapes`: mesh 加载和操作
- `tf2_ros`: 坐标变换
- `geometry_msgs`: 位姿消息

## 自定义修改

### 修改物体颜色
在 RViz 的 PlanningScene 中，碰撞物体显示为特定颜色。如果想要自定义颜色，需要修改 `collision_object` 为 `visualization_msgs::Marker`。

### 修改更新频率
默认每帧都更新。如果需要降低频率，可以在 `poseCallback` 中添加节流逻辑。

### 使用多个物体
复制节点代码，修改 `object_id` 参数，订阅不同的话题即可。

## 许可证
与 MoveIt 和 ROS 2 相同的许可证。
