from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import numpy as np
from scipy.spatial.transform import Rotation


def generate_launch_description():
    """红瓶插孔任务的「demo.launch 类比」启动文件。

    参照 arm_config/launch/demo.launch.py，只负责基础设施 + 虚拟映射显示：
      - move_group / rviz / 控制器（generate_demo_launch）
      - 手眼静态 TF（base_link -> camera_frame）
      - bottle_visualizer（订阅 /object_pose，显示瓶网格 + 两个目标位姿轴）

    故意不包含：
      - foundationpose_node  —— 由你单独 `ros2 launch foundationpose_ros2 foundationpose.launch.py` 起
      - bottle_insert        —— 由你最后手动 `ros2 run hello_moveit bottle_insert` 起
      - servo_node / object_visualizer / action_dispatcher（三角块跟踪那套，这里不需要）
    """

    # ==================== 串口参数（真实机械臂） ====================
    comm_port_arg = DeclareLaunchArgument(
        'comm_port',
        default_value='/dev/ttyUSB0',
        description='串口设备路径，例如 /dev/ttyUSB0'
    )
    comm_baudrate_arg = DeclareLaunchArgument(
        'comm_baudrate',
        default_value='115200',
        description='串口波特率'
    )

    moveit_config = (
        MoveItConfigsBuilder("robot_description", package_name="arm_config")
        .robot_description(
            mappings={
                "comm_port":     LaunchConfiguration("comm_port"),
                "comm_baudrate": LaunchConfiguration("comm_baudrate"),
            }
        )
        .to_moveit_configs()
    )

    # ==================== 手眼标定结果（与 demo.launch.py 完全一致） ====================
    # 眼在手外，base_link -> camera_frame 静态 TF
    rotation_matrix = np.array([
        [-0.9987, 0.0500, 0.0021],
        [-0.0011, 0.0191, -0.9998],
        [-0.0501, -0.9986, -0.0190]
    ])
    translation = np.array([0.0371, 0.5454, 0.2647])

    r = Rotation.from_matrix(rotation_matrix)
    quaternion = r.as_quat()  # [x, y, z, w]

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_base_broadcaster',
        arguments=[
            str(translation[0]), str(translation[1]), str(translation[2]),
            str(quaternion[0]), str(quaternion[1]), str(quaternion[2]), str(quaternion[3]),
            'base_link',
            'camera_frame'
        ],
        output='screen'
    )

    # ==================== 红瓶虚拟映射可视化节点 ====================
    mesh_file_arg = DeclareLaunchArgument(
        'mesh_file',
        default_value='/home/qyx/LLM/code/ws_moveit/src/hello_moveit/bottle_red.STL',
        description='Path to the STL mesh file'
    )

    bottle_visualizer_node = Node(
        package='hello_moveit',
        executable='bottle_visualizer',
        name='bottle_visualizer',
        output='screen',
        parameters=[{
            'mesh_file': LaunchConfiguration('mesh_file'),
            'object_id': 'bottle',
            'planning_frame': 'base_link',
            'mesh_scale': 0.001,  # 将毫米单位转换为米单位 (mm -> m)
        }]
    )

    # 获取 demo launch 的内容（move_group / rviz / 控制器等）
    demo_launch = generate_demo_launch(moveit_config)

    # 合并所有节点
    ld = LaunchDescription()
    ld.add_action(comm_port_arg)
    ld.add_action(comm_baudrate_arg)
    ld.add_action(mesh_file_arg)
    ld.add_action(static_tf_node)
    ld.add_action(bottle_visualizer_node)

    # 添加 demo launch 中的所有节点
    for entity in demo_launch.entities:
        ld.add_action(entity)

    return ld
