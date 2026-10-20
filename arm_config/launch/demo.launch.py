from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import numpy as np
from scipy.spatial.transform import Rotation
import os


def generate_launch_description():
    # ==================== 串口参数 ====================
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
                # 暂时使用 Fake 控制器进行仿真
                #  "use_fake_hardware": "true",
            }
        )
        .to_moveit_configs()
    )

    # 载入 Servo 参数，必须封装在{"moveit_servo": xxx} 下
    import yaml
    servo_yaml = os.path.join(FindPackageShare("arm_config").find("arm_config"), "config", "servo.yaml")
    with open(servo_yaml, 'r') as file:
        raw_servo_params = yaml.safe_load(file)
    servo_params = {"moveit_servo": raw_servo_params}
    joint_limits_yaml = os.path.join(FindPackageShare("arm_config").find("arm_config"), "config", "joint_limits.yaml")
    with open(joint_limits_yaml, 'r') as file:
        robot_description_planning = {"robot_description_planning": yaml.safe_load(file)}
    
    # 根据源码工作空间的示例配置，我们应该使用 standalone 的 servo_node_main，而非 component
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            robot_description_planning,
            # Intentionally do not load kinematics plugin params into servo_node.
            # This makes MoveIt Servo use inverse-Jacobian mode, which is more stable
            # for high-rate Cartesian servoing in this setup.
        ],
        output="screen",
    )

    # ==================== 手眼标定结果 ====================
    # 注意：请确认您的标定结果是以下哪种：
    
    # 1. T_base_camera (base_link → camera_frame)：表示相机在基座坐标系中的位姿
    # 2. T_camera_base (camera_frame → base_link)：表示基座在相机坐标系中的位姿
    
    # 以下是您的标定结果（请确认是哪个方向）
    # 旋转矩阵
    rotation_matrix = np.array([
        [-0.9987, 0.0500, 0.0021],
        [-0.0011, 0.0191, -0.9998],
        [-0.0501, -0.9986, -0.0190]
    ])
    
    # 平移向量 (单位：米)
    translation = np.array([0.0371, 0.5454, 0.2647])
    
    # ==================== 变换求逆（如果需要） ====================
    # 如果标定给出的是 T_camera_base，但 TF 需要 T_base_camera，则需要求逆
    # 取消下面两行的注释来使用逆变换：
    
    # rotation_matrix = rotation_matrix.T  # 旋转矩阵的逆 = 转置
    # translation = -rotation_matrix @ translation  # 平移的逆变换
    
    # 将旋转矩阵转换为四元数
    r = Rotation.from_matrix(rotation_matrix)
    quaternion = r.as_quat()  # [x, y, z, w]
    
    # 静态TF发布器：从 base_link 到 camera_frame
    # 这个 TF 表示：相机在机器人基座坐标系中的位置和姿态
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_base_broadcaster',
        arguments=[
            str(translation[0]), str(translation[1]), str(translation[2]),  # x y z
            str(quaternion[0]), str(quaternion[1]), str(quaternion[2]), str(quaternion[3]),  # qx qy qz qw
            'base_link',  # parent frame：父坐标系
            'camera_frame'  # child frame：子坐标系（相机位于 base_link 的这个位姿）
        ],
        output='screen'
    )
    
    # Object Visualizer 节点 - 订阅 /object_pose 并在 RViz 中显示
    # 声明mesh文件路径参数
    mesh_file_arg = DeclareLaunchArgument(
        'mesh_file',
        default_value='/home/qyx/LLM/code/ws_moveit/src/hello_moveit/red_triangle.STL',
        description='Path to the STL mesh file'
    )
    
    object_visualizer_node = Node(
        package='hello_moveit',
        executable='object_visualizer',
        name='object_visualizer',
        output='screen',
        parameters=[{
            'mesh_file': LaunchConfiguration('mesh_file'),
            'object_id': 'tracked_object',
            'planning_frame': 'base_link',
            'mesh_scale': 0.001,  # 将毫米单位转换为米单位 (mm -> m)
        }]
    )

    # FoundationPose ROS2 节点：ZeroMQ 订阅 -> /object_pose + /object_intent + TF
    foundationpose_node = Node(
        package='foundationpose_ros2',
        executable='foundationpose_node',
        name='foundationpose_node',
        output='screen',
    )

    # 统一动作调度节点：按 intent 执行 approach/grasp/grind
    action_dispatcher_node = Node(
        package='hello_moveit',
        executable='action_dispatcher',
        name='action_dispatcher',
        output='screen',
    )

    # 获取 demo launch 的内容
    demo_launch = generate_demo_launch(moveit_config)
    
    # 合并所有节点
    ld = LaunchDescription()
    ld.add_action(comm_port_arg)
    ld.add_action(comm_baudrate_arg)
    ld.add_action(mesh_file_arg)
    ld.add_action(static_tf_node)
    ld.add_action(object_visualizer_node)
    ld.add_action(foundationpose_node)
    ld.add_action(action_dispatcher_node)

    # 添加 demo launch 中的所有节点
    for entity in demo_launch.entities:
        ld.add_action(entity)

    ld.add_action(servo_node)
    
    return ld
