from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # 声明参数
    mesh_file_arg = DeclareLaunchArgument(
        'mesh_file',
        default_value='/home/ruibin/ws_moveit/src/hello_moveit/red_triangle.STL',
        description='Path to the mesh file (STL) to visualize'
    )
    
    object_id_arg = DeclareLaunchArgument(
        'object_id',
        default_value='tracked_object',
        description='ID of the object in the planning scene'
    )
    
    planning_frame_arg = DeclareLaunchArgument(
        'planning_frame',
        default_value='base_link',
        description='Planning frame for MoveIt'
    )
    
    mesh_scale_arg = DeclareLaunchArgument(
        'mesh_scale',
        default_value='1.0',
        description='Scale factor for the mesh'
    )
    
    # Object Visualizer 节点
    object_visualizer_node = Node(
        package='hello_moveit',
        executable='object_visualizer',
        name='object_visualizer',
        output='screen',
        parameters=[{
            'mesh_file': LaunchConfiguration('mesh_file'),
            'object_id': LaunchConfiguration('object_id'),
            'planning_frame': LaunchConfiguration('planning_frame'),法
            'mesh_scale': LaunchConfiguration('mesh_scale'),
        }]
    )
    
    return LaunchDescription([
        mesh_file_arg,
        object_id_arg,
        planning_frame_arg,
        mesh_scale_arg,
        object_visualizer_node
    ])
