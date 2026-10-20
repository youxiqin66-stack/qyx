from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    controllers_yaml = get_package_share_directory("arm_config") + "/config/ros2_controllers.yaml"

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c", "/controller_manager",
            "--param-file", controllers_yaml,
        ],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "-c", "/controller_manager",
            "--param-file", controllers_yaml,
        ],
        output="screen",
    )

    arm_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_velocity_controller",
            "--inactive",
            "-c", "/controller_manager",
            "--param-file", controllers_yaml,
        ],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[arm_controller_spawner],
            )
        )
    )
    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=arm_controller_spawner,
                on_exit=[arm_velocity_controller_spawner],
            )
        )
    )
    return ld
