from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    simple_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["simple_velocity_controller",
                "--controller-manager",
                "/controller_manager"
        ]
    )

    kinematics_controller_py = Node(
        package="mscbot_controller",
        executable="kinematics_controller.py",
  
    )

    return LaunchDescription(
        [
            joint_state_broadcaster_spawner,
            simple_controller,
            kinematics_controller_py
        ]
    )