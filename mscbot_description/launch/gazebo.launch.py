from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
import os
from os import pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # Directories and file paths
    mscbot_description = get_package_share_directory("mscbot_description")
    mscbot_description_prefix = get_package_prefix("mscbot_description")

    model_path = os.path.join(mscbot_description, "models")
    model_path += pathsep + os.path.join(mscbot_description_prefix, "share")

    # Environment variable for Gazebo model path
    env_variable = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)

    # Declare launch arguments
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(mscbot_description, "urdf", "mscbot.urdf.xacro"),
        description="Absolute path to robot URDF file"
    )

    world_arg = DeclareLaunchArgument(
        name="world",
        default_value=os.path.join(mscbot_description, "worlds", "test_world.world"),
        description="Absolute path to Gazebo world file"
    )

    # Robot description
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)

    # Nodes and Gazebo launch
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py")
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py")
        )
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "mscbot", "-topic", "robot_description"],
        output="screen"
    )

    return LaunchDescription([
        env_variable,
        model_arg,
        world_arg,
        robot_state_publisher,
        start_gazebo_server,
        start_gazebo_client,
        spawn_robot
    ])
