from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory("robot_arm_controller"))
        / "config"
        / "robot_arm.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Absolute path to the robot arm parameter YAML file",
            ),
            Node(
                package="robot_arm_controller",
                executable="dynamixel_arm_node",
                name="dynamixel_arm_controller",
                output="screen",
                parameters=[LaunchConfiguration("config_file")],
            ),
        ]
    )
