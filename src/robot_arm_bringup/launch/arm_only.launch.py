from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory("robot_arm_bringup"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("device_name", default_value="/dev/ttyUSB0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(share / "launch" / "ros2_control.launch.py")),
                launch_arguments={
                    "device_name": LaunchConfiguration("device_name"),
                    "use_gripper": "false",
                    "controllers_file": str(share / "config" / "controllers_arm_only.yaml"),
                }.items(),
            ),
        ]
    )
