from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arm_share = Path(get_package_share_directory("robot_arm_bringup"))
    fss_share = Path(get_package_share_directory("fss_bringup"))

    device_name = LaunchConfiguration("device_name")
    start_fss = LaunchConfiguration("start_fss")
    arm_only = LaunchConfiguration("arm_only")
    use_sim = LaunchConfiguration("use_sim")
    cpu_pin = LaunchConfiguration("cpu_pin")
    log_level = LaunchConfiguration("log_level")
    use_realsense = LaunchConfiguration("use_realsense")
    require_healthy = LaunchConfiguration("require_healthy")

    fss_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(fss_share / "launch" / "fss_full.launch.py")),
        launch_arguments={
            "use_sim": use_sim,
            "cpu_pin": cpu_pin,
            "log_level": log_level,
            "use_realsense": use_realsense,
        }.items(),
        condition=IfCondition(start_fss),
    )

    full_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(arm_share / "launch" / "ros2_control.launch.py")),
        launch_arguments={
            "device_name": device_name,
            "use_gripper": "true",
            "namespace": "robot_arm",
            "trajectory_output_topic": "requested_joint_trajectory",
            "controllers_file": str(arm_share / "config" / "controllers_fss.yaml"),
        }.items(),
        condition=UnlessCondition(arm_only),
    )

    arm_only_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(arm_share / "launch" / "ros2_control.launch.py")),
        launch_arguments={
            "device_name": device_name,
            "use_gripper": "false",
            "namespace": "robot_arm",
            "trajectory_output_topic": "requested_joint_trajectory",
            "controllers_file": str(
                arm_share / "config" / "controllers_fss_arm_only.yaml"
            ),
        }.items(),
        condition=IfCondition(arm_only),
    )

    supervisor = Node(
        package="robot_arm_controller",
        executable="fss_arm_supervisor_node",
        namespace="robot_arm",
        parameters=[
            {
                "requested_trajectory_topic": "requested_joint_trajectory",
                "controller_trajectory_topic": (
                    "arm_trajectory_controller/joint_trajectory"
                ),
                "joint_states_topic": "joint_states",
                "mode_topic": "/system/mode",
                "health_topic": "/system/health",
                "require_healthy": ParameterValue(require_healthy, value_type=bool),
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "device_name",
                default_value="/dev/ttyUSB0",
                description="Dynamixel U2D2 device; prefer /dev/serial/by-id/...",
            ),
            DeclareLaunchArgument(
                "start_fss",
                default_value="false",
                description=(
                    "Start fss_full.launch.py too; keep false when FSS is already running"
                ),
            ),
            DeclareLaunchArgument(
                "arm_only",
                default_value="true",
                description="Start only joints 1-3 and leave the unfinished gripper disabled",
            ),
            DeclareLaunchArgument("use_sim", default_value="false"),
            DeclareLaunchArgument("cpu_pin", default_value="false"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("use_realsense", default_value="false"),
            DeclareLaunchArgument(
                "require_healthy",
                default_value="true",
                description="Require healthy FSS navigation and actuators in ACTIVE mode",
            ),
            fss_launch,
            full_arm_launch,
            arm_only_launch,
            supervisor,
        ]
    )
