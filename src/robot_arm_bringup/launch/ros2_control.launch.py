from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from robot_arm_bringup.calibration import (
    load_calibration,
    relative_limits_deg,
    xacro_arguments,
)


def generate_launch_description():
    share = Path(get_package_share_directory("robot_arm_bringup"))
    xacro_file = share / "urdf" / "robot_arm.urdf.xacro"
    calibration = load_calibration(share / "config" / "robot_arm_calibration.yaml")
    xacro_calibration_arguments = xacro_arguments(calibration)
    arm_names, arm_mins, arm_maxs, _ = relative_limits_deg(calibration, False)
    all_names, all_mins, all_maxs, _ = relative_limits_deg(calibration, True)
    device_name = LaunchConfiguration("device_name")
    use_gripper = LaunchConfiguration("use_gripper")
    controllers_file = LaunchConfiguration("controllers_file")
    namespace = LaunchConfiguration("namespace")
    trajectory_output_topic = LaunchConfiguration("trajectory_output_topic")
    start_controllers_inactive = LaunchConfiguration("start_controllers_inactive")
    controller_manager_name = PathJoinSubstitution(["/", namespace, "controller_manager"])
    joint_states_topic = PathJoinSubstitution(["/", namespace, "joint_states"])
    dynamic_joint_states_topic = PathJoinSubstitution(
        ["/", namespace, "dynamic_joint_states"]
    )

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                str(xacro_file),
                " device_name:=",
                device_name,
                " use_gripper:=",
                use_gripper,
                *xacro_calibration_arguments,
            ]
        ),
        value_type=str,
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=namespace,
        parameters=[controllers_file],
        remappings=[
            ("~/robot_description", "robot_description"),
            ("/joint_states", joint_states_topic),
            ("/dynamic_joint_states", dynamic_joint_states_topic),
        ],
        output="screen",
    )
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )
    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["joint_state_broadcaster", "--controller-manager", controller_manager_name],
        condition=UnlessCondition(start_controllers_inactive),
        output="screen",
    )
    joint_state_spawner_inactive = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            controller_manager_name,
            "--inactive",
        ],
        condition=IfCondition(start_controllers_inactive),
        output="screen",
    )
    trajectory_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["arm_trajectory_controller", "--controller-manager", controller_manager_name],
        condition=UnlessCondition(start_controllers_inactive),
        output="screen",
    )
    trajectory_spawner_inactive = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=[
            "arm_trajectory_controller",
            "--controller-manager",
            controller_manager_name,
            "--inactive",
        ],
        condition=IfCondition(start_controllers_inactive),
        output="screen",
    )
    degree_bridge_arm_only = Node(
        package="robot_arm_controller",
        executable="degree_trajectory_bridge",
        namespace=namespace,
        parameters=[
            {
                "joint_names": arm_names,
                "min_position_deg": arm_mins,
                "max_position_deg": arm_maxs,
                "output_topic": trajectory_output_topic,
            }
        ],
        condition=UnlessCondition(use_gripper),
        output="screen",
    )
    degree_bridge_with_gripper = Node(
        package="robot_arm_controller",
        executable="degree_trajectory_bridge",
        namespace=namespace,
        parameters=[
            {
                "joint_names": all_names,
                "min_position_deg": all_mins,
                "max_position_deg": all_maxs,
                "output_topic": trajectory_output_topic,
            }
        ],
        condition=IfCondition(use_gripper),
        output="screen",
    )
    diagnostics_arm_only = Node(
        package="robot_arm_hardware",
        executable="dynamixel_diagnostics_node",
        namespace=namespace,
        parameters=[{"joint_names": arm_names}],
        condition=UnlessCondition(use_gripper),
        output="screen",
    )
    diagnostics_with_gripper = Node(
        package="robot_arm_hardware",
        executable="dynamixel_diagnostics_node",
        namespace=namespace,
        parameters=[{"joint_names": all_names}],
        condition=IfCondition(use_gripper),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("device_name", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("use_gripper", default_value="true"),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("start_controllers_inactive", default_value="false"),
            DeclareLaunchArgument(
                "trajectory_output_topic",
                default_value="arm_trajectory_controller/joint_trajectory",
            ),
            DeclareLaunchArgument(
                "controllers_file",
                default_value=str(share / "config" / "controllers.yaml"),
            ),
            control_node,
            state_publisher,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=control_node,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(
                                reason="controller_manager exited; stopping bringup"
                            )
                        )
                    ],
                )
            ),
            joint_state_spawner,
            joint_state_spawner_inactive,
            degree_bridge_arm_only,
            degree_bridge_with_gripper,
            diagnostics_arm_only,
            diagnostics_with_gripper,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_spawner,
                    on_exit=[trajectory_spawner],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_spawner_inactive,
                    on_exit=[trajectory_spawner_inactive],
                )
            ),
        ]
    )
