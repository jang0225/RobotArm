from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("robot_arm_bringup"))
    xacro_file = share / "urdf" / "robot_arm.urdf.xacro"
    device_name = LaunchConfiguration("device_name")
    use_gripper = LaunchConfiguration("use_gripper")
    controllers_file = LaunchConfiguration("controllers_file")
    namespace = LaunchConfiguration("namespace")
    trajectory_output_topic = LaunchConfiguration("trajectory_output_topic")
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
            ]
        ),
        value_type=str,
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=namespace,
        parameters=[{"robot_description": robot_description}, controllers_file],
        remappings=[
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
        output="screen",
    )
    trajectory_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["arm_trajectory_controller", "--controller-manager", controller_manager_name],
        output="screen",
    )
    degree_bridge = Node(
        package="robot_arm_controller",
        executable="degree_trajectory_bridge",
        namespace=namespace,
        parameters=[
            {
                "include_gripper": ParameterValue(
                    use_gripper,
                    value_type=bool,
                ),
                "output_topic": trajectory_output_topic,
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("device_name", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("use_gripper", default_value="true"),
            DeclareLaunchArgument("namespace", default_value=""),
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
            degree_bridge,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_spawner,
                    on_exit=[trajectory_spawner],
                )
            ),
        ]
    )
