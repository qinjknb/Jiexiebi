from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "DOFBOT_Pro-V24", package_name="My_JXB_moveit"
    ).to_moveit_configs()

    demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("My_JXB_moveit"), "launch", "demo.launch.py"]
            )
        )
    )

    cycle = Node(
        package="dofbot_cycle_demo",
        executable="cycle_demo",
        name="dofbot_cycle_demo",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {
                "arm_target": LaunchConfiguration("arm_target"),
                "arm_home": LaunchConfiguration("arm_home"),
                "repeat_count": ParameterValue(
                    LaunchConfiguration("repeat_count"), value_type=int
                ),
                "arm_speed": ParameterValue(
                    LaunchConfiguration("arm_speed"), value_type=float
                ),
                "gripper_speed": ParameterValue(
                    LaunchConfiguration("gripper_speed"), value_type=float
                ),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_target", default_value="init"),
            DeclareLaunchArgument("arm_home", default_value="up"),
            DeclareLaunchArgument("repeat_count", default_value="3"),
            DeclareLaunchArgument("arm_speed", default_value="0.1"),
            DeclareLaunchArgument("gripper_speed", default_value="0.1"),
            demo,
            TimerAction(period=30.0, actions=[cycle]),
        ]
    )
