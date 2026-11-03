"""Run the existing four-step MoveIt cycle with the physical mirror."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit = MoveItConfigsBuilder(
        "DOFBOT_Pro-V24", package_name="My_JXB_moveit"
    ).to_moveit_configs()
    demo_and_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("dofbot_servo_bridge"), "launch", "mirror_demo.launch.py"
        ])),
        launch_arguments={
            "serial_port": LaunchConfiguration("serial_port"),
            "test_mode": LaunchConfiguration("test_mode"),
            "enable_motion": LaunchConfiguration("enable_motion"),
            "calibration_confirmed": LaunchConfiguration("calibration_confirmed"),
        }.items(),
    )
    cycle = Node(
        package="dofbot_cycle_demo", executable="cycle_demo",
        name="dofbot_cycle_demo", output="screen",
        parameters=[
            moveit.robot_description, moveit.robot_description_semantic,
            moveit.robot_description_kinematics,
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
    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("test_mode", default_value="true"),
        DeclareLaunchArgument("enable_motion", default_value="false"),
        DeclareLaunchArgument("calibration_confirmed", default_value="false"),
        DeclareLaunchArgument("arm_target", default_value="init"),
        DeclareLaunchArgument("arm_home", default_value="up"),
        DeclareLaunchArgument("repeat_count", default_value="1"),
        DeclareLaunchArgument("arm_speed", default_value="0.1"),
        DeclareLaunchArgument("gripper_speed", default_value="0.1"),
        demo_and_bridge,
        TimerAction(period=30.0, actions=[cycle]),
    ])
