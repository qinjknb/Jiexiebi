"""Run the bridge beside an already running MoveIt simulation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("test_mode", default_value="true"),
        DeclareLaunchArgument("enable_motion", default_value="false"),
        DeclareLaunchArgument("calibration_confirmed", default_value="false"),
        Node(
            package="dofbot_servo_bridge",
            executable="servo_bridge",
            name="dofbot_servo_bridge",
            output="screen",
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare("dofbot_servo_bridge"), "config", "bridge.yaml"
                ]),
                {
                    "serial_port": LaunchConfiguration("serial_port"),
                    "test_mode": ParameterValue(LaunchConfiguration("test_mode"), value_type=bool),
                    "enable_motion": ParameterValue(
                        LaunchConfiguration("enable_motion"), value_type=bool
                    ),
                    "calibration_confirmed": ParameterValue(
                        LaunchConfiguration("calibration_confirmed"), value_type=bool
                    ),
                },
            ],
        ),
    ])
