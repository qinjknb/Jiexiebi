"""Start the existing MoveIt demo and the physical servo mirror together."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def include(package, filename, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare(package), "launch", filename
        ])),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("test_mode", default_value="true"),
        DeclareLaunchArgument("enable_motion", default_value="false"),
        DeclareLaunchArgument("calibration_confirmed", default_value="false"),
        include("My_JXB_moveit", "demo.launch.py"),
        include("dofbot_servo_bridge", "servo_bridge.launch.py", {
            "serial_port": LaunchConfiguration("serial_port"),
            "test_mode": LaunchConfiguration("test_mode"),
            "enable_motion": LaunchConfiguration("enable_motion"),
            "calibration_confirmed": LaunchConfiguration("calibration_confirmed"),
        }),
    ])
