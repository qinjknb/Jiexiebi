"""单独启动联动桥（不启动仿真与循环程序），供手动调试使用。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("dofbot_servo_bridge")

    serial_port = LaunchConfiguration("serial_port")
    baudrate = LaunchConfiguration("baudrate")
    test_mode = LaunchConfiguration("test_mode")

    bridge = Node(
        package="dofbot_servo_bridge",
        executable="servo_bridge",
        name="dofbot_servo_bridge",
        output="screen",
        parameters=[
            PathJoinSubstitution([pkg_share, "config", "bridge.yaml"]),
            {
                "serial_port": serial_port,
                "baudrate": ParameterValue(baudrate, value_type=int),
                "test_mode": ParameterValue(test_mode, value_type=bool),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_port", default_value="/dev/ttyUSB0",
            description="接 STM32 USART2 的 USB-TTL 串口设备"),
        DeclareLaunchArgument("baudrate", default_value="115200"),
        DeclareLaunchArgument(
            "test_mode", default_value="false",
            description="true 时用虚拟串口，只打印命令不发送"),
        bridge,
    ])
