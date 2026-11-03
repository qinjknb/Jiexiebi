"""组合启动：MoveIt 仿真 + 双关节往返循环 + 舵机联动桥。

仿真里 Arm1/Arm2 每走一步，联动桥就把关节角换算成舵机角度，
经串口发给 STM32，驱动真实舵机同步动作（仿真 → 实物同步）。

用法示例：
  ros2 launch dofbot_servo_bridge bridge_cycle.launch.py \
      repeat_count:=1 serial_port:=/dev/ttyUSB0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bridge_share = FindPackageShare("dofbot_servo_bridge")

    # ---- 透传给 two_joint_cycle.launch.py 的仿真循环参数 ----
    goal_name = LaunchConfiguration("goal_name")
    repeat_count = LaunchConfiguration("repeat_count")
    speed = LaunchConfiguration("speed")
    pause_seconds = LaunchConfiguration("pause_seconds")
    endpoint_tolerance = LaunchConfiguration("endpoint_tolerance")

    # ---- MoveIt 配置包名/机器人名（适配不同工作空间布局）----
    robot_name = LaunchConfiguration("robot_name")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    demo_package = LaunchConfiguration("demo_package")
    demo_launch_file = LaunchConfiguration("demo_launch_file")

    # ---- 联动桥参数 ----
    serial_port = LaunchConfiguration("serial_port")
    baudrate = LaunchConfiguration("baudrate")
    test_mode = LaunchConfiguration("test_mode")

    cycle = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("dofbot_two_joint_cycle"),
                "launch", "two_joint_cycle.launch.py",
            ])
        ),
        launch_arguments={
            "goal_name": goal_name,
            "repeat_count": repeat_count,
            "speed": speed,
            "pause_seconds": pause_seconds,
            "endpoint_tolerance": endpoint_tolerance,
            "robot_name": robot_name,
            "moveit_config_package": moveit_config_package,
            "demo_package": demo_package,
            "demo_launch_file": demo_launch_file,
        }.items(),
    )

    bridge = Node(
        package="dofbot_servo_bridge",
        executable="servo_bridge",
        name="dofbot_servo_bridge",
        output="screen",
        parameters=[
            PathJoinSubstitution([bridge_share, "config", "bridge.yaml"]),
            {
                "serial_port": serial_port,
                "baudrate": ParameterValue(baudrate, value_type=int),
                "test_mode": ParameterValue(test_mode, value_type=bool),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("goal_name", default_value="test"),
        DeclareLaunchArgument("repeat_count", default_value="3"),
        DeclareLaunchArgument("speed", default_value="0.1"),
        DeclareLaunchArgument("pause_seconds", default_value="0.5"),
        DeclareLaunchArgument("endpoint_tolerance", default_value="0.01"),
        DeclareLaunchArgument(
            "robot_name", default_value="DOFBOT_Pro-V24",
            description="MoveIt 配置里的机器人名（SRDF/URDF 的 name）"),
        DeclareLaunchArgument(
            "moveit_config_package", default_value="dofbot_pro_moveit",
            description="MoveIt 配置包名（内含 config/ 与 SRDF 的包）"),
        DeclareLaunchArgument(
            "demo_package", default_value="dofbot_pro_moveit",
            description="demo 启动文件所在的包"),
        DeclareLaunchArgument(
            "demo_launch_file", default_value="demo.launch.py",
            description="demo 启动文件名（launch 子目录下）"),
        DeclareLaunchArgument(
            "serial_port", default_value="/dev/ttyUSB0",
            description="接 STM32 USART2 的 USB-TTL 串口设备"),
        DeclareLaunchArgument("baudrate", default_value="115200"),
        DeclareLaunchArgument(
            "test_mode", default_value="false",
            description="true 时用虚拟串口，只打印命令不发送"),
        cycle,
        bridge,
    ])
