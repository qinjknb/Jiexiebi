#!/usr/bin/env python3
"""实机上机 launch：MoveIt(move_group) + RViz + 真实机械臂桥接节点。

和 ``ros2 launch My_JXB_moveit demo.launch.py``（纯仿真）的区别只有一个：
**不启动 ros2_control 的 mock 硬件**，而是启动 ``trajectory_bridge`` 节点，
由它把 MoveIt 规划出来的同一条轨迹通过串口发给 STM32。

用法：

    # 1) 先不接机械臂，把整条链路跑通（RViz 里能看到机械臂跟着动）
    ros2 launch dofbot_hw_bridge arm_real.launch.py hardware:=false

    # 2) 接上机械臂正式运行
    ros2 launch dofbot_hw_bridge arm_real.launch.py port:=/dev/ttyUSB0

    # 3) 用自定义标定文件 + 自动回中位
    ros2 launch dofbot_hw_bridge arm_real.launch.py \\
        calib_file:=$HOME/My_JXB/src/dofbot_hw_bridge/config/arm_calib.yaml auto_home:=true

然后和仿真完全一样地用：RViz 里拖 Goal → Plan & Execute，
或者 ``ros2 launch dofbot_cycle_demo cycle_demo.launch.py``（不要再叠加 demo.launch.py）。
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_setup(context, *args, **kwargs):
    from moveit_configs_utils import MoveItConfigsBuilder

    moveit_config = MoveItConfigsBuilder(
        "DOFBOT_Pro-V24", package_name="My_JXB_moveit").to_moveit_configs()

    # ---------------- robot_state_publisher ----------------
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description, {"use_sim_time": False}],
    )

    # ---------------- world -> base_link 固定变换 ----------------
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="log",
        arguments=["--x", "0", "--y", "0", "--z", "0",
                   "--roll", "0", "--pitch", "0", "--yaw", "0",
                   "--frame-id", "world", "--child-frame-id", "base_link"],
    )

    # ---------------- 真实机械臂桥 ----------------
    bridge = Node(
        package="dofbot_hw_bridge",
        executable="trajectory_bridge",
        name="dofbot_hw_bridge",
        output="screen",
        parameters=[{
            "calib_file": LaunchConfiguration("calib_file"),
            "port": LaunchConfiguration("port"),
            "baud": ParameterValue(LaunchConfiguration("baud"), value_type=int),
            "hardware": LaunchConfiguration("hardware"),
            "auto_home": LaunchConfiguration("auto_home"),
            "home_time": ParameterValue(LaunchConfiguration("home_time"), value_type=float),
            "interpolation": LaunchConfiguration("interpolation"),
            "command_rate": ParameterValue(LaunchConfiguration("command_rate"),
                                           value_type=float),
            "max_velocity": ParameterValue(LaunchConfiguration("max_velocity"),
                                           value_type=float),
        }],
    )

    # ---------------- move_group ----------------
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": False,
                # 双保险：允许起始状态比 URDF 限位多出 0.1rad（MoveIt 会自己钳位并告警），
                # 避免舵机回读的一点点噪声就让规划被拒绝。
                "start_state_max_bounds_error": 0.1,
            },
        ],
    )

    # ---------------- RViz ----------------
    rviz_config = os.path.join(str(moveit_config.package_path), "config", "moveit.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": False},
        ],
    )

    actions = [rsp, static_tf, bridge, move_group, rviz]

    # ---------------- 可选：自动跑 dofbot_cycle_demo 的取放循环 ----------------
    # 注意：不要再去 launch dofbot_cycle_demo/cycle_demo.launch.py，
    # 那个文件内部又 include 了 demo.launch.py（会额外拉起 mock 控制器）。
    cycle = Node(
        package="dofbot_cycle_demo",
        executable="cycle_demo",
        name="dofbot_cycle_demo",
        output="screen",
        condition=IfCondition(LaunchConfiguration("with_cycle")),
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {
                "arm_target": LaunchConfiguration("arm_target"),
                "arm_home": LaunchConfiguration("arm_home"),
                "repeat_count": ParameterValue(LaunchConfiguration("repeat_count"),
                                               value_type=int),
                "arm_speed": ParameterValue(LaunchConfiguration("arm_speed"),
                                            value_type=float),
                "gripper_speed": ParameterValue(LaunchConfiguration("gripper_speed"),
                                                value_type=float),
            },
        ],
    )
    actions.append(TimerAction(period=10.0, actions=[cycle]))
    return actions


def generate_launch_description():
    here = os.path.dirname(os.path.abspath(__file__))
    default_calib = os.path.normpath(os.path.join(here, "..", "config", "arm_calib.yaml"))

    args = [
        DeclareLaunchArgument(
            "hardware", default_value="true",
            description="true=驱动真实机械臂；false=只发关节状态（先验证整条链路）"),
        DeclareLaunchArgument(
            "port", default_value="/dev/ttyUSB0",
            description="USB-TTL 串口设备名（Windows: COMx；WSL 需先 usbipd 转发）"),
        DeclareLaunchArgument(
            "baud", default_value="115200", description="与固件 ARM_LINK_BAUD 一致"),
        DeclareLaunchArgument(
            "calib_file", default_value=default_calib, description="关节标定 YAML"),
        DeclareLaunchArgument(
            "auto_home", default_value="false", description="启动后先回中位"),
        DeclareLaunchArgument(
            "home_time", default_value="2.0", description="回中位用时（秒）"),
        DeclareLaunchArgument(
            "interpolation", default_value="auto",
            description="轨迹插值：auto|cubic|linear"),
        DeclareLaunchArgument(
            "command_rate", default_value="50.0",
            description="向 STM32 下发目标的频率 Hz（调小可降低舵机压力，如 25）"),
        DeclareLaunchArgument(
            "max_velocity", default_value="3000.0",
            description="PC 侧安全限速 ticks/s（300° 舵机 13.65 tick/度，"
                        "1000 ≈ 73°/s；0=不限制）"),
        DeclareLaunchArgument(
            "with_cycle", default_value="false",
            description="true=启动 10 秒后自动跑 dofbot_cycle_demo 的取放循环"),
        DeclareLaunchArgument("arm_target", default_value="init"),
        DeclareLaunchArgument("arm_home", default_value="up"),
        DeclareLaunchArgument("repeat_count", default_value="1"),
        DeclareLaunchArgument("arm_speed", default_value="0.1"),
        DeclareLaunchArgument("gripper_speed", default_value="0.1"),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_launch_setup)])
