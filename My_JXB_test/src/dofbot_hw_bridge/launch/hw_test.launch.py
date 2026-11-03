#!/usr/bin/env python3
"""只启动"模型 + 桥接节点 + RViz"，不启动 MoveIt。

用于上机前的分步验证：
    1) 看 RViz 里的机械臂姿态是否与真机一致（说明标定的方向/零点对了）；
    2) 用 ros2 topic echo /joint_states 检查数据；
    3) 手动发一条轨迹测试动作服务器：

       ros2 action send_goal /arm_group_controller/follow_joint_trajectory \\
         control_msgs/action/FollowJointTrajectory \\
         "{trajectory: {joint_names: [Arm1_Joint, Arm2_Joint, Arm3_Joint, Arm4_Joint, Arm5_Joint],
            points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 2}},
                     {positions: [0.3, -0.3, 0.2, 0.0, 0.0], time_from_start: {sec: 5}}]}}"

用法：
    ros2 launch dofbot_hw_bridge hw_test.launch.py hardware:=false   # 纯虚拟
    ros2 launch dofbot_hw_bridge hw_test.launch.py                   # 接真机
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_setup(context, *args, **kwargs):
    from moveit_configs_utils import MoveItConfigsBuilder

    moveit_config = MoveItConfigsBuilder(
        "DOFBOT_Pro-V24", package_name="My_JXB_moveit").to_moveit_configs()

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description, {"use_sim_time": False}],
    )

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
            "interpolation": LaunchConfiguration("interpolation"),
        }],
    )

    rviz_config = os.path.join(str(moveit_config.package_path), "config", "moveit.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        condition=IfCondition(LaunchConfiguration("rviz")),
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            {"use_sim_time": False},
        ],
    )

    return [rsp, bridge, rviz]


def generate_launch_description():
    here = os.path.dirname(os.path.abspath(__file__))
    default_calib = os.path.normpath(os.path.join(here, "..", "config", "arm_calib.yaml"))
    return LaunchDescription([
        DeclareLaunchArgument("hardware", default_value="true"),
        DeclareLaunchArgument("port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baud", default_value="115200"),
        DeclareLaunchArgument("calib_file", default_value=default_calib),
        DeclareLaunchArgument("interpolation", default_value="auto"),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=_launch_setup),
    ])
