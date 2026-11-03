"""启动：MoveIt 仿真 demo + 双关节往返循环节点。

MoveIt 配置包名、机器人名、demo 启动文件都可通过 launch 参数覆盖，
适配不同的工作空间布局（默认按 dofbot_pro_moveit / DOFBOT_Pro-V24）：

  ros2 launch dofbot_two_joint_cycle two_joint_cycle.launch.py repeat_count:=1
  ros2 launch dofbot_two_joint_cycle two_joint_cycle.launch.py \
      moveit_config_package:=my_robot_moveit_config robot_name:=<机器人名> \
      demo_package:=my_robot_moveit_config
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.utilities import perform_substitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def _setup(context):
    # launch 参数要到运行期才能取到字符串，这里手动做替换
    robot_name = perform_substitution(context, LaunchConfiguration("robot_name"))
    moveit_config_package = perform_substitution(
        context, LaunchConfiguration("moveit_config_package"))
    demo_package = perform_substitution(context, LaunchConfiguration("demo_package"))
    demo_launch_file = perform_substitution(
        context, LaunchConfiguration("demo_launch_file"))

    moveit_config = MoveItConfigsBuilder(
        robot_name, package_name=moveit_config_package
    ).to_moveit_configs()

    demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare(demo_package), "launch", demo_launch_file]
            )
        )
    )

    cycle = Node(
        package="dofbot_two_joint_cycle",
        executable="two_joint_cycle",
        name="dofbot_two_joint_cycle",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {
                "goal_name": LaunchConfiguration("goal_name"),
                "repeat_count": ParameterValue(
                    LaunchConfiguration("repeat_count"), value_type=int
                ),
                "speed": ParameterValue(LaunchConfiguration("speed"), value_type=float),
                "pause_seconds": ParameterValue(
                    LaunchConfiguration("pause_seconds"), value_type=float
                ),
                "endpoint_tolerance": ParameterValue(
                    LaunchConfiguration("endpoint_tolerance"), value_type=float
                ),
            },
        ],
    )

    return [demo, TimerAction(period=30.0, actions=[cycle])]


def generate_launch_description():
    return LaunchDescription(
        [
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
            OpaqueFunction(function=_setup),
        ]
    )
