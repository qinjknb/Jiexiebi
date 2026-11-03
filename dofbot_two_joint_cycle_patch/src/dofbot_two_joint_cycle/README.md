# DOFBOT PRO 双关节往返循环仿真

本程序针对提供的 `dofbot_pro_moveit` 包：`arm_group` 只包含 `Arm1_Joint`、`Arm2_Joint`；目标姿态 `test` 是 `(-0.9286, -0.5294)` rad。

启动时从 `/joint_states` 记录**实际初始角度**。每轮只做两段运动：

1. MoveIt 规划并执行从当前初始角度到 `test` 的去程。
2. 将同一条已规划关节轨迹的路点和时间顺序反向，等待回程执行结束并检查是否回到初始角度。

去程或回程任一步失败就停止，不会跳过错误继续下一轮。回程是同一条**指令轨迹**的反向播放；仿真控制器跟踪会有微小误差，因此用 `endpoint_tolerance` 检查实际关节状态。下一轮会重新规划一次去程。

## 安装与首次运行

先关闭已经运行的 `demo.launch.py`，避免同时启动两套 `move_group` 和控制器。将补丁 ZIP 复制到 Ubuntu 的 `~/Downloads/` 后，在 Ubuntu 终端运行：

```bash
unzip -o ~/Downloads/dofbot_two_joint_cycle_patch.zip -d ~/two_joint
cd ~/two_joint
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to dofbot_two_joint_cycle
source install/setup.bash
ros2 launch dofbot_two_joint_cycle two_joint_cycle.launch.py repeat_count:=1
```

以上假设你的工作空间是 `~/two_joint`，即其下已有 `src/dofbot_pro_moveit/`。如果工作空间实际路径不同，将命令中的 `~/two_joint` 换为真实路径。补丁只新增 `src/dofbot_two_joint_cycle/`，并把原 MoveIt 包的 `moveit.rviz` 中轨迹循环播放关掉；机器人模型、URDF 和 `test` 角度均未修改。建议事先备份自己的 `moveit.rviz` 布局。

首次运行观察终端应依次出现：`planning outbound` → `executing outbound` → `executing the exact joint path in reverse` → `Cycle 1 complete`。RViz 中应看到 Arm1/Arm2 到 `test` 一次、沿关节轨迹返回一次。

确认单轮正确后，关闭当前启动（Ctrl+C），再运行：

```bash
ros2 launch dofbot_two_joint_cycle two_joint_cycle.launch.py repeat_count:=3
```

`repeat_count:=3` 表示完整往返三轮；`repeat_count:=0` 持续重复直到 Ctrl+C。`speed:=0.1` 为速度和加速度缩放比例；`pause_seconds:=0.5` 让目标点和初始点各停半秒。`goal_name:=test` 可改成同一规划组中其他已定义的命名姿态。

每次重新启动程序都会重新读取当时的实际初始角度。想回到同一个固定的零位时，先让机器人回到零位，再启动本程序。本程序仅用于 ZIP 中的 `mock_components/GenericSystem` 仿真配置。

## 如果看起来动作仍重复

补丁设置了 `Loop Animation: false` 和 `Show Robot Visual: false`，避免 RViz 反复播放已规划轨迹或显示另一个叠加机器人。以终端的 `Cycle N`、去程和回程日志，以及 `/joint_states` 中的 Arm1/Arm2 实际角度判断真实执行次数。如果编译或执行失败，发来从第一条 `ERROR` 前后开始的完整终端日志。
