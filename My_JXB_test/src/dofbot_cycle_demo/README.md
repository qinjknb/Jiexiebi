# DOFBOT PRO 仿真循环任务

此 ROS 2 包调用 MoveIt，按顺序执行以下四步。每一步都先规划，再等待执行结果；失败时立即停止，不会继续下一步。

1. `arm_group` 到 `init`
2. `grip_group` 到 `close`
3. `arm_group` 回 `up`
4. `grip_group` 到 `open`

默认运行 3 轮。`init`、`up`、`close`、`open` 均来自现有 `My_JXB_moveit/config/DOFBOT_Pro-V24.srdf`。现有 `initial_positions.yaml` 中五个机械臂关节初始值都为 0，与 `up` 相同。

## 安装

将整个 `dofbot_cycle_demo` 文件夹复制到 Ubuntu 工作区的 `~/My_JXB/src/`。确保 `My_JXB_moveit` 已在同一工作区，且夹爪控制器已改为 `JointTrajectoryController` / `FollowJointTrajectory`。压缩包的 `moveit_config_updates/` 附有这两份已修改的控制器配置；如果 Ubuntu 源码仍是旧配置，请先用它们覆盖 `~/My_JXB/src/My_JXB_moveit/config/` 中的同名文件。

关闭此前运行的 `demo.launch.py`，然后在 Ubuntu 终端执行：

```bash
cd ~/My_JXB
colcon build --packages-up-to dofbot_cycle_demo
source install/setup.bash
ros2 launch dofbot_cycle_demo cycle_demo.launch.py
```

这一个启动命令会打开原有 MoveIt demo，并在 30 秒后启动循环节点。30 秒用于等待 MoveIt 和 RViz 初始化；循环节点还会等待关节状态。终端会打印每一步的规划、执行和完成信息。

## 改目标和次数

默认机械臂目标为 `init`、返回位姿为 `up`。要把目标改为现有的 `down` 位姿并只运行 1 轮：

```bash
ros2 launch dofbot_cycle_demo cycle_demo.launch.py arm_target:=down repeat_count:=1
```

`repeat_count:=0` 表示持续循环，直到按 Ctrl+C。`arm_speed` 和 `gripper_speed` 是 0 到 1 之间的速度及加速度比例，默认均为 0.1，例如 `gripper_speed:=0.2`。命名位姿必须先存在于 SRDF；在 RViz 拖动出的临时位姿不能直接作为 `arm_target` 名称使用。

## 验证

1. 首次先用 `repeat_count:=1` 看完整的四步顺序。
2. 终端应依次出现 `arm to target: complete`、`close gripper: complete`、`arm to home: complete`、`open gripper: complete` 和 `Cycle 1 complete`。
3. 如果出现 `planning failed` 或 `execution failed`，循环会停在该步。先检查 `ros2 control list_controllers` 是否显示两个控制器都为 `active`，并检查 `ros2 action list` 中是否有 `/grip_group_controller/follow_joint_trajectory`。

这是纯仿真包。运行时不要同时运行厂家将 `/joint_states` 转换为实机命令的 `SimulateToArm` 程序。
