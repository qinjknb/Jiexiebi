# MoveIt 仿真与六舵机实机联动

本包订阅现有 `My_JXB_moveit/demo.launch.py` 发布的 `/joint_states`，将五个机械臂关节及一个夹爪关节的执行位置转换为 GCWJ 固件的 `MOVE ID 角度 时间` 命令。仿真控制器继续执行 MoveIt 轨迹，实机按相同的关节路径跟随。夹爪机构的其余关节是 URDF 的 `mimic` 关节，不需要单独舵机命令。

## 固件限制与上电顺序

**没有修改 `GCWJ`。** 当前 `GCWJ/User/main.c` 在每次 STM32 上电后会广播 `SETID 1`，让当时已经上电、接在 PA9 总线上的所有舵机都变成 ID 1。对于一块 STM32 共用六舵机的接法，必须先给 STM32 上电并等待至少 2 秒，确认固件已完成启动广播，**之后**再给已分别设置 ID 1～6 的六个舵机上电。若 STM32 在舵机供电期间复位，必须断开舵机电源、逐个重新设置 ID 1～6，再按该顺序启动。切勿对六个舵机同时发送 `SETID`，因为该命令本身也是广播。

桥接程序打开串口后先发送 `RANGE`，再对 ID 1～6 逐个 `PING`。任何一个 ID 无应答，就拒绝运动并重试。六个 ID 均在线后用 `READ` 检查实机起点与仿真位置的差，默认要求每轴不超过 10°；不对齐也拒绝运动。

接线沿用 `ServoHost/实测记录.md`：USB-TTL TX→STM32 PA3、RX→PA2、GND 共地；PA9 接舵机数据总线；舵机使用独立合适电源。串口默认 `/dev/ttyUSB0`、115200 8N1。

## 标定

编辑 [`config/bridge.yaml`](config/bridge.yaml) 中六个并行数组，对应 `Arm1_Joint`～`Arm5_Joint` 和 `grip_joint`，默认舵机 ID 1～6。换算公式：

```
舵机角度 = offsets_deg[i] + directions[i] × 关节角(rad) × scales_deg_per_rad[i]
```

默认 `offset=150°`、`direction=+1`、`scale=180/π` **仅是示例**，尚未按实机标定。先分别确定每轴零位、正方向、机械角度边界，再填写 `offsets_deg`、`directions`、`scales_deg_per_rad`、`min_degrees`、`max_degrees` 和 `range_deg`。完成并核对标定后，启动时才传入 `calibration_confirmed:=true`；否则实机模式不会打开串口或运动。超出配置边界的目标会被拒绝，不会悄悄裁剪到极限。仿真 `Arm5_Joint` 的全部范围超过示例映射的 300° 行程，需要按实际机械传动比和舵机行程设置标定值后才能走完整范围。

`/joint_states` 是仿真控制器的位置，并非真实舵机反馈；桥只在开始时用 `READ` 核对实机位置。由于未改固件，六轴命令按顺序发送，不能保证严格同一时刻起步；MoveIt 的执行成功也只代表仿真控制器成功。实际路径精度受舵机速度、负载、通信延迟和标定影响。

## 构建与运行（Ubuntu ROS 2）

在包含本仓库 `src` 的 ROS 工作区根目录：

```bash
source /opt/ros/jazzy/setup.bash
sudo apt install python3-serial
colcon build --packages-up-to dofbot_servo_bridge
source install/setup.bash
```

先在不连接舵机电源的状态验证六路指令（只打印、不打开串口）：

```bash
ros2 launch dofbot_servo_bridge mirror_demo.launch.py \
  test_mode:=true enable_motion:=true
```

在 RViz 的 MoveIt 面板规划、执行路径。日志会显示 `[DRY RUN] MOVE 1 ...` 至 `MOVE 6 ...`。默认 `enable_motion:=false`，所以只启动仿真时不会发指令。

完成实机标定、ID 检查及上电顺序后，运行：

```bash
ros2 launch dofbot_servo_bridge mirror_demo.launch.py \
  serial_port:=/dev/ttyUSB0 test_mode:=false enable_motion:=true \
  calibration_confirmed:=true
```

若要执行仓库现有的“机械臂到 init → 夹爪 close → 机械臂 up → 夹爪 open”路径一次：

```bash
ros2 launch dofbot_servo_bridge mirror_cycle.launch.py \
  serial_port:=/dev/ttyUSB0 test_mode:=false enable_motion:=true repeat_count:=1 \
  calibration_confirmed:=true
```

已有 MoveIt 仿真正在运行时，只启动桥：

```bash
ros2 launch dofbot_servo_bridge servo_bridge.launch.py \
  serial_port:=/dev/ttyUSB0 test_mode:=false enable_motion:=true \
  calibration_confirmed:=true
```

不要同时启动多个 bridge，也不要让 `ServoHost` 占用同一串口。需要让设备可访问时，可将用户加入 `dialout` 组并重新登录。在 `src/dofbot_servo_bridge` 目录内运行 `python3 -m unittest discover -s test -v`，可执行不依赖 ROS 的协议与映射测试。
