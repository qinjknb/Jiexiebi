# dofbot_hw_bridge —— DOFBOT Pro 机械臂 MoveIt 实机桥

把 MoveIt 的 `FollowJointTrajectory` 轨迹，按时间戳采样后通过 USB-TTL 发给 STM32，
由 STM32 用 `SYNC WRITE` 同步驱动 6 个德晟总线舵机；同时发布 `/joint_states`。

实机执行的是**同一条规划轨迹**（同样的路点、同样的时间戳），所以动作与 RViz 仿真一致。

```
MoveIt(move_group) ──action──► trajectory_bridge ──AA 55 帧/CRC16──► STM32 ──SYNC WRITE──► 6× 总线舵机
                                     └── /joint_states ──► RobotStatePublisher / RViz
```

---

## 1. 编译

```bash
cd ~/My_JXB
colcon build --symlink-install --packages-select dofbot_hw_bridge
source install/setup.bash
```

依赖：`sudo apt install python3-serial ros-jazzy-moveit ros-jazzy-moveit-configs-utils`

## 2. 零硬件自测（建议先做）

```bash
ros2 run dofbot_hw_bridge dofbot_selftest
# 期望末行：全部自测通过：协议、标定、链路、轨迹下发一致
```

它用内置的"假 STM32"（`serial_link.FakeMcuTransport`）跑完整的协议握手、
6 关节标定换算、2 秒轨迹下发、急停/回中，全部不碰硬件。

## 3. 运行

```bash
# ① 整条软件链路（不接机械臂）：RViz 里模型会跟着轨迹动
ros2 launch dofbot_hw_bridge arm_real.launch.py hardware:=false

# ② 接真机
ros2 launch dofbot_hw_bridge arm_real.launch.py port:=/dev/ttyUSB0

# ③ 顺带跑取放循环
ros2 launch dofbot_hw_bridge arm_real.launch.py port:=/dev/ttyUSB0 with_cycle:=true
```

然后在 RViz 里像仿真一样：拖 Goal → `Plan & Execute`。

### launch 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `hardware` | true | false = 虚拟模式（不打开串口，只发 `/joint_states`） |
| `port` | /dev/ttyUSB0 | USB-TTL 设备名 |
| `baud` | 115200 | 与固件 `ARM_LINK_BAUD` 一致 |
| `calib_file` | 包内 config/arm_calib.yaml | 关节标定文件 |
| `auto_home` | false | 启动后先回中位 |
| `home_time` | 2.0 | 回中位用时（秒） |
| `interpolation` | auto | `auto`(有速度用三次 Hermite) / `cubic` / `linear` |
| `with_cycle` | false | 启动 10 秒后自动跑 `dofbot_cycle_demo` |
| `arm_target` / `arm_home` | init / up | 取放循环的目标/回位姿态（SRDF 命名姿态） |
| `repeat_count` | 1 | 循环次数，0 = 无限 |

## 4. 命令行工具 `servo_tool`

```bash
ros2 run dofbot_hw_bridge servo_tool scan                  # 扫舵机 ID
ros2 run dofbot_hw_bridge servo_tool read                  # 读 6 个位置 + 换算角度
ros2 run dofbot_hw_bridge servo_tool move 2 20             # 关节2 到 +20°
ros2 run dofbot_hw_bridge servo_tool goto 0,0,-30,0,0,-20  # 6 关节一起到指定角度
ros2 run dofbot_hw_bridge servo_tool raw 2047,2047,2047,2047,2047,2047
ros2 run dofbot_hw_bridge servo_tool home --ms 2000
ros2 run dofbot_hw_bridge servo_tool stop
ros2 run dofbot_hw_bridge servo_tool limits                # 把标定文件的软限位写进固件
ros2 run dofbot_hw_bridge servo_tool busmode 1             # 切换总线下发方式
ros2 run dofbot_hw_bridge servo_tool monitor               # 实时状态帧
ros2 run dofbot_hw_bridge servo_tool text "PING 1"         # 发原文本命令给固件
```

公共参数：`--port /dev/ttyUSB0`、`--baud 115200`、`--calib <yaml>`、`--fake`（用假固件练习）。

## 5. 节点参数（也可用 `config/bridge_params.yaml`）

```
calib_file, port, baud, hardware, auto_home, home_time,
command_rate, state_rate, max_velocity, interpolation,
telemetry_source(auto|meas|cmd), arm_action, grip_action,
joint_states_topic, startup_timeout
```

## 6. 安全机制

| 机制 | 位置 | 行为 |
|---|---|---|
| 链路超时 | 固件 400ms | 收不到新目标 → 停在当前位置（IDLE） |
| 速度限速 | 固件 `ARM_MAX_DPS`、PC `max_velocity` | 每周期步进上限，防甩臂 |
| 软限位 | 固件 `SET_LIMITS` | 目标超限自动钳位并置 `LIMIT` 标志 |
| 软启动 | 固件前 400ms 步长减半 | 第一个目标不跳变 |
| 软急停 | 服务 `~/estop` | 立刻停发、拒绝新目标 |
| 上电安全 | 固件 | 上电不主动动作，等命令 |

```bash
ros2 service call /dofbot_hw_bridge/estop  std_srvs/srv/Trigger
ros2 service call /dofbot_hw_bridge/enable std_srvs/srv/SetBool "{data: true}"
ros2 service call /dofbot_hw_bridge/home   std_srvs/srv/Trigger
ros2 service call /dofbot_hw_bridge/status std_srvs/srv/Trigger
```

## 7. 模块说明

| 文件 | 职责 | 能否独立运行 |
|---|---|---|
| `protocol.py` | 帧编解码 + CRC16 | `python3 protocol.py` 自测 |
| `calib.py` | 关节角 ↔ 舵机位置、标定 YAML 读取 | 被其他模块调用 |
| `traj.py` | 轨迹采样（线性 / 三次 Hermite） | `python3 traj.py` 自测 |
| `serial_link.py` | 串口链路、超时、假 STM32 | 被其他模块调用 |
| `trajectory_bridge.py` | ROS 2 节点（动作服务器 + 状态发布 + 服务） | `ros2 run` |
| `servo_tool.py` | 命令行调试/标定 | `ros2 run` / `python3 -m` |
| `selftest.py` | 无硬件全链路自测 | `ros2 run` / `python3 -m` |

## 8. 排查

* **串口打不开**：`ls -l /dev/ttyUSB0`；权限 → `sudo usermod -aG dialout $USER` 重新登录。
* **PING 超时**：固件没烧 / TX-RX 接反 / 没共地 / 波特率不符。
* **方向反、幅度不对**：改 `config/arm_calib.yaml` 的 `direction`、`center`、`range_deg`。
* **想让第一条轨迹前先回中位**：`auto_home:=true`。
* **想看固件真实回读**：`telemetry_source: meas`（默认 auto 就是有遥测就用真实值）。
