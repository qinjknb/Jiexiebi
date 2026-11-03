# dofbot_servo_bridge —— ROS2 仿真 → 德晟智能舵机 同步桥

把 MoveIt 仿真里的关节角实时转成德晟智能舵机命令，经 USB-TTL 发给 STM32（GCWJ
固件），让真实舵机跟着仿真同步摆动。**仿真在 Ubuntu 上跑，串口也接在 Ubuntu 上。**

## 一、联动链路

```
┌─────────────────────────── Ubuntu ───────────────────────────┐
│ dofbot_two_joint_cycle (MoveIt 仿真)                          │
│   └─ /joint_states (Arm1_Joint / Arm2_Joint, rad)            │
│                        │ 订阅                                 │
│ dofbot_servo_bridge (本包)                                    │
│   ├─ 角度映射 + 频率节流 + 死区过滤                           │
│   └─ "MOVE <id> <角度°> <时间ms>" 文本行 (115200-8-N-1)      │
└──────────────────────────┬───────────────────────────────────┘
                    USB-TTL (TX→PA3, RX→PA2, GND 共地)
┌──────────────────────────▼───────────────────────────────────┐
│ STM32F103 (GCWJ/User/main.c, 上位模式)                        │
│   └─ PA9 单线半双工 → 德晟 v4.01 协议 → 智能舵机 (ID=1)      │
└──────────────────────────────────────────────────────────────┘
```

文本命令协议与 [ServoHost](../../../ServoHost/README.md) 上位机、`GCWJ/User/main.c`
固件完全一致，上位机与固件都不需要改动。

## 二、接线（Ubuntu 侧）

| USB-TTL | STM32 | 说明 |
|---|---|---|
| TX | PA3 (USART2_RX) | 桥发出的 MOVE 命令 |
| RX | PA2 (USART2_TX) | STM32 应答（READY/OK） |
| GND | GND | **必须共地** |

舵机数据线接 PA9、外部电源供电，与《实测记录》的接法相同，只是把 USB-TTL 从
Windows 电脑换插到 Ubuntu 机器。

## 三、安装

```bash
cd ~/two_joint
source /opt/ros/jazzy/setup.bash
pip3 install pyserial            # 真实串口模式依赖
colcon build --packages-select dofbot_two_joint_cycle dofbot_servo_bridge
# 两个包都要构建：bridge_cycle.launch.py 会引用 dofbot_two_joint_cycle 的 launch
source install/setup.bash
```

串口权限：`sudo usermod -aG dialout $USER` 后重新登录（CH340 一般插上即 `ttyUSB0`）。

## 四、运行

### 1. 全联动：仿真循环 + 桥一起启动（推荐）

```bash
ros2 launch dofbot_servo_bridge bridge_cycle.launch.py \
    repeat_count:=1 serial_port:=/dev/ttyUSB0
```

启动后桥会：打开串口 → 下发 `RANGE 300` → 收到 STM32 `READY` 问候 → 仿真循环
开始动（约 30 秒后，等 move_group 起来）→ 真实舵机同步摆动。终端里每条发出的
命令都可用 debug 级别日志查看：

```bash
ros2 launch dofbot_servo_bridge bridge_cycle.launch.py repeat_count:=1 \
    serial_port:=/dev/ttyUSB0 --ros-args --log-level dofbot_servo_bridge:=debug
```

### 2. 只起桥（手动调试）

```bash
ros2 launch dofbot_servo_bridge servo_bridge.launch.py serial_port:=/dev/ttyUSB0
```

手动发布关节角即可让舵机动（-0.5 rad ≈ 121.35°）：

```bash
ros2 topic pub /joint_states sensor_msgs/msg/JointState \
  "{name: ['Arm1_Joint'], position: [-0.5]}"
```

### 3. 无硬件自检（任何机器都能跑，无需串口/舵机/pyserial）

```bash
ros2 launch dofbot_servo_bridge servo_bridge.launch.py test_mode:=true
ros2 topic pub -r 20 /joint_states sensor_msgs/msg/JointState \
  "{name: ['Arm1_Joint'], position: [-0.2]}"
```

桥日志里应出现 `[TEST] TX >>> MOVE 1 138.54 100` 之类的行，且频率不超过
`max_rate_hz`。也可以组合启动整条链路验证：`bridge_cycle.launch.py test_mode:=true`，
此时只做仿真、不驱动实物。

## 五、参数（config/bridge.yaml）

| 参数 | 默认 | 说明 |
|---|---|---|
| `serial_port` | `/dev/ttyUSB0` | 串口设备（`ls /dev/ttyUSB*` 查看；也可用 `/dev/serial/by-id/...`） |
| `baudrate` | `115200` | 与固件 USART2 一致 |
| `range_deg` | `300` | 舵机满行程角度，与固件/上位机一致（180/270/300/360） |
| `move_time_ms` | `100` | 每条 MOVE 的运行时间；20 Hz 发送时取 100~150 ms 更平滑 |
| `max_rate_hz` | `20.0` | 命令发送频率上限 |
| `deadband_deg` | `0.2` | 角度死区，变化小于该值不重复发送 |
| `test_mode` | `false` | 虚拟串口自检模式 |
| `joint_names` | `[Arm1_Joint]` | 要联动的关节名（并行数组，与下面几项一一对应） |
| `servo_ids` | `[1]` | 对应舵机 ID |
| `offsets_deg` | `[150.0]` | 舵机角度零点偏移（0~4095 行程中位是 150°） |
| `scales_deg_per_rad` | `[57.29578]` | 弧度→角度比例（180/π；有减速比就乘减速比） |
| `directions` | `[1.0]` | 方向（-1 反转） |

**角度映射公式**：`舵机角度(°) = offset_deg + direction × 关节角(rad) × scale_deg_per_rad`，
超出 `[0, range_deg]` 自动钳位。仿真目标 `test` 为 (-0.9286, -0.5294) rad，用默认
参数对应 96.8° / 119.7°，都在舵机行程内。

## 六、扩展到两个舵机（Arm1 + Arm2）

1. **固件**（`GCWJ/User/main.c`）：删掉上电的 `Servo_SetID(1);` 广播（约第 552 行），
   否则总线上第二个舵机的 ID 会被改掉。烧录后用 ServoHost 的"设置舵机ID"分别给
   两个舵机设 ID=1、ID=2（一次只接一个设）。
2. **config/bridge.yaml**：按文件内注释把 5 个并行数组都加第二项
   （`joint_names: [Arm1_Joint, Arm2_Joint]`、`servo_ids: [1, 2]` 等）。
3. 重启桥即可，无需改代码。两条 MOVE 会先后发出（间隔一个发送周期），
   如需严格同步起步，可在固件里加 `Servo_MoveSync` 的文本命令（后续可做）。

## 七、常见问题

1. **日志报"打开串口失败"**：确认 USB-TTL 已插 Ubuntu、`ls /dev/ttyUSB*` 存在、
   没被其他程序占用；加入 `dialout` 组后重新登录。
2. **串口打开、舵机不动**：确认 STM32 已烧录上位模式 `main.c`（收到 `READY` 问候）；
   检查舵机外部供电、共地、数据线接 PA9；先开 `ServoHost` 在 Windows 上验证链路，
   再换到 Ubuntu。
3. **舵机角度不对/乱动**：核对 `range_deg` 与舵机型号一致（300/270/180），
   以及 offset/direction/scale 的符号；用第 4 节"只起桥 + 手动发关节角"逐项标定。
4. **舵机动作滞后**：`move_time_ms` 别超过发送周期的 2~3 倍（20 Hz 时 ≤ 150 ms）；
   仿真循环的 `speed` 参数调小（如 0.05）让轨迹更慢更平滑。
5. **STM32 的 DEMO 抢动作**：上电演示收到第一条 MOVE 会自动停止并进入手动模式，
   属正常现象；想恢复演示发 `DEMO`。
