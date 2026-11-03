# DOFBOT PRO 双关节循环 · 三目录联动补丁（仿真 → 实物同步）

本补丁把三个项目串成一条完整的"**仿真 → 实物**"链路：MoveIt 仿真里 Arm1/Arm2
每走一步，关节角就实时换算成舵机角度，经 USB-TTL 发给 STM32，驱动真实德晟智能
舵机同步摆动。

## 一、三目录职责

| 目录 | 角色 | 运行位置 |
|---|---|---|
| `dofbot_two_joint_cycle_patch/`（本目录） | ROS 2 仿真与联动桥 | Ubuntu (ROS 2 Jazzy) |
| `GCWJ/` | STM32F103 下位机固件（文本命令 ↔ 德晟 v4.01 舵机总线） | STM32 板（烧录后运行） |
| `ServoHost/` | Python 上位机 GUI，手动调试/标定舵机 | Windows（可选） |

联动时的完整链路：

```
[Ubuntu]  dofbot_two_joint_cycle 仿真循环 (MoveIt, Arm1/Arm2)
              │  /joint_states (rad)
              ▼
          dofbot_servo_bridge 联动桥（本补丁新增）
              │  角度映射: 舵机角度 = offset + direction × rad × 180/π
              │  节流 20Hz + 死区 0.2° → "MOVE <id> <角度°> <时间ms>"
              ▼  USB-TTL 115200 (TX→PA3, RX→PA2, GND 共地)
[STM32]   GCWJ 固件 main.c（上位模式, USART2 收命令）
              │  PA9 单线半双工, 德晟 v4.01 协议
              ▼
[实物]    德晟串口总线智能舵机 (当前 ID=1，可扩展到 ID=1、2 对应两关节)
```

- `GCWJ` 固件、`ServoHost` 上位机**都不需要改动**（单舵机场景）；
- `ServoHost` 在联动中退居"调试工具"：换机调试、标定零点、改舵机 ID 时在
  Windows 上用，日常联动只靠 Ubuntu 上的桥。

## 二、目录结构

```
dofbot_two_joint_cycle_patch/
├── README.md                           本说明（总联动文档）
├── tools/                              ★ 一键运行机械臂工具（无需 ROS）
│   ├── play_arm.py                     一键运行机械臂（内置仿真路径/回放录制文件）
│   ├── record_path.py                  录制仿真 /joint_states 为路径文件
│   └── trajectories/sample_cycle.json 示例路径
└── src/
    ├── dofbot_pro_moveit/              原 MoveIt 配置包（未改动）
    ├── dofbot_two_joint_cycle/         双关节往返循环仿真节点（原补丁）
    └── dofbot_servo_bridge/            ★ 新增：仿真→实物联动桥
        ├── config/bridge.yaml          关节→舵机映射与串口参数
        ├── launch/servo_bridge.launch.py      只起桥（调试）
        ├── launch/bridge_cycle.launch.py      仿真循环+桥一起起（推荐）
        └── README.md                  桥的详细说明（参数/接线/双舵机扩展）
```

## 三、快速开始（Ubuntu）

```bash
unzip -o ~/Downloads/dofbot_two_joint_cycle_patch.zip -d ~/two_joint
cd ~/two_joint
source /opt/ros/jazzy/setup.bash
pip3 install pyserial
sudo usermod -aG dialout $USER        # 串口权限，改完重新登录
colcon build --packages-select dofbot_two_joint_cycle dofbot_servo_bridge
# 注意：必须把 dofbot_two_joint_cycle 一起构建，
# bridge_cycle.launch.py 会引用它的 two_joint_cycle.launch.py
source install/setup.bash

# 接线：USB-TTL TX→STM32 PA3、RX→PA2、GND→GND；舵机数据线接 PA9、外部供电
ros2 launch dofbot_servo_bridge bridge_cycle.launch.py \
    repeat_count:=1 serial_port:=/dev/ttyUSB0
```

启动顺序：串口打开 → 下发 `RANGE 300` → 收到 STM32 `READY STM32F103 SERVO HOST V1.0`
→ 约 30 秒后 move_group 就绪、循环开始 → 真实舵机跟随 Arm1_Joint 同步摆动。

无硬件自检（验证命令生成逻辑）：

```bash
ros2 launch dofbot_servo_bridge bridge_cycle.launch.py test_mode:=true
# 日志里出现 "[TEST] TX >>> MOVE 1 xxx.xx 100" 即联动正常
```

### 一键运行机械臂（无需 ROS，推荐日常使用）

不想每次起整套仿真时，直接跑工具脚本（Ubuntu/Windows 均可，只要串口在）：

```bash
python3 tools/play_arm.py                    # 自动找 CH340 串口（Windows: --port COM9）
python3 tools/play_arm.py --dry-run          # 无硬件演练，先看一遍命令流
```

运行即动：读舵机当前位置作起点 → 按仿真路径去 `test` 目标再原路返回，默认 3 轮，
全程打印进度，Ctrl+C 停止。想完全复现仿真里规划的那条轨迹，先用
`tools/record_path.py` 边跑仿真边录制，再 `play_arm.py --path 录制文件` 回放。
详见 [tools/README.md](tools/README.md)。

## 四、角度映射与标定

`config/bridge.yaml` 里每个关节独立配置：

```
舵机角度(°) = offset_deg + direction × 关节角(rad) × scale_deg_per_rad
```

默认 `offset=150`（行程中位）、`scale=57.2958`（180/π）、`direction=+1`，仿真目标
`test` (-0.9286, -0.5294) rad 对应 96.8° / 119.7°。实装时因减速比和安装零点不同，
按 [桥 README](src/dofbot_servo_bridge/README.md) 第 4 节"只起桥 + 手动发关节角"
逐项标定 offset 和 direction。

## 五、扩展到两个舵机（Arm1 + Arm2）

1. 固件 `GCWJ/User/main.c`：删除上电 `Servo_SetID(1);` 广播（约第 552 行），
   避免改掉总线第二个舵机的 ID；重新烧录。
2. 用 ServoHost（Windows）逐个设 ID=1、ID=2。
3. `config/bridge.yaml` 的 5 个并行数组都加第二项（文件内有现成注释）。
4. 重启联动即可。

## 六、常见问题速查

| 现象 | 排查 |
|---|---|
| 打开串口失败 | USB-TTL 是否插在 Ubuntu；`ls /dev/ttyUSB*`；dialout 组；串口被占 |
| 收到 READY 但舵机不动 | 舵机外部供电、共地、数据线 PA9；先在 Windows 用 ServoHost 验证 |
| 角度不对/乱动 | `range_deg` 与舵机型号（300/270/180）；offset/direction 符号 |
| 舵机跟不上 | 调小仿真 `speed`（如 0.05）、`move_time_ms` ≤ 150 |
| 上电后舵机自摆 | STM32 演示模式，第一条 MOVE 到达后自动停止并进入手动模式 |

详细参数与排查见 [src/dofbot_servo_bridge/README.md](src/dofbot_servo_bridge/README.md)；
仿真循环本身的说明见 [src/dofbot_two_joint_cycle/README.md](src/dofbot_two_joint_cycle/README.md)；
固件与上位机协议见 `GCWJ/User/main.c` 头注释与 `ServoHost/README.md`。
