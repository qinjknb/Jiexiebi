# 一键运行机械臂工具（tools/）

不依赖 ROS 的机械臂路径播放器 + 仿真路径录制器。只要 Python3（串口模式加
pyserial），在 **Ubuntu 或 Windows** 上都能用；串口接法与联动桥相同
（USB-TTL：TX→PA3、RX→PA2、GND 共地；舵机数据线接 PA9）。

```
tools/
├── play_arm.py             ★ 一键运行机械臂（内置仿真路径 / 回放录制文件）
├── record_path.py          录制仿真 /joint_states 为路径 JSON（Ubuntu 上运行）
└── trajectories/
    └── sample_cycle.json   示例路径（0 → test → 0，2 秒单程）
```

## 一、直接跑（内置仿真路径，推荐）

```bash
# Ubuntu
pip3 install pyserial
python3 tools/play_arm.py                      # 自动找 CH340 串口

# Windows（ServoHost 那台电脑也行，串口如 COM9）
python tools/play_arm.py --port COM9
```

运行时程序会：

1. 打开串口、发 `RANGE 300` 同步满行程，等待 STM32 `READY` 问候；
2. `READ` 读取各舵机**当前实际位置**作为本轮起点（读不到就用 `--home` 默认 150°）；
3. 生成仿真同款路径：起点 → `test` 目标(-0.9286, -0.5294 rad) → 原路返回，
   余弦缓入缓出，端点停 `--pause` 秒，默认 3 轮；
4. 按时间轴以 20 Hz 发送 `MOVE <id> <角度°> <时间ms>`，实时打印进度，Ctrl+C 停止。

常用参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `--port` | 自动找 CH340 | 串口设备（`/dev/ttyUSB0` / `COM9`） |
| `--repeat` | 3（文件模式 1） | 往返轮数 |
| `--segment-time` | 6.0 | 去程/回程单段秒数（仿真 speed≈0.1 的节奏） |
| `--pause` | 0.5 | 端点停顿秒数 |
| `--goal1` `--goal2` | -0.9286 / -0.5294 | 目标关节角(rad)，即仿真 `test` 姿态 |
| `--home1` `--home2` | 150.0 | 读不到舵机位置时的起点角度(°) |
| `--offset1/2` `--scale1/2` `--dir1/2` | 150 / 57.2958 / 1 | 角度映射（与 bridge.yaml 一致） |
| `--range` | 300 | 满行程角度 |
| `--joints` | 2 | 驱动关节数；只有 1 个舵机时用 `--joints 1` |
| `--dt-ms` | 50 | 指令间隔 ms |
| `--timescale` | 1.0 | 时间缩放（0.05 = 快 20 倍，演练用） |
| `--dry-run` | 关 | 无硬件演练：只打印命令，不需要 pyserial |
| `--verbose` | 关 | 打印全部串口应答 |

无硬件先演练一遍（任何机器）：

```bash
python3 tools/play_arm.py --dry-run --repeat 1 --segment-time 0.6 --pause 0.1 --timescale 0.1
```

## 二、回放录制的仿真路径（最忠实于仿真）

1. Ubuntu 上开两个终端：
   ```bash
   # 终端 1：跑仿真循环（约 30 秒后开始动）
   ros2 launch dofbot_two_joint_cycle two_joint_cycle.launch.py repeat_count:=1
   # 终端 2：录制实际执行的关节轨迹
   python3 tools/record_path.py -o tools/trajectories/my_path.json
   ```
2. 仿真循环结束后在终端 2 按 Ctrl+C，路径即保存；
3. 之后随时一键回放（不需要 ROS 环境，Ubuntu/Windows 均可）：
   ```bash
   python3 tools/play_arm.py --path tools/trajectories/my_path.json --repeat 1
   ```

录制文件是纯 JSON（`joints` + `waypoints[{t_ms, rad}]`），播放器会按 `--dt-ms`
自动重采样，录制频率高低都不影响回放。仓库里带了示例
[trajectories/sample_cycle.json](trajectories/sample_cycle.json)，可先试：
`python3 tools/play_arm.py --path tools/trajectories/sample_cycle.json --dry-run`。

## 三、注意事项

1. **两个舵机**：`--joints 2` 会同时发 ID=1、ID=2 的指令；只接了一个舵机时
   用 `--joints 1`（固件改不改都能用，ID=2 无舵机时只是没有实物响应）。
2. **角度映射**：默认假设舵机 150° 对应 0 rad、行程 300°；实物安装零点不同时
   调 `--offset1/--offset2`（= 0 rad 对应的舵机角度）和 `--dir1/--dir2`（±1）。
3. **节奏**：`--segment-time` 越大越慢越平滑；默认 6 秒对应仿真 `speed:=0.1`
   的节奏，太快时舵机扭矩不足会丢步，建议从 6 秒起步。
4. **与实时联动桥的关系**：本工具是"离线播放"——路径固定、不需要仿真在场；
   需要"仿真走到哪舵机跟到哪"的实时同步时用
   `ros2 launch dofbot_servo_bridge bridge_cycle.launch.py`（见桥的 README）。
