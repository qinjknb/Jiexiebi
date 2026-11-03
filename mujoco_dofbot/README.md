# dofbot_mujoco —— DOFBOT Pro 在 Windows + MuJoCo 上跑关节循环

把原来「Ubuntu 上 ROS 2 + MoveIt + 舵机桥」那套搬到 Windows：用 MuJoCo 做仿真，
不再需要 ROS、colcon、move_group 和 RViz，同时保留原来的 STM32 / 德晟舵机链路。

## 一、和原方案的关系

| 原方案（Ubuntu） | 本方案（Windows） |
|---|---|
| `dofbot_pro_moveit` + `demo.launch.py` | `models/dofbot_pro.xml`（从同一份 URDF 转出） |
| MoveIt 规划 → `mock_components` 执行 | `dofbot_mj/motion.py` 余弦缓入缓出 + 直接摆关节 |
| `dofbot_cycle_demo`（arm→init→夹爪→arm→up→夹爪） | `config/default.yaml` 里的 `cycle.steps` |
| `/joint_states` 话题 | 仿真循环每个周期直接 `servo.update()` |
| `dofbot_servo_bridge` 节点 | `dofbot_mj/servo_link.py`（同样的三道闸和 MOVE 协议） |
| RViz 观察 | MuJoCo 交互窗口 |
| 命名姿态来自 SRDF | **直接读同一份 SRDF**，角度不会和 Ubuntu 侧走偏 |

角度映射公式、MOVE 文本协议、满行程 300°、位置码 `= 角度×4095/300` 全部沿用，
所以 `GCWJ` 固件和 `ServoHost` 上位机都不用改。

## 二、快速开始

```powershell
cd D:\jixiebi_pro\3\mujoco_dofbot
# 已在本目录创建 .venv；搬到其他电脑后需重新创建并安装依赖
.\.venv\Scripts\python.exe -m pip install -r requirements.txt

.\.venv\Scripts\python.exe tools\check_env.py  # 环境自检
.\.venv\Scripts\python.exe run.py --headless --speed 0 --repeat 1 --servo dry
.\.venv\Scripts\python.exe run.py              # 开 MuJoCo 窗口跑循环
```

`python run.py` 默认就是**试运行**：MuJoCo 里手臂会动，串口只打印
`[DRY] MOVE 1 150.00 120 | ...`，不会碰真机。确认动作方向对之后再开实机同步。

## 三、模型：从 URDF 生成 MJCF

模型不是手画的，是从 `src/dofbot_pro_description/urdf/DOFBOT_Pro-V24.urdf` 转出来的：

```powershell
python -m dofbot_mj.urdf2mjcf ..\src\dofbot_pro_description\urdf\DOFBOT_Pro-V24.urdf -o models\dofbot_pro.xml
```

转换器做的事：

* `package://` 网格路径解析到本地 `meshes/`，单位自动判断（本次是米，缩放 1.0）；
* URDF 的惯性张量做特征分解，写成 MuJoCo 的 `diaginertia` + 主轴 `quat`
  （避免 `fullinertia` 字段顺序踩坑）；
* **MuJoCo 单个 STL 上限 20 万面**，`Arm4_Link` 有 21.8 万面，按面数原样切成
  `models/mesh_parts/Arm4_Link_p0/p1.STL` 两份再各挂一个 geom —— 形状零损失，
  不是减面；
* URDF 的 5 个 mimic 夹爪关节转成普通 hinge，关系写进
  `models/dofbot_pro.json`，运行时按 `q = multiplier × q_grip + offset` 驱动；
* 底盘按网格最低点抬高 15 mm，正好坐在地面上；
* 非 fixed 关节自动生成位置执行器（`physics` 模式用）。

生成的 `models/dofbot_pro.xml` 是普通 MJCF 文本，可以直接手改。

## 四、动作序列

`config/default.yaml` 的 `cycle.steps` 与 `src/dofbot_cycle_demo/src/cycle_demo.cpp`
一致，每项引用 SRDF 里的命名姿态：

```yaml
cycle:
  steps:
    - arm: init      # 机械臂去 init 姿态
    - grip: close    # 夹爪闭合
    - arm: up        # 机械臂回 up
    - grip: open     # 夹爪张开
  repeat: 3
  segment_seconds: 6.0   # 每段时长，对应 MoveIt speed≈0.1 的节奏
  pause_seconds: 0.5
  profile: cosine         # cosine 缓入缓出 | linear
endpoint_tolerance_rad: 0.01   # 每段终点校验（对应 C++ 的 endpoint_tolerance）
```

可用姿态（来自 `src/My_JXB_moveit/config/DOFBOT_Pro-V24.srdf`）：

```powershell
python run.py --list-poses
```

想换动作只需改这几行，不用改代码。`hold:` 可以给单步单独设停顿时长。

### 实测姿态（扫描位等）

除了 SRDF 里的姿态，还能把**实机摆好的姿态**录下来复用，存在 `config/poses.yaml`：

```powershell
# 把臂摆到位，然后录一个名字
python tools\record_pose.py --name scan --note "扫描位（深度相机取景点）"

# 之后随时把实机开回这个位（按记录的舵机角度下发）
python tools\servo_host_gui_drive.py --pose scan

# 也能在 cycle.steps 里直接引用
#   - arm: scan
```

每条记录里存两份数据：`servo_deg`（六个舵机角，用来驱动实物）和
`joint_rad`（换算后的关节角，用来在 MuJoCo 里摆出同一姿态）。
`tools/record_pose.py` 会打印成表，便于核对。

> 已记录：`scan`（2026-10-31，Arm1 +0.004 / Arm2 −0.973 / Arm3 +1.597 /
> Arm4 +1.413 / Arm5 +0.031 / grip −1.584 rad）

## 五、控制模式

| 模式 | 说明 |
|---|---|
| `kinematic`（默认） | 直接把关节摆到目标角，跟踪无误差 —— 等价原方案的 `mock_components` |
| `physics` | 走位置执行器 + 真实重力，看起来更像真机，会有毫秒级跟随滞后 |

```powershell
python run.py --mode physics
```

## 六、驱动实物舵机

```powershell
python run.py --servo off                          # 完全不碰串口
python run.py                                      # 试运行(dry)：只打印 MOVE
python run.py --servo live --calibration-confirmed # 真开 COM9，仿真↔实物同步
python run.py --servo live --port COM5 --calibration-confirmed
```

三道闸沿用原设计：

1. `test_mode=true`（默认）：只打印，不打开串口；
2. `enable_motion`：允许发送运动指令；
3. `calibration_confirmed`：确认六轴方向/零点已标定。
   **没确认就 `--servo live` 会被拒绝**，只留一句提示，不打开串口。

连上后还会做**起点对齐检查**（`start_tolerance_deg`，默认 10°）：先 `READ` 六个
舵机的实际位置，和仿真目标对不上就拒绝运动，避免"上次停在别处，一上电猛地拽过去"。
所以实机同步前，先让实物大致停在仿真起始姿态（`up`，六轴都约 150°），
或把 `start_tolerance_deg` 调大让它主动拉到起点。

串口接法与 `ServoHost` 完全一样：

```
电脑 USB ──CH340──> STM32 PA3/PA2 + GND 共地
STM32 PA9 ──> 舵机数据线（外部供电）
```

## 七、实机同步 demo（相对运动，安全起步）

实机第一次上电时，**不要**直接命令 SRDF 的绝对姿态：`bridge.yaml` 里那套
`offset=150 / direction=+1` 是占位值，没标定过。用只读探测看一眼就知道——
本机实测按该映射换算，`Arm4_Joint` 读到 +101.5°、`grip_joint` 读到 +59.6°，
都超出了 URDF 限位。

所以 demo 用**相对运动**：仿真从实机实测姿态起步，每轴只走一个小角度差。
这样即使绝对零点不对，仿真和实物走的仍是同一个角度差，也不会撞限位。

```powershell
# 1) 只读探测（PING + READ，不发运动指令），并把仿真起点设成实机姿态
python run.py --start-from-servo --headless --speed 0 --repeat 1 --servo dry

# 2) 六轴小幅摆动 demo
python run.py --config config\demo.yaml --start-from-servo --servo live

# 3) 夹爪单轴 demo（五轴保护停转时的可用版本）
python run.py --config config\demo_grip.yaml --start-from-servo --servo live
```

`cycle.steps` 支持三种写法，可混用：

```yaml
steps:
  - arm: init                     # SRDF 命名姿态（绝对）
  - grip: close
  - joints: {Arm1_Joint: 0.3}     # 绝对关节角
  - relative: {grip_joint: 0.2}   # 相对「初始姿态」的增量（demo 用这种）
```

### 实测中碰到的两个硬件 / 固件事实

1. **固件回的应答是 `OK MOVE <id>`，不带位置码**，而 `main.c` 头注释和原
   Ubuntu 桥的匹配串都写成 `OK MOVE <id> `（带尾随空格）——原版会一直等到超时。
   本包的 `protocol.request()` 已改成按 token 边界匹配，两种应答都认。
2. **ID1~ID5 拒绝执行位置写入**：能 PING、能 READ，但 `MOVE` 后位置纹丝不动，
   连固件自身的 `DEMO` 摆动也不动；同一时刻 ID6（夹爪）执行 `MOVE` 正常。
   这只能说明通信与运动响应不同步，尚不能据此确认输出级保护或堵转。
   2026-11-03 用户补充舵机使用 5V 供电；需核对具体型号的额定电压、
   电源电流能力和动作时舵机端电压，再区分欠压、负载、回差和保护状态。
   相关待核对项见 `config/servo_power.yaml`；ID4 补偿尚未通过往返验证。

## 八、常用命令

### 实物照片姿态对照

`config/visual_match.yaml` 是根据 2026-10-31 的实物照片和同一时刻六轴只读
位置估算的显示映射。运行下列命令可重新读取舵机位置并渲染 MuJoCo 姿态：

```powershell
.\.venv\Scripts\python.exe tools\live_pose.py --render out\pose_match\adjusted_live.png
```

默认观察方位角为 180°，可用 `--azimuth` 与 `--elevation` 调整视角。
可视化模型 `models/dofbot_pro_visual.xml` 还把底板安装方向转了 180°，
使臂朝照片中靠显示器的左侧短边，并将相机移到 ID5 上方；ID5 显示方向
也按实物改正。原始 URDF 和用于运动的 `models/dofbot_pro.xml` 未改。
这份配置关闭实机运动开关；照片估算的零位只适合视觉核对，不能作为精确抓取标定。

实时显示实机姿态与 Orbbec 彩色深度点云：

```powershell
.\.venv\Scripts\python.exe tools\depth_live_follow.py --hz 5 --points 3000
```

此窗口持续读取六轴和深度帧，关闭窗口即可释放 COM9 与相机。
当前相机坐标轴和光心位置仍是目视估计，点云与机器人之间的精确重合
需要进一步手眼标定。

### 实机小幅联动与真实深度点云

在当前工作区的 Windows 环境中，先安装 `requirements_demo.txt`，再运行：

```powershell
.\.venv\Scripts\python.exe -m pip install -r requirements_demo.txt
.\.venv\Scripts\python.exe tools\demo_live_depth.py --id 6 --delta-deg -3 --out out\live_depth_grip
```

脚本先读取六轴实测位置，只向选中的舵机发送一次小幅运动指令，
在峰值读取 Orbbec 深度帧并输出相机坐标系三维点云，然后回到起点并读回核对。
它还输出 MuJoCo 的起点和峰值姿态图。相机坐标系点云没有与机器人世界坐标
强行重合，因为当前手眼外参只是近似估计。该命令需要访问 COM9 和深度相机。

```powershell
python run.py --headless --speed 0 --repeat 1   # 全速自检，无窗口
python run.py --speed 2                          # 2 倍速看
python run.py --repeat 0                         # 一直循环到 Ctrl+C
python run.py --segment-seconds 2                # 临时加快每段动作
python run.py --list-joints                      # 关节名/限位/mimic 关系
python run.py --pose init --render out\init.png  # 渲染某个姿态出图
python tests\test_smoke.py                       # 14 项自检
```

## 九、目录结构

```
mujoco_dofbot/
├── run.py                      入口（窗口/无头/渲染/列表）
├── config/default.yaml         所有可调参数
├── models/
│   ├── dofbot_pro.xml          从 URDF 生成的 MJCF
│   ├── dofbot_pro.json         关节/mimic/执行器 sidecar
│   └── mesh_parts/             超限网格的拆分件
├── dofbot_mj/
│   ├── urdf2mjcf.py            URDF → MJCF 转换器
│   ├── config.py               配置加载
│   ├── model.py                关节索引、mimic 展开、两种控制模式
│   ├── motion.py               余弦轨迹与时间线
│   ├── protocol.py             文本协议 + 角度映射（对齐 servo_protocol.py）
│   ├── servo_link.py           舵机桥线程（三道闸、对齐检查、串口重连）
│   ├── poses.py                SRDF 命名姿态解析
│   └── runner.py               播放器主循环
├── tools/
│   ├── check_env.py            环境自检
│   └── inspect_model.py        模型检查 + 预览渲染
└── tests/test_smoke.py         14 项无 ROS 自检
```

## 十、排错

| 现象 | 处理 |
|---|---|
| `找不到模型` | 按第三节跑一次转换器 |
| 打不开窗口 | 用 `--headless`；或确认显卡驱动/OpenGL |
| 手臂动作和真机方向相反 | 改 `config/default.yaml` 里对应关节的 `direction: -1.0` |
| 舵机不动但日志有 MOVE | 检查舵机外部供电、共地、数据线 PA9，先用 `ServoHost` 验证链路 |
| 日志一直"起点未对齐" | 把实物摆到 `up` 姿态附近，或加大 `start_tolerance_deg` |
| 想换机器人 | `--model 你的.xml`，或改 `config` 里 `model.mjcf` |
