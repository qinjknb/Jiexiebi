# 草莓识别（最小可运行包）

从主项目 `E:\PythonProject` 抽出的**只含识别部分**的包，用于和机械臂对接。
不含训练数据、不含训练脚本、不含机械臂代码。

---

## 一、这个包能做什么

```
深度相机取图 -> YOLO 识别草莓(品种+成熟度) -> 算每颗的 3D 坐标 -> 打印/显示
```

输出示例：

```
[15:11:48] 检出 2 颗草莓：
   #1 good/好    置信0.72 | 距离  312 mm | 坐标 X=  -19 Y=   63 Z= 311 mm | 直径 49 mm
   #2 medium/中  置信0.65 | 距离  405 mm | 坐标 X=   52 Y=  -18 Z= 404 mm | 直径 43 mm
```

坐标含义：**相机坐标系，单位毫米**。
`X` 向右、`Y` 向下、`Z` 向前（沿光轴，就是到相机的距离）。

> 要接到机械臂，需要再做一次 **手眼标定**把相机坐标转成机械臂基座坐标
> （`R_cam2base` + `t_cam2base`，4x4 矩阵）。这部分在主项目的 `calib/` 里，本包未包含。

---

## 二、文件夹结构

```
最小文件夹/
├── strawberry_demo.py   识别主程序（入口）
├── orbbec_v1.py         Orbbec 深度相机封装（ctypes 调 SDK）
├── weights/
│   └── best.pt         YOLO 模型（mAP50 0.728，5 个类别）
├── sdk/                 Orbbec SDK v1 运行时（11.7 MB，已裁掉用不到的部分）
├── requirements.txt     Python 依赖
├── run.bat             一键启动（会检查管理员权限）
└── README.md           本文件
```

---

## 三、环境要求（重要）

| 项 | 要求 |
|---|---|
| 操作系统 | Windows 10/11 x64 |
| Python | **3.9**（3.10/3.11 一般也可以，但 3.9 是实测环境） |
| 显卡 | 有 NVIDIA GPU 更好（自动用 CUDA），没有也能跑 CPU（慢些） |
| 权限 | **必须管理员权限运行**，否则相机打不开（见"已知问题 1"） |
| 相机 | 奥比中光 DaBai DCW2（本项目配套） |

---

## 四、安装

当前工作区已在 `.venv` 配置 Python 3.11、文档锁定的依赖和 PyTorch CPU 版。
可直接运行 `.\.venv\python.exe strawberry_demo.py --help` 检查入口；
`run.bat` 会优先使用这个解释器。以下步骤用于在其他电脑上重新安装。

```powershell
# 1) 建虚拟环境（强烈建议，别装到全局）
python -m venv .venv
.\.venv\Scripts\activate

# 2) 装依赖（HTTPS 被墙时用阿里云镜像）
pip install -r requirements.txt
pip install -i http://mirrors.aliyun.com/pypi/simple/ --trusted-host mirrors.aliyun.com -r requirements.txt

# 3) 如果 pip 要自己装 torch，按你的显卡选：
#    CUDA 12.1（有独显）
pip install torch==2.5.1 torchvision==0.20.1 --index-url https://download.pytorch.org/whl/cu121
#    纯 CPU（体积小很多，装得快）
pip install torch==2.5.1 torchvision==0.20.1 --index-url https://download.pytorch.org/whl/cpu
```

---

## 五、运行

**方式一（推荐）**：右键 `run.bat` → **以管理员身份运行**

**方式二**：管理员 PowerShell

```powershell
cd 最小文件夹
.\.venv\Scripts\python.exe strawberry_demo.py
```

### 常用参数

```powershell
.\.venv\Scripts\python.exe strawberry_demo.py --no-show    # 不弹窗口，只看控制台
.\.venv\Scripts\python.exe strawberry_demo.py --quiet      # 控制台少打印
.\.venv\Scripts\python.exe strawberry_demo.py --conf 0.5   # 调高置信度阈值
```

### 按键

| 键 | 作用 |
|---|---|
| `q` / `ESC` | 退出 |
| `s` | 保存当前帧（彩色+深度 PNG） |
| `d` | 切换深度图叠加显示 |

---

## 六、已知问题（务必看）

### 1. 必须用管理员权限，否则相机打不开

普通权限下会报：

```
MFCreateDeviceSource(...) returned: HResult 0x80070005: "拒绝访问。"
```

`0x80070005` 就是 `E_ACCESSDENIED`（权限不足）。这是本机的安全策略/软件拦截，
不是代码 bug，也不是相机坏了（Windows「相机」应用能正常出图就是证据）。

**解决：用管理员权限运行。** 如果不想每次提权，把 `python.exe` 加入安全软件的
摄像头白名单即可。

### 2. 长时间运行可能内存报错（本包已缓解）

现象（原版代码）：

```
Alloc frame buffer failed! Out of max size! allocated=2047.878MB
```

原因：SDK 有 2048MB 帧缓冲池，每帧必须调用 `ob_delete_frame` /
`ob_delete_frameset` 归还，原版代码漏了这两个调用，约 **1460 帧
（15fps 下约 97 秒）** 就打满。

**本包已在 `orbbec_v1.py` 里补上帧释放**，正常情况下不会再出现。
如果仍然出现，说明还有别的泄漏点，把日志发给主项目维护者。

### 3. 相机必须距目标 0.4~0.8 米

实测：距离 <30cm 时结构光盲区导致深度稠密度只有 3%；>40cm 可达 75~93%。
程序里 `DEPTH_VALID_MIN = 250` 会把近距离无效值过滤掉。

### 4. 彩色流必须是 MJPG

改 `COLOR_PROFILE_INDEX` 换成 RGB888（921KB/帧）会因为 USB2.0 带宽不足，
深度流从 6/6 掉到 4/6。**不要改**。

### 5. 万一 sdk/ 报缺 DLL

`sdk/` 是从完整的 OrbbecViewer 目录按 **PE 导入表依赖分析**裁出来的
（115.8 MB → 11.7 MB，只保留 `OrbbecSDK.dll` 真正依赖的那些）。
理论上够了，万一运行时报"找不到 xxx.dll"：

把完整目录里的同名文件拷进 `sdk/` 即可。完整目录在主项目：

```
OrbbecViewer_v1.10.27_202509252154_win_x64_release/
```

**裁掉的都是这些**（确认与本用途无关，共约 104 MB）：
`avcodec-58.dll`(42.5MB)、`avformat-58.dll`(10MB)、`avfilter-7.dll`(7.8MB)、
`libcrypto/libssl`(3.9MB)、`OrbbecViewer.exe`(2.4MB)、`USBDownloadTool.exe`(2.1MB)、
FFmpeg 其余组件、`glew32.dll`（OpenGL）、`u-boot.bin`/`usb_updater.bin`（固件升级用）。

> 判断依据：`OrbbecSDK.dll` 的导入表和内部字符串里**都没有**这些名字，
> 说明运行时不会去加载它们。

---

## 七、给接手同学的技术要点

1. **相机开关流程不能乱**：必须
   `OrbbecDCW2()` → `enable_streams([...], align_mode=2)` → `get_frame_pair()`
   → `close()`。分两次 `start()` 会互相顶掉。

2. **`align_mode=2` 必须开**：开启 D2C 对齐后深度图变成 640x480 且与彩色同坐标系，
   YOLO 的像素坐标可以直接查深度。不开的话映射会取到背景（实测 NCC 仅 0.073）。

3. **只走 Orbbec SDK v1（ctypes）**：OpenNI2、SDK v2 都试过，不兼容这台相机。
   `sdk/` 目录里就是 v1 运行时，别替换。

4. **控制台是 GBK**：脚本里**不要用 emoji**，会 `UnicodeEncodeError` 崩掉。

5. **相机自检**（需要接相机 + 管理员权限，会实际开流取 10 帧看稠密度）：

   ```powershell
   .\.venv\Scripts\python.exe orbbec_v1.py
   ```

   稠密度 >30% 的帧数 ≥5 算正常。**注意相机要对着 0.4m 外的实物**，
   对着天花板/远处会是 0%（超出量程，不是故障）。

---

## 八、模型信息

| 项 | 值 |
|---|---|
| 文件 | `weights/best.pt` |
| 训练来源 | `runs/detect/train-22` |
| mAP50 | 0.728 |
| 类别 | `strawberry`(0), `good`(1), `raw`(2), `medium`(3), `overripe`(4) |
| 说明 | 0 是定位框，1~4 是成熟度分类 |

> 注意：`overripe` 类的精确率只有 **0.033**（训练样本仅 189 个 vs `good` 1172 个），
> 用它做分拣前务必加置信度兜底（主项目的 `strawberry_sort.py` 里用
> `--min-rconf 0.4` 把低置信度的判为"未定级"）。
