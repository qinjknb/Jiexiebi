"""dofbot_hw_bridge —— DOFBOT Pro 机械臂 MoveIt 实机桥接。

模块划分：
    protocol      串口二进制协议（与 STM32 固件 robot_link.c 一一对应）
    calib         关节标定（角度 <-> 舵机位置）
    traj          轨迹采样（线性 / 三次 Hermite）
    serial_link   串口链路 + 假 STM32（离线自测）
    trajectory_bridge  ROS 2 节点：FollowJointTrajectory 动作 + /joint_states
    servo_tool    命令行调试工具
    selftest      无硬件自测
"""

__version__ = "1.0.0"
