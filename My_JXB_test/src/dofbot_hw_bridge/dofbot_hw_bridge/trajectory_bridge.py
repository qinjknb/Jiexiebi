#!/usr/bin/env python3
"""MoveIt -> 真实机械臂 轨迹桥。

作用
----
把 MoveIt 发给 ``/arm_group_controller/follow_joint_trajectory`` 与
``/grip_group_controller/follow_joint_trajectory`` 的时间参数化轨迹，
按 50Hz 采样、换算成舵机位置，通过 USB-TTL 串口经二进制协议发给 STM32，
由 STM32 用 SYNC WRITE 同步驱动 6 个总线舵机。

因为执行的是 MoveIt 自己规划出来的同一条轨迹（同样的时间戳、同样的限速），
所以实机动作与 RViz 仿真里看到的完全一致。

同时本节点发布 ``/joint_states``，让 RViz / MoveIt 显示真实状态。

无硬件也能跑：``-p hardware:=false`` 时不打开串口，只按轨迹发布关节状态，
可以把 launch + MoveIt + RViz 整条链路先跑通，再接机械臂。
"""

from __future__ import annotations

import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger

from . import protocol as P
from .calib import ARM_JOINTS, DEFAULT_JOINT_ORDER, GRIP_JOINTS, load_calib
from .serial_link import LinkError, open_serial
from .traj import TrajSampler


_DYN = ParameterDescriptor(dynamic_typing=True)


def _to_bool(value, default=False):
    """把 true/false/1/0/yes/no/"true" 等各种写法统一成 bool。

    必须自己做这一步：launch 里 ParameterValue(value_type=bool) 对字符串
    "false" 可能得到 True（bool("false") 是真），会导致 hardware:=false 失效、
    节点跑去打开串口然后退出。
    """
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    s = str(value).strip().lower()
    if s in ("1", "true", "yes", "on", "enable", "enabled"):
        return True
    if s in ("0", "false", "no", "off", "disable", "disabled", ""):
        return False
    return default


class TrajectoryBridge(Node):
    """动作服务器 + 串口下发 + /joint_states 发布。"""

    def __init__(self):
        super().__init__("dofbot_hw_bridge")

        # ---------------- 参数 ----------------
        self.declare_parameter("calib_file", "")
        self.declare_parameter("port", "")
        self.declare_parameter("baud", 0)
        self.declare_parameter("hardware", "true", _DYN)
        self.declare_parameter("auto_home", "false", _DYN)
        self.declare_parameter("home_time", 2.0)
        self.declare_parameter("command_rate", 0.0)
        self.declare_parameter("state_rate", 0.0)
        self.declare_parameter("max_velocity", -1.0)
        self.declare_parameter("interpolation", "auto")   # auto|linear|cubic
        self.declare_parameter("telemetry_source", "auto")  # auto|meas|cmd
        self.declare_parameter("arm_action", "/arm_group_controller/follow_joint_trajectory")
        self.declare_parameter("grip_action", "/grip_group_controller/follow_joint_trajectory")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("startup_timeout", 2.0)

        gp = self.get_parameter
        self.hardware = _to_bool(gp("hardware").value, True)
        self.auto_home = _to_bool(gp("auto_home").value, False)
        self.home_time = float(gp("home_time").value)
        self.interp = str(gp("interpolation").value).lower()
        self.telem_source = str(gp("telemetry_source").value).lower()
        self.startup_timeout = float(gp("startup_timeout").value)

        # ---------------- 标定 ----------------
        calib_file = str(gp("calib_file").value)
        if not calib_file:
            from ament_index_python.packages import get_package_share_directory
            import os
            calib_file = os.path.join(get_package_share_directory("dofbot_hw_bridge"),
                                      "config", "arm_calib.yaml")
        self.calib = load_calib(calib_file)
        self.calib_file = calib_file
        self.get_logger().info("标定文件: %s" % calib_file)
        self.get_logger().info(str(self.calib))

        if gp("port").value:
            self.calib.port = str(gp("port").value)
        if int(gp("baud").value) > 0:
            self.calib.baud = int(gp("baud").value)
        cmd_rate = float(gp("command_rate").value)
        if cmd_rate > 0.0:
            self.calib.command_rate = cmd_rate
        state_rate = float(gp("state_rate").value)
        if state_rate > 0.0:
            self.calib.state_rate = state_rate
        mv = float(gp("max_velocity").value)
        if mv >= 0.0:
            self.calib.max_velocity = mv

        # ---------------- 状态 ----------------
        self._lock = threading.RLock()
        self._angles = {n: 0.0 for n in DEFAULT_JOINT_ORDER}   # 当前指令角（弧度）
        self._ticks = list(self.calib.home_ticks())            # 当前指令位置
        self._vel = {n: 0.0 for n in DEFAULT_JOINT_ORDER}
        self._last_pub = time.monotonic()
        self._estop = False
        self._active_goals = 0
        self._link = None
        self._fw = None
        self._start_time = time.monotonic()
        self._diag_hint_done = False
        self._arm_action_name = str(gp("arm_action").value)
        self._last_tx = time.monotonic()
        self._tx_count = 0

        # ---------------- 硬件 ----------------
        if self.hardware:
            self._open_hardware()
        else:
            self.get_logger().warning(
                "hardware:=false —— 虚拟模式，只发 /joint_states，不驱动机械臂")

        # ---------------- 发布 / 服务 / 动作 ----------------
        self.cb_group = ReentrantCallbackGroup()

        self.joint_pub = self.create_publisher(
            JointState, str(gp("joint_states_topic").value), 10)

        self.create_service(Trigger, "~/home", self._srv_home, callback_group=self.cb_group)
        self.create_service(Trigger, "~/estop", self._srv_estop, callback_group=self.cb_group)
        self.create_service(SetBool, "~/enable", self._srv_enable, callback_group=self.cb_group)
        self.create_service(Trigger, "~/status", self._srv_status, callback_group=self.cb_group)

        self._srvs = []
        for joints, name, label in ((ARM_JOINTS, str(gp("arm_action").value), "arm"),
                                    (GRIP_JOINTS, str(gp("grip_action").value), "grip")):
            srv = ActionServer(
                self,
                FollowJointTrajectory,
                name,
                execute_callback=self._make_execute(joints, label),
                goal_callback=self._on_goal,
                cancel_callback=lambda goal_handle: CancelResponse.ACCEPT,
                callback_group=self.cb_group,
            )
            self._srvs.append(srv)
            self.get_logger().info("动作服务器: %s" % name)

        self.state_timer = self.create_timer(
            1.0 / max(1.0, self.calib.state_rate), self._publish_state,
            callback_group=self.cb_group)

        self.get_logger().info(
            "就绪：指令 %0.0fHz，状态 %0.0fHz，最大速度 %0.0f ticks/s"
            % (self.calib.command_rate, self.calib.state_rate, self.calib.max_velocity))

    # ------------------------------------------------------------------
    # 硬件初始化
    # ------------------------------------------------------------------
    def _open_hardware(self):
        port = self.calib.port
        self.get_logger().info("打开串口 %s @ %d ..." % (port, self.calib.baud))
        try:
            self._link = open_serial(port, self.calib.baud, logger=self.get_logger())
        except Exception as exc:
            raise RuntimeError(
                "串口打开失败: %s\n"
                "  1) 确认 USB-TTL 已插上：ls /dev/ttyUSB*\n"
                "  2) 当前用户在 dialout 组：sudo usermod -aG dialout $USER（重新登录生效）\n"
                "  3) 用 -p port:=/dev/ttyUSB0 指定正确端口\n"
                "  4) 想先不接硬件：加 -p hardware:=false" % exc)

        deadline = time.monotonic() + max(1.0, self.startup_timeout) * 5
        fw = None
        while time.monotonic() < deadline:
            try:
                fw = self._link.ping(0.4)
                break
            except LinkError:
                time.sleep(0.3)
        if fw is None:
            raise RuntimeError(
                "STM32 无应答（PING 超时）。检查：\n"
                "  * STM32 是否已烧录本仓库的固件（build_fw.ps1 -Flash）\n"
                "  * USB-TTL 的 TX->PA3, RX->PA2, GND->GND\n"
                "  * 波特率是否与 ARM_LINK_BAUD 一致（默认 115200）")
        self._fw = fw
        self.get_logger().info(
            "STM32 在线：固件 v%d.%d，关节数 %d，总线方式 %d"
            % (fw["ver_major"], fw["ver_minor"], fw["njoint"], fw["busmode"]))

        # 软限位 + 遥测频率
        tmin, tmax = self.calib.limits()
        try:
            self._link.set_limits(tmin, tmax, 1.0)
            self._link.set_telemetry(max(1, self.calib.telem_rate), 1.0)
        except LinkError as exc:
            self.get_logger().warning("设置限位/遥测失败：%s" % exc)

        # 读回真实位置作为起点；读不到就用标定中位
        ticks = None
        for _ in range(3):
            try:
                ticks = self._link.read_pos(0.6)
                break
            except LinkError:
                time.sleep(0.1)
        if ticks is None or all(t == P.UNKNOWN_TICK for t in ticks):
            ticks = self.calib.home_ticks()
            self.get_logger().warning(
                "读取舵机位置失败（舵机没上电？），先按标定中位处理: %s" % ticks)
        else:
            ticks = [self.calib.home_ticks()[i] if t == P.UNKNOWN_TICK else t
                     for i, t in enumerate(ticks)]

        with self._lock:
            self._ticks = list(ticks)
            self._angles.update(self.calib.ticks_to_angles(ticks))

        try:
            self._link.set_mode(P.MODE_STREAM, 1.0)
            self._link.send_targets(self._ticks)   # 先锁住当前位置
            self._last_tx = time.monotonic()
        except LinkError as exc:
            self.get_logger().warning("进入跟随时失败：%s" % exc)

        if self.auto_home:
            self.get_logger().info("auto_home:=true，回中位 ...")
            self._do_home(self.home_time)

        self.get_logger().info(
            "初始位置(ticks)=%s  角度(deg)=%s"
            % (self._ticks,
               [round(math.degrees(self._angles[n]), 1) for n in DEFAULT_JOINT_ORDER]))

    # ------------------------------------------------------------------
    # 角度 -> 位置 下发
    # ------------------------------------------------------------------
    def _push_angles(self, angles: dict) -> bool:
        """把关节角合并进指令并下发。返回是否真的发了帧。"""
        with self._lock:
            if self._estop:
                return False
            target = list(self._ticks)
            new_ticks = self.calib.angles_to_ticks(angles)
            for i, name in enumerate(DEFAULT_JOINT_ORDER):
                if name not in angles:
                    continue
                new = new_ticks[i]
                old = target[i]
                # PC 侧安全限速（ticks/s）
                vmax = self.calib.max_velocity
                if vmax > 0.0:
                    dt = max(1e-3, min(0.1, time.monotonic() - self._last_tx))
                    step = vmax * dt
                    if abs(new - old) > step:
                        new = int(old + math.copysign(step, new - old))
                target[i] = max(0, min(P.TICK_MAX, int(new)))
            self._ticks = target
            # 用真正下发下去的 ticks 反算角度，保证 /joint_states 与实机一致
            self._angles.update(self.calib.ticks_to_angles(target))
            ticks = list(self._ticks)

        if self._link is not None:
            self._link.send_targets(ticks)
            self._tx_count += 1
        self._last_tx = time.monotonic()
        return True

    # ------------------------------------------------------------------
    # 动作回调
    # ------------------------------------------------------------------
    def _on_goal(self, goal_request):
        try:
            self.get_logger().info("收到轨迹目标：%d 个路点，关节 %s"
                                   % (len(goal_request.trajectory.points),
                                      ",".join(goal_request.trajectory.joint_names)))
        except Exception:
            pass
        if self._estop:
            self.get_logger().warning("急停状态下拒绝新目标（调用 ~/enable true 恢复）")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _make_execute(self, wanted_joints, label):
        def execute(goal_handle):
            return self._execute(goal_handle, wanted_joints, label)
        return execute

    def _execute(self, goal_handle, wanted_joints, label):
        traj = goal_handle.request.trajectory
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL

        if len(traj.points) == 0:
            self.get_logger().info("[%s] 空轨迹，直接成功" % label)
            goal_handle.succeed()
            return result

        try:
            sampler = TrajSampler(traj.joint_names, traj.points, wanted_joints,
                                  interp=self.interp)
        except ValueError as exc:
            self.get_logger().error("[%s] %s" % (label, exc))
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(exc)
            goal_handle.abort()
            return result

        t_end = sampler.duration
        self._active_goals += 1
        self.get_logger().info(
            "[%s] 开始执行：%d 个路点，时长 %.2fs，插值=%s，关节 %s"
            % (label, sampler.n_points, t_end,
               "cubic" if sampler.use_vel else "linear", ",".join(wanted_joints)))

        # 固件在 400ms 没有新目标后会回到 IDLE（安全保护），
        # 所以每条轨迹开始前要重新进入"跟随"模式。
        if self._link is not None:
            try:
                self._link.set_mode(P.MODE_STREAM, 0.5)
            except LinkError as exc:
                self.get_logger().warning("进入跟随模式失败：%s" % exc)

        period = 1.0 / max(1.0, self.calib.command_rate)
        t_start = time.monotonic()
        next_tick = t_start
        ok = True

        # ---- 诊断：统计每个关节下发位置的方向回折次数 ----
        # 正常一条轨迹应该单调（回折 0~1 次，端点微调算 1 次）。
        # 回折很多 + 实机来回抖 -> 要么轨迹/插值有问题，要么舵机中途复位。
        idx_map = [DEFAULT_JOINT_ORDER.index(j) for j in wanted_joints]
        last_dir = [0] * len(wanted_joints)
        reversals = [0] * len(wanted_joints)
        with self._lock:
            prev_ticks = [self._ticks[i] for i in idx_map]
        first_ticks = list(prev_ticks)

        try:
            while True:
                if self._estop or goal_handle.is_cancel_requested:
                    ok = False
                    break
                now = time.monotonic()
                if now < next_tick:
                    time.sleep(min(period, next_tick - now))
                    continue
                next_tick += period
                t = now - t_start
                self._push_angles(sampler.at(t))

                with self._lock:
                    cur = [self._ticks[i] for i in idx_map]
                for k in range(len(wanted_joints)):
                    dlt = cur[k] - prev_ticks[k]
                    if dlt != 0:
                        sgn = 1 if dlt > 0 else -1
                        if last_dir[k] != 0 and sgn != last_dir[k]:
                            reversals[k] += 1
                        last_dir[k] = sgn
                        prev_ticks[k] = cur[k]
                if t >= t_end:
                    break

            # 用最后一个点收尾
            self._push_angles(sampler.at(t_end))
        except Exception as exc:      # 串口异常等
            self.get_logger().error("[%s] 执行异常：%s" % (label, exc))
            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            result.error_string = str(exc)
            ok = False
        finally:
            self._active_goals -= 1

        if not ok and goal_handle.is_cancel_requested:
            self.get_logger().warning("[%s] 被取消" % label)
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            goal_handle.canceled()
            return result
        if not ok:
            goal_handle.abort()
            return result

        goal_handle.succeed()
        with self._lock:
            final_ticks = [self._ticks[i] for i in idx_map]
        detail = ", ".join("%s:%d->%d(回折%d次)" % (j, a, b, r)
                           for j, a, b, r in zip(wanted_joints, first_ticks,
                                                 final_ticks, reversals))
        self.get_logger().info("[%s] 执行完成：%s" % (label, detail))
        total_rev = sum(reversals)
        if total_rev > 3:
            self.get_logger().warning(
                "[%s] 下发指令出现 %d 次方向回折（正常应为 0~2 次）。\n"
                "  若实机动作过程中来回抖动/开合，按这个顺序排查：\n"
                "   1) 轨迹插值换成线性试：ros2 launch ... interpolation:=linear\n"
                "   2) 把 max_velocity 降到 1000 以下（供电不足会触发舵机保护性复位）\n"
                "   3) 舵机供电：12V 需 ≥5A，接头要拧紧（掉电会让舵机跳回启动位置）"
                % (label, total_rev))
        return result

    # ------------------------------------------------------------------
    # 状态发布
    # ------------------------------------------------------------------
    def _publish_state(self):
        now = time.monotonic()
        dt = max(1e-3, now - self._last_pub)
        self._last_pub = now

        with self._lock:
            angles = dict(self._angles)

        # 有遥测时优先用舵机回读的真实位置
        use_meas = self.telem_source in ("auto", "meas")
        if use_meas and self._link is not None and self._link.latest_state is not None:
            meas = self._link.latest_state["meas"]
            real = self.calib.ticks_to_angles(meas)
            for k, v in real.items():
                angles[k] = v

        # 把发布出去的角度钳进 URDF 限位：舵机回读有 ±几个 tick 噪声，
        # 正好落在限位边界（比如 grip_joint 的 0）上时，MoveIt 会判定
        # "Start state out of bounds" 而拒绝规划。
        names = list(DEFAULT_JOINT_ORDER)
        clamped = []
        for n in names:
            raw = angles[n]
            fixed = self.calib.clamp_rad(n, raw)
            if abs(fixed - raw) > 1e-6:
                clamped.append((n, raw, fixed))
            angles[n] = fixed
        if clamped and (not hasattr(self, "_clamp_warned") or not self._clamp_warned):
            self._clamp_warned = True
            n, raw, fixed = clamped[0]
            self.get_logger().warning(
                "关节 %s 回读 %.4f rad 略超 URDF 限位，发布时已钳到 %.4f rad"
                "（把 arm_calib.yaml 里该关节的 center 往量程内挪 20~30 tick 可彻底解决）"
                % (n, raw, fixed))

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = [float(angles[n]) for n in names]
        # 速度用差分估算并做限幅，避免回读抖动导致下游误判
        vel = []
        for n in names:
            v = (angles[n] - self._vel.get(n, angles[n])) / dt
            vel.append(float(max(-10.0, min(10.0, v))))
        msg.velocity = vel
        self._vel = {n: angles[n] for n in names}
        self.joint_pub.publish(msg)

        # 诊断：启动 8 秒后还没收到任何轨迹，提示一次（只提示一次，不刷屏）
        if (not self._diag_hint_done
                and now - self._start_time > 8.0
                and self._active_goals == 0
                and self._tx_count == 0):
            self._diag_hint_done = True
            self.get_logger().warning(
                "已经在发布 /joint_states，但还没收到任何轨迹目标。\n"
                "  · 在 RViz 里要按 Plan & Execute（只按 Plan 不会动）\n"
                "  · 手动测试：ros2 action send_goal %s "
                "control_msgs/action/FollowJointTrajectory \"{}\""
                % self._arm_action_name)

    # ------------------------------------------------------------------
    # 服务
    # ------------------------------------------------------------------
    def _do_home(self, seconds):
        if self._link is None:
            with self._lock:
                self._angles = self.calib.ticks_to_angles(self.calib.home_ticks())
                self._ticks = self.calib.home_ticks()
            return True
        try:
            self._link.home(int(max(0.2, seconds) * 1000), 1.0)
            self._ticks = self.calib.home_ticks()
            self._angles.update(self.calib.ticks_to_angles(self._ticks))
            time.sleep(min(2.0, max(0.2, seconds)))
            self._link.set_mode(P.MODE_STREAM, 1.0)
            return True
        except LinkError as exc:
            self.get_logger().error("回中位失败：%s" % exc)
            return False

    def _srv_home(self, request, response):
        response.success = self._do_home(self.home_time)
        response.message = "已回中位" if response.success else "回中位失败"
        self.get_logger().info(response.message)
        return response

    def _srv_estop(self, request, response):
        self._estop = True
        if self._link is not None:
            try:
                self._link.stop_motion(0.5)
            except LinkError as exc:
                self.get_logger().warning("急停指令未确认：%s" % exc)
        response.success = True
        response.message = "急停：已停止下发，目标全部拒绝；~/enable true 恢复"
        self.get_logger().error(response.message)
        return response

    def _srv_enable(self, request, response):
        self._estop = not bool(request.data)
        if request.data:
            if self._link is not None:
                try:
                    self._link.set_mode(P.MODE_STREAM, 0.5)
                    self._link.send_targets(self._ticks)
                except LinkError as exc:
                    response.success = False
                    response.message = "恢复失败：%s" % exc
                    return response
            response.message = "已恢复，可以接受新目标"
        else:
            self._srv_estop(None, Trigger.Response())
            response.message = "已停止（软急停）"
        response.success = True
        self.get_logger().info(response.message)
        return response

    def _srv_status(self, request, response):
        st = self._link.latest_state if self._link is not None else None
        if st is None:
            response.success = False
            response.message = "无遥测（hardware:=%s）" % self.hardware
            return response
        response.success = True
        response.message = (
            "mode=%s flags=%s cmd=%s meas=%s"
            % (P.MODE_TEXT.get(st["mode"], st["mode"]), P.flags_text(st["flags"]),
               st["cmd"], st["meas"]))
        return response

    # ------------------------------------------------------------------
    def destroy_node(self):
        try:
            if self._link is not None:
                try:
                    self._link.set_mode(P.MODE_IDLE, 0.3)
                except Exception:
                    pass
                self._link.shutdown()
        finally:
            super().destroy_node()


try:                                     # rclpy Humble+ 才有
    from rclpy.executors import ExternalShutdownException
except ImportError:                      # pragma: no cover
    class ExternalShutdownException(Exception):
        pass


def main(argv=None):
    rclpy.init(args=argv)
    node = None
    try:
        node = TrajectoryBridge()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as exc:
        print("\n[启动失败] %s\n" % exc)
        raise SystemExit(1)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
