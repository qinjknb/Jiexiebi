# -*- coding: utf-8 -*-
"""
Orbbec DaBai DCW2 深度相机 Python 封装（基于 ctypes 调用 Orbbec SDK v1）

背景说明
--------
本相机只支持 Orbbec SDK **v1**（固件 2.0.5.5，内部走 OpenNI 私有协议）：
    - OpenNI2 SDK      -> 不识别
    - Orbbec SDK v2    -> 不识别
    - Orbbec SDK v1    -> OK（本模块走这条路）
由于 v1 没有可用的 Python 预编译绑定、且本机无 C++ 编译器，
这里直接 ctypes 调用 v1 的 C API（OrbbecSDK.dll 导出了 342 个 ob_ 函数）。

⚠️ 重要硬件约束（实测）
----------------------
**最小有效测距约 30cm**。相机对近处物体（<30cm）大面积测不出深度。
本机实测：远处场景稠密度 75%，近距离场景仅 3%。
=> 草莓识别时，相机应距目标 **0.4m 以上**。

用法
----
    from orbbec_v1 import OrbbecDCW2
    with OrbbecDCW2() as cam:
        depth = cam.get_depth()          # numpy uint16, 单位 mm, 0=无效
        valid = depth[depth > 0]
        print(valid.min(), valid.max())
"""
import ctypes
import os
import struct

import numpy as np

# ---------------------------------------------------------------- 常量

SDK_DIR_DEFAULT = os.environ.get(
    "ORBBEC_SDK_DIR",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "sdk"))

# OBStreamType
STREAM_VIDEO = 0
STREAM_IR = 1
STREAM_COLOR = 2
STREAM_DEPTH = 3
STREAM_ACCEL = 4
STREAM_GYRO = 5

# 设备属性 ID（实测确认可用）
PROP_LASER = 42          # bool, 激光/深度开关
PROP_DEPTH_PARAM = 63    # int
PROP_MAX_DEPTH = 23      # int, 实测 10000 (mm)

# 无效深度值
INVALID_DEPTH = 0


class OrbbecError(RuntimeError):
    pass


class OrbbecDCW2:
    """Orbbec DaBai DCW2 深度相机（SDK v1 / ctypes）。

    典型用法：
        cam = OrbbecDCW2()
        cam.start_depth()
        depth = cam.get_depth()
        cam.stop()
        cam.close()

    或使用上下文管理器：
        with OrbbecDCW2() as cam:
            depth = cam.get_depth()
    """

    def __init__(self, sdk_dir=SDK_DIR_DEFAULT, stream_type=STREAM_DEPTH,
                 profile_index=0, verbose=False):
        self.sdk_dir = sdk_dir
        self.stream_type = stream_type
        self.profile_index = profile_index
        self.verbose = verbose

        self._ctx = None
        self._pipe = None
        self._cfg = None
        self._dev = None
        self._started = False
        self._width = None
        self._height = None
        # 帧归还计数（确认没有泄漏）
        self.frames_freed = 0
        self.framesets_freed = 0

        self._load_library()
        self._init_device()

    # ------------------------------------------------------------ 底层
    def _log(self, *a):
        if self.verbose:
            print("[OrbbecDCW2]", *a)

    def _load_library(self):
        if not os.path.isdir(self.sdk_dir):
            raise OrbbecError("SDK 目录不存在: %s" % self.sdk_dir)
        dll = os.path.join(self.sdk_dir, "OrbbecSDK.dll")
        if not os.path.isfile(dll):
            raise OrbbecError("找不到 OrbbecSDK.dll: %s" % dll)

        # SDK 需要在该目录下找 OrbbecSDKConfig_v1.0.xml 与 ob_usb.dll
        os.add_dll_directory(self.sdk_dir)
        cwd = os.getcwd()
        os.chdir(self.sdk_dir)
        try:
            self.lib = ctypes.WinDLL(dll)
        finally:
            os.chdir(cwd)
        self._decl()
        self._log("OrbbecSDK.dll 已加载 (v1)")

    def _decl(self):
        """声明所有用到的函数签名（均已实测验证）。"""
        lib = self.lib
        P, I, U = ctypes.c_void_p, ctypes.c_int, ctypes.c_uint32
        B = ctypes.c_bool

        def sig(name, restype, argtypes):
            f = getattr(lib, name, None)
            if f is None:
                raise OrbbecError("SDK 缺少函数: %s" % name)
            f.restype = restype
            f.argtypes = argtypes
            return f

        # context / device
        sig("ob_create_context", P, [ctypes.POINTER(P)])
        sig("ob_query_device_list", P, [P, ctypes.POINTER(P)])
        sig("ob_device_list_device_count", I, [P, ctypes.POINTER(P)])
        sig("ob_device_list_get_device_name", ctypes.c_char_p,
            [P, I, ctypes.POINTER(P)])
        sig("ob_device_list_get_device_serial_number", ctypes.c_char_p,
            [P, I, ctypes.POINTER(P)])

        # pipeline / config
        sig("ob_create_pipeline", P, [P, ctypes.POINTER(P)])
        sig("ob_create_config", P, [ctypes.POINTER(P)])
        sig("ob_pipeline_get_device", P, [P, ctypes.POINTER(P)])
        sig("ob_pipeline_get_stream_profile_list", P,
            [P, I, ctypes.POINTER(P)])
        sig("ob_stream_profile_list_count", I, [P, ctypes.POINTER(P)])
        sig("ob_stream_profile_list_get_profile", P,
            [P, I, ctypes.POINTER(P)])
        sig("ob_stream_profile_width", I, [P, ctypes.POINTER(P)])
        sig("ob_stream_profile_height", I, [P, ctypes.POINTER(P)])
        sig("ob_stream_profile_fps", I, [P, ctypes.POINTER(P)])
        sig("ob_stream_profile_format", I, [P, ctypes.POINTER(P)])
        sig("ob_config_enable_stream", None, [P, P, ctypes.POINTER(P)])
        sig("ob_pipeline_start_with_config", None, [P, P, ctypes.POINTER(P)])
        sig("ob_pipeline_stop", None, [P, ctypes.POINTER(P)])

        # frames
        sig("ob_pipeline_wait_for_frameset", P, [P, U, ctypes.POINTER(P)])
        sig("ob_frameset_depth_frame", P, [P, ctypes.POINTER(P)])
        sig("ob_frameset_ir_frame", P, [P, ctypes.POINTER(P)])
        sig("ob_frameset_color_frame", P, [P, ctypes.POINTER(P)])
        sig("ob_frame_width", I, [P, ctypes.POINTER(P)])
        sig("ob_frame_height", I, [P, ctypes.POINTER(P)])
        sig("ob_frame_format", I, [P, ctypes.POINTER(P)])
        sig("ob_frame_data_size", U, [P, ctypes.POINTER(P)])
        sig("ob_frame_data", P, [P, ctypes.POINTER(P)])

        # properties
        sig("ob_device_set_bool_property", None, [P, I, B, ctypes.POINTER(P)])
        sig("ob_device_set_int_property", None, [P, I, I, ctypes.POINTER(P)])
        sig("ob_device_get_bool_property", B, [P, I, ctypes.POINTER(P)])
        sig("ob_device_get_int_property", I, [P, I, ctypes.POINTER(P)])

        # 帧释放：必须配对调用，否则 SDK 的 2048MB 帧缓冲池
        # 约 1460 帧（15fps 下 97 秒）后被占满，报
        # "Alloc frame buffer failed! Out of max size!"
        sig("ob_delete_frame", None, [P])
        sig("ob_delete_frameset", None, [P])

    # ------------------------------------------------------------ 帧释放
    def _release_frame(self, fr):
        """释放单帧（必须配对 ob_frameset_*_frame）"""
        if fr:
            self.lib.ob_delete_frame(fr)
            self.frames_freed += 1

    def _release_frameset(self, fs):
        """释放 frameset（必须配对 ob_pipeline_wait_for_frameset）"""
        if fs:
            self.lib.ob_delete_frameset(fs)
            self.framesets_freed += 1

    def release_stats(self):
        """已归还的帧数。跑一阵后应与取帧次数吻合，不涨就是还有泄漏。"""
        return {"frames": self.frames_freed,
                "framesets": self.framesets_freed}

    # ------------------------------------------------------------ 初始化
    def _init_device(self):
        P = ctypes.c_void_p
        self._ctx = P(self.lib.ob_create_context(ctypes.byref(P())))
        if not self._ctx:
            raise OrbbecError("创建 context 失败")

        dl = P(self.lib.ob_query_device_list(self._ctx, ctypes.byref(P())))
        n = self.lib.ob_device_list_device_count(dl, ctypes.byref(P()))
        if n < 1:
            raise OrbbecError("未检测到相机。请确认已插好且驱动 obdrv4 正常。")
        name = self.lib.ob_device_list_get_device_name(dl, 0,
                                                       ctypes.byref(P()))
        sn = self.lib.ob_device_list_get_device_serial_number(
            dl, 0, ctypes.byref(P()))
        self.device_name = name.decode() if name else "?"
        self.serial = sn.decode() if sn else "?"
        self._log("找到设备: %s (SN=%s)" % (self.device_name, self.serial))

        self._pipe = P(self.lib.ob_create_pipeline(self._ctx,
                                                  ctypes.byref(P())))
        self._cfg = P(self.lib.ob_create_config(ctypes.byref(P())))
        self._dev = P(self.lib.ob_pipeline_get_device(self._pipe,
                                                      ctypes.byref(P())))

    # ------------------------------------------------------------ 属性
    def set_laser(self, on=True):
        """开关激光发射器（深度数据依赖它）。"""
        self.lib.ob_device_set_bool_property(self._dev, PROP_LASER,
                                             bool(on), ctypes.byref(ctypes.c_void_p()))
        return self.get_laser()

    def get_laser(self):
        return bool(self.lib.ob_device_get_bool_property(
            self._dev, PROP_LASER, ctypes.byref(ctypes.c_void_p())))

    def get_max_depth_mm(self):
        return self.lib.ob_device_get_int_property(
            self._dev, PROP_MAX_DEPTH, ctypes.byref(ctypes.c_void_p()))

    def list_profiles(self, stream_type=None):
        """列出指定流的可用 profile。"""
        st = self.stream_type if stream_type is None else stream_type
        plist = self.lib.ob_pipeline_get_stream_profile_list(
            self._pipe, st, ctypes.byref(ctypes.c_void_p()))
        cnt = self.lib.ob_stream_profile_list_count(
            plist, ctypes.byref(ctypes.c_void_p()))
        out = []
        for i in range(cnt):
            pr = self.lib.ob_stream_profile_list_get_profile(
                plist, i, ctypes.byref(ctypes.c_void_p()))
            e = ctypes.c_void_p()
            out.append({
                "index": i,
                "width": self.lib.ob_stream_profile_width(pr, ctypes.byref(e)),
                "height": self.lib.ob_stream_profile_height(pr, ctypes.byref(e)),
                "fps": self.lib.ob_stream_profile_fps(pr, ctypes.byref(e)),
                "format": self.lib.ob_stream_profile_format(pr, ctypes.byref(e)),
            })
        return out

    # ------------------------------------------------------------ 流控制
    def start(self, stream_type=None, profile_index=None):
        """启动指定流（默认 DEPTH）。"""
        if self._started:
            self.stop()
        st = self.stream_type if stream_type is None else stream_type
        idx = self.profile_index if profile_index is None else profile_index

        plist = self.lib.ob_pipeline_get_stream_profile_list(
            self._pipe, st, ctypes.byref(ctypes.c_void_p()))
        prof = self.lib.ob_stream_profile_list_get_profile(
            plist, idx, ctypes.byref(ctypes.c_void_p()))
        if not prof:
            raise OrbbecError("取 profile 失败 (stream=%d, index=%d)" % (st, idx))

        e = ctypes.c_void_p()
        self._width = self.lib.ob_stream_profile_width(prof, ctypes.byref(e))
        self._height = self.lib.ob_stream_profile_height(prof, ctypes.byref(e))
        self._fps = self.lib.ob_stream_profile_fps(prof, ctypes.byref(e))

        self.lib.ob_config_enable_stream(self._cfg, prof, ctypes.byref(e))
        self.lib.ob_pipeline_start_with_config(self._pipe, self._cfg,
                                               ctypes.byref(e))
        self._started = True
        self._log("流已启动: %dx%d @%dfps" % (self._width, self._height,
                                              self._fps))
        return self

    def start_depth(self, profile_index=None):
        return self.start(STREAM_DEPTH, profile_index)

    def start_ir(self, profile_index=None):
        return self.start(STREAM_IR, profile_index)

    def start_color(self, profile_index=None):
        return self.start(STREAM_COLOR, profile_index)

    def enable_streams(self, streams, align_mode=None):
        """一次性启用多路流并启动（**推荐用于彩色+深度同时取流**）。

        streams:    [(stream_type, profile_index), ...]
        align_mode: None=不对齐；0~3=调用 ob_config_set_align_mode 启用 D2C。
                    实测本机（DaBai DCW2）：
                        mode=1 -> 深度变 640x480（与彩色一致），稠密度 89.9%
                        mode=2 -> 深度变 640x480，稠密度 93.2%  <- 推荐
                    对齐后深度图直接位于彩色坐标系，
                    YOLO 检测框的像素坐标可直接用于查深度，无需坐标映射。

        例：
            cam.enable_streams([(STREAM_COLOR, 0), (STREAM_DEPTH, 0)],
                               align_mode=2)

        为什么需要它：分两次调用 start() 会导致 pipeline 被重启，
        后一次会顶掉前一次；而且配置必须一次性提交才会同时生效。
        """
        if self._started:
            self.stop()
        e = ctypes.c_void_p()
        info = []
        for st, idx in streams:
            plist = self.lib.ob_pipeline_get_stream_profile_list(
                self._pipe, st, ctypes.byref(e))
            prof = self.lib.ob_stream_profile_list_get_profile(
                plist, idx, ctypes.byref(e))
            if not prof:
                raise OrbbecError("取 profile 失败 (stream=%d, index=%d)"
                                  % (st, idx))
            self.lib.ob_config_enable_stream(self._cfg, prof, ctypes.byref(e))

        # 在 enable_stream 之后、pipeline start 之前设置对齐模式
        if align_mode is not None:
            f = getattr(self.lib, "ob_config_set_align_mode", None)
            if f is None:
                raise OrbbecError("SDK 不支持 ob_config_set_align_mode")
            f.restype = None
            f.argtypes = [ctypes.c_void_p, ctypes.c_int,
                          ctypes.POINTER(ctypes.c_void_p)]
            f(self._cfg, int(align_mode), ctypes.byref(e))
            self.align_mode = int(align_mode)
        else:
            self.align_mode = None

        for st, idx in streams:
            plist = self.lib.ob_pipeline_get_stream_profile_list(
                self._pipe, st, ctypes.byref(e))
            prof = self.lib.ob_stream_profile_list_get_profile(
                plist, idx, ctypes.byref(e))
            info.append((st,
                         self.lib.ob_stream_profile_width(prof, ctypes.byref(e)),
                         self.lib.ob_stream_profile_height(prof, ctypes.byref(e)),
                         self.lib.ob_stream_profile_fps(prof, ctypes.byref(e))))

        self.lib.ob_pipeline_start_with_config(self._pipe, self._cfg,
                                               ctypes.byref(e))
        self._started = True
        for st, w, h, fps in info:
            self._log("已启用流 type=%d: %dx%d @%dfps" % (st, w, h, fps))
        if align_mode is not None:
            self._log("D2C 对齐已启用 (mode=%d)" % align_mode)
        return self

    def stop(self):
        if self._started and self._pipe:
            try:
                self.lib.ob_pipeline_stop(self._pipe,
                                          ctypes.byref(ctypes.c_void_p()))
            except Exception:
                pass
            self._started = False

    def close(self):
        self.stop()
        self._pipe = None
        self._cfg = None
        self._dev = None
        self._ctx = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # ------------------------------------------------------------ 取帧
    def _get_frame(self, getter_name, timeout_ms):
        """通用取帧，返回 (buf, width, height, format) 或 None。

        返回的 buf 是 bytes 拷贝；原始 ob_frame / frameset 在这里立即释放。
        不释放会让 SDK 帧缓冲池耗尽（跑约 97 秒后停止出图）。
        """
        if not self._started:
            self.start()
        fs = self.lib.ob_pipeline_wait_for_frameset(
            self._pipe, int(timeout_ms), ctypes.byref(ctypes.c_void_p()))
        if not fs:
            return None
        fr = None
        try:
            getter = getattr(self.lib, getter_name)
            fr = getter(fs, ctypes.byref(ctypes.c_void_p()))
            if not fr:
                return None

            e = ctypes.c_void_p()
            w = self.lib.ob_frame_width(fr, ctypes.byref(e))
            h = self.lib.ob_frame_height(fr, ctypes.byref(e))
            fmt = self.lib.ob_frame_format(fr, ctypes.byref(e))
            nbytes = self.lib.ob_frame_data_size(fr, ctypes.byref(e))
            ptr = self.lib.ob_frame_data(fr, ctypes.byref(e))
            if not ptr:
                return None

            buf = ctypes.string_at(ptr, nbytes)
            return buf, w, h, fmt
        finally:
            self._release_frame(fr)
            self._release_frameset(fs)

    def get_frame_pair(self, timeout_ms=2000, want_color=True,
                       want_depth=True, want_ir=False):
        """从**同一个 frameset** 中取出彩色/深度/红外，保证时空对齐。

        同时启用多路流时，应优先用本方法而不是分别调用
        get_color_bgr() + get_depth()，否则两次 wait_for_frameset
        可能拿到不同时刻的帧。

        返回 dict: {"color": ndarray|None, "depth": ndarray|None,
                    "ir": ndarray|None}
        """
        if not self._started:
            self.start()
        out = {"color": None, "depth": None, "ir": None}
        fs = self.lib.ob_pipeline_wait_for_frameset(
            self._pipe, int(timeout_ms), ctypes.byref(ctypes.c_void_p()))
        if not fs:
            return out
        e = ctypes.c_void_p()

        def _frame(getter_name):
            # 取完数据立刻释放 frame：SDK 是引用计数，不释放永远不归还
            getter = getattr(self.lib, getter_name)
            fr = getter(fs, ctypes.byref(e))
            if not fr:
                return None
            try:
                w = self.lib.ob_frame_width(fr, ctypes.byref(e))
                h = self.lib.ob_frame_height(fr, ctypes.byref(e))
                nb = self.lib.ob_frame_data_size(fr, ctypes.byref(e))
                ptr = self.lib.ob_frame_data(fr, ctypes.byref(e))
                if not ptr or nb <= 0:
                    return None
                return ctypes.string_at(ptr, nb), w, h
            finally:
                self._release_frame(fr)

        try:
            if want_depth:
                r = _frame("ob_frameset_depth_frame")
                if r:
                    buf, w, h = r
                    need = w * h * 2
                    if len(buf) >= need:
                        out["depth"] = np.frombuffer(
                            buf[:need], dtype="<u2").reshape(h, w).copy()

            if want_ir:
                r = _frame("ob_frameset_ir_frame")
                if r:
                    buf, w, h = r
                    need = w * h * 2
                    if len(buf) >= need:
                        out["ir"] = np.frombuffer(
                            buf[:need], dtype="<u2").reshape(h, w).copy()

            if want_color:
                r = _frame("ob_frameset_color_frame")
                if r:
                    buf, w, h = r
                    out["color"] = self._decode_color(buf, w, h)
        finally:
            self._release_frameset(fs)

        return out

    @staticmethod
    def _decode_color(buf, w, h):
        """把彩色帧原始字节解码为 BGR ndarray（自动识别 YUYV/RGB/MJPG）。"""
        n = w * h
        # YUYV
        if abs(len(buf) - n * 2) <= max(16, n // 100):
            a = np.frombuffer(buf[:n * 2], dtype=np.uint8).reshape(-1, 4)
            y0 = a[:, 0].astype(np.int16)
            u = a[:, 1].astype(np.int16) - 128
            y1 = a[:, 2].astype(np.int16)
            v = a[:, 3].astype(np.int16) - 128
            bgr = np.empty((n, 3), dtype=np.uint8)
            for off, yy in ((0, y0), (1, y1)):
                bgr[off::2, 0] = np.clip(yy + 1.772 * u, 0, 255).astype(np.uint8)
                bgr[off::2, 1] = np.clip(yy - 0.344 * u - 0.714 * v,
                                         0, 255).astype(np.uint8)
                bgr[off::2, 2] = np.clip(yy + 1.402 * v, 0, 255).astype(np.uint8)
            return bgr.reshape(h, w, 3).copy()
        # RGB888
        if abs(len(buf) - n * 3) <= max(16, n // 100):
            rgb = np.frombuffer(buf[:n * 3], dtype=np.uint8).reshape(h, w, 3)
            return rgb[:, :, ::-1].copy()
        # MJPG
        try:
            import cv2
            img = cv2.imdecode(np.frombuffer(buf, dtype=np.uint8),
                               cv2.IMREAD_COLOR)
            if img is not None:
                return img
        except Exception:
            pass
        return None

    def get_depth(self, timeout_ms=2000):
        """取一帧深度图。

        返回: numpy.ndarray, dtype=uint16, shape=(H, W), 单位 mm。
              0 表示该像素无效（无回波 / 超出量程 / 盲区）。
        失败返回 None。
        """
        got = self._get_frame("ob_frameset_depth_frame", timeout_ms)
        if got is None:
            return None
        buf, w, h, _fmt = got
        need = w * h * 2
        if len(buf) < need:
            return None
        arr = np.frombuffer(buf[:need], dtype="<u2").reshape(h, w)
        return arr.copy()

    def get_ir(self, timeout_ms=2000):
        """取一帧红外图（Y10/Y8，稠密度高，可用于辅助定位）。"""
        got = self._get_frame("ob_frameset_ir_frame", timeout_ms)
        if got is None:
            return None
        buf, w, h, _fmt = got
        need = w * h * 2
        if len(buf) < need:
            return None
        return np.frombuffer(buf[:need], dtype="<u2").reshape(h, w).copy()

    def get_color_bgr(self, timeout_ms=2000, raw=False):
        """取一帧彩色图并自动转成 OpenCV 的 BGR 格式（H,W,3 uint8）。

        自动识别以下几种情况（按 data_size 判断）：
            data_size ~= w*h*2  -> YUYV/YUV422（常见默认）
            data_size ~= w*h*3  -> RGB888
            其他                -> MJPG 压缩流，用 cv2.imdecode 解

        raw=True 时不做转换，直接返回 (bytes, w, h, fmt) 供自行处理。
        """
        got = self._get_frame("ob_frameset_color_frame", timeout_ms)
        if got is None:
            return None
        buf, w, h, fmt = got
        if raw:
            return buf, w, h, fmt

        n = w * h
        # --- YUYV (YUV422 packed, 2 bytes/px) ---
        if abs(len(buf) - n * 2) <= max(16, n // 100):
            y = np.frombuffer(buf[:n * 2], dtype=np.uint8)
            y = y.reshape(-1, 4)
            y0 = y[:, 0].astype(np.int16)
            u = y[:, 1].astype(np.int16) - 128
            y1 = y[:, 2].astype(np.int16)
            v = y[:, 3].astype(np.int16) - 128
            # BT.601 YUV -> RGB
            r0 = np.clip(y0 + 1.402 * v, 0, 255)
            g0 = np.clip(y0 - 0.344 * u - 0.714 * v, 0, 255)
            b0 = np.clip(y0 + 1.772 * u, 0, 255)
            r1 = np.clip(y1 + 1.402 * v, 0, 255)
            g1 = np.clip(y1 - 0.344 * u - 0.714 * v, 0, 255)
            b1 = np.clip(y1 + 1.772 * u, 0, 255)
            bgr = np.empty((n, 3), dtype=np.uint8)
            bgr[0::2, 0] = b0.astype(np.uint8)
            bgr[0::2, 1] = g0.astype(np.uint8)
            bgr[0::2, 2] = r0.astype(np.uint8)
            bgr[1::2, 0] = b1.astype(np.uint8)
            bgr[1::2, 1] = g1.astype(np.uint8)
            bgr[1::2, 2] = r1.astype(np.uint8)
            return bgr.reshape(h, w, 3).copy()

        # --- RGB888 (3 bytes/px) ---
        if len(buf) >= n * 3 and abs(len(buf) - n * 3) <= max(16, n // 100):
            rgb = np.frombuffer(buf[:n * 3], dtype=np.uint8).reshape(h, w, 3)
            return rgb[:, :, ::-1].copy()      # RGB -> BGR

        # --- 其他：当作 MJPG 处理 ---
        try:
            import cv2
            arr = np.frombuffer(buf, dtype=np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if img is not None:
                return img
        except Exception:
            pass

        # --- 兜底：按 2 字节灰度处理，返回灰度复制成 3 通道 ---
        if len(buf) >= n * 2:
            g = np.frombuffer(buf[:n * 2], dtype="<u2").reshape(h, w)
            g8 = (g >> 2).astype(np.uint8)
            return np.stack([g8, g8, g8], axis=2)
        return None

    # ------------------------------------------------------------ 工具
    def depth_stats(self, depth):
        """返回深度图的统计信息 dict。"""
        if depth is None:
            return {"valid": 0, "density": 0.0}
        v = depth[depth > 0]
        if v.size == 0:
            return {"valid": 0, "density": 0.0}
        return {
            "valid": int(v.size),
            "total": int(depth.size),
            "density": 100.0 * v.size / depth.size,
            "min": int(v.min()),
            "max": int(v.max()),
            "median": int(np.median(v)),
            "mean": float(v.mean()),
        }

    def depth_at(self, depth, x, y, window=3):
        """取指定像素附近的有效深度中位数（抗噪）。返回 mm，0 表示无有效值。"""
        if depth is None:
            return 0
        h, w = depth.shape
        x0, x1 = max(0, x - window), min(w, x + window + 1)
        y0, y1 = max(0, y - window), min(h, y + window + 1)
        patch = depth[y0:y1, x0:x1]
        v = patch[patch > 0]
        return int(np.median(v)) if v.size else 0

    def pixel_to_3d(self, depth, x, y, fx=None, fy=None, cx=None, cy=None):
        """把像素 + 深度反投影为相机坐标系下的 3D 点 (X, Y, Z)，单位 mm。

        相机坐标系：X 向右, Y 向下, Z 向前（光轴）。
        未提供内参时用视场角估算（91°x62°，精度有限，仅用于粗定位）。
        """
        z = self.depth_at(depth, x, y)
        if z <= 0:
            return None
        if fx is None or fy is None or cx is None or cy is None:
            import math
            hfov = math.radians(91.0)
            vfov = math.radians(62.0)
            fx = (depth.shape[1] / 2.0) / math.tan(hfov / 2.0)
            fy = (depth.shape[0] / 2.0) / math.tan(vfov / 2.0)
            cx = depth.shape[1] / 2.0
            cy = depth.shape[0] / 2.0
        return ((x - cx) * z / fx, (y - cy) * z / fy, float(z))


# ---------------------------------------------------------------- 自测
if __name__ == "__main__":
    print("=" * 66)
    print("  OrbbecDCW2 自测")
    print("=" * 66)

    with OrbbecDCW2(verbose=True) as cam:
        print("\n可用 DEPTH profile:")
        for p in cam.list_profiles(STREAM_DEPTH):
            print("   [%2d] %sx%s @%sfps fmt=%s" % (p["index"], p["width"],
                                                   p["height"], p["fps"],
                                                   p["format"]))

        print("\n激光状态: %s" % cam.get_laser())
        print("最大深度: %s mm" % cam.get_max_depth_mm())

        print("\n启动深度流，连取 10 帧 ...")
        cam.start_depth()
        import time
        time.sleep(1.5)
        ok = 0
        for i in range(10):
            d = cam.get_depth()
            if d is None:
                print("   帧 %d: 取帧失败" % i)
                continue
            s = cam.depth_stats(d)
            if s["density"] > 30:
                ok += 1
            print("   帧 %d: %s  稠密=%.1f%%  范围=%s..%s  中位=%s"
                  % (i, "%dx%d" % (d.shape[1], d.shape[0]), s["density"],
                     s.get("min"), s.get("max"), s.get("median")))
        print("\n稠密度 >30%% 的帧数: %d/10" % ok)
        print("\n结论: %s" % ("深度流工作正常 [OK]" if ok >= 5
                              else "稠密度偏低，请检查相机是否对准 0.4m 外的有纹理目标 [WARN]"))
