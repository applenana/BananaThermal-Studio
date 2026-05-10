"""
thermal_dual_app.py
====================

热成像 + 可见光 双光投屏上位机主程序.

特性
----

* **激活流程**: 与老 ``激活上位机.py`` 兼容. 启动后自动搜索串口, 调用
  ``GetSysInfo`` 取设备信息, 已激活 → 直接进入推流; 未激活 → 弹出激活码界面.
* **双光并排**: 左侧为热成像 + 温度数据 + 温度曲线, 右侧为可见光画面.
* **可见光默认关闭**: 用户勾选 "启用可见光投屏" 后, 上位机才开始向设备发
  ``vstream`` 命令. 关闭则停止心跳, 设备 1 秒后自动停推.
* **统一字节流解析**: 串口接收线程把所有字节交给 ``FrameParser``,
  由其分发热帧 / 可见光帧到对应回调.
* **滤波**: 卡尔曼 + (可选) 双边滤波, 与老程序一致.

依赖
----

``pip install -r requirements.txt``

入口
----

``python thermal_dual_app.py``
"""

from __future__ import annotations

import json
import os
import queue
import struct
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

import numpy as np
import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

try:
    import cv2
    HAS_CV2 = True
except ImportError:  # pragma: no cover
    HAS_CV2 = False

from frame_parser import FrameParser
from visible_view import VisibleView


# ============================================================================
# 字体加载 (matplotlib 中文支持). 同目录下 font/HarmonyOS_Sans_SC_Regular.ttf 可选
# ============================================================================
_FONT_PATH = os.path.join(os.path.dirname(__file__), "font", "HarmonyOS_Sans_SC_Regular.ttf")
if os.path.exists(_FONT_PATH):
    _font_prop = fm.FontProperties(fname=_FONT_PATH)
    plt.rcParams["font.sans-serif"] = [_font_prop.get_name(), "SimHei", "Microsoft YaHei", "DejaVu Sans"]
else:
    _font_prop = None
    plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False


# ============================================================================
# 卡尔曼滤波 (与老程序一致)
# ============================================================================
class KalmanFilter:
    """一维标量卡尔曼滤波器, 用于温度统计值平滑."""

    def __init__(self, process_variance: float = 1e-4, measurement_variance: float = 1e-2):
        self.q = process_variance
        self.r = measurement_variance
        self.x = 0.0
        self.p = 1.0
        self._init = False

    def update(self, z: float) -> float:
        if not self._init:
            self.x = z
            self._init = True
            return z
        # 预测
        p_pred = self.p + self.q
        # 更新
        k = p_pred / (p_pred + self.r)
        self.x = self.x + k * (z - self.x)
        self.p = (1 - k) * p_pred
        return self.x


class ThermalKalmanFilter:
    """24x32 像素逐点卡尔曼滤波器, 抑制随机噪声."""

    def __init__(self, h: int = 24, w: int = 32, process_variance: float = 1e-4,
                 measurement_variance: float = 1e-2):
        self.h, self.w = h, w
        self._x = None  # 后验估计 (h, w)
        self._p = np.ones((h, w), dtype=np.float32)
        self.q = process_variance
        self.r = measurement_variance

    def filter(self, frame: np.ndarray) -> np.ndarray:
        if self._x is None:
            self._x = frame.astype(np.float32).copy()
            return self._x.copy()
        p_pred = self._p + self.q
        k = p_pred / (p_pred + self.r)
        self._x = self._x + k * (frame - self._x)
        self._p = (1.0 - k) * p_pred
        return self._x.copy()

    def reset(self):
        self._x = None
        self._p = np.ones((self.h, self.w), dtype=np.float32)


# ============================================================================
# 主应用
# ============================================================================
class ThermalDualApp:
    """热成像 + 可见光双光上位机主类."""

    # ------------------------------ 配置 ------------------------------
    BAUDRATE = 115200
    THERMAL_HB_INTERVAL = 0.5     # `stream` 心跳, 与老程一致
    VISIBLE_HB_INTERVAL = 0.5     # `vstream` 心跳, 设备端 1s 超时, 必须 <1s
    MAX_HISTORY_POINTS = 100      # 温度曲线最大保留点数

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("热成像 + 可见光 双光上位机")

        # ---- 全局字体加大: 默认 12 号 (Tk 原默认9 号过小) ----
        self._setup_fonts()

        # 窗口尺寸随字号缩放: ratio = size / 9 (Tk 默认字号9)
        try:
            from tkinter import font as tkfont
            _ratio = tkfont.nametofont("TkDefaultFont").cget("size") / 9.0
        except Exception:
            _ratio = 1.0
        self._win_ratio = _ratio
        w = int(640 * _ratio); h = int(360 * _ratio)
        self.root.geometry(f"{w}x{h}")
        self.root.minsize(w, h)
        self._after_id_queue: str | None = None  # 保存 after 句柄以便退出时取消

        # ---- 串口与设备 ----
        self.serial_port: serial.Serial | None = None
        self.serial_lock = threading.Lock()       # 保护对 serial_port 的写入
        self.is_connected = False
        self.is_activated = False
        self.device_info: dict | None = None
        self.device_serial = "未获取"
        self.manual_port: str | None = None       # 用户手选 COM 口
        self.available_ports: list[tuple[str, str]] = []

        # ---- 后台线程 ----
        self.monitor_thread: threading.Thread | None = None
        self.reader_thread: threading.Thread | None = None
        self.thermal_hb_thread: threading.Thread | None = None
        self.visible_hb_thread: threading.Thread | None = None
        self.stop_event = threading.Event()       # 程序退出时置位
        self.thermal_hb_running = False
        self.visible_hb_running = False

        # ---- 帧解析 ----
        self.parser = FrameParser(
            on_thermal=self._on_thermal_frame_from_thread,
            on_visible=self._on_visible_frame_from_thread,
        )

        # ---- 数据缓存 ----
        self.thermal_data = np.zeros((24, 32), dtype=np.float32)
        self.thermal_kalman = ThermalKalmanFilter()
        self.kf_max = KalmanFilter()
        self.kf_min = KalmanFilter()
        self.kf_avg = KalmanFilter()
        self.history_max: list[float] = []
        self.history_min: list[float] = []
        self.history_avg: list[float] = []

        # ---- 主线程消息队列 (后台 → UI) ----
        self.msg_queue: queue.Queue = queue.Queue()

        # ---- UI ----
        self._build_ui()
        self._refresh_ports()

        # 启动监控线程
        self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()

        # 启动队列轮询
        self._after_id_queue = self.root.after(100, self._process_queue)

    # =================================================================
    # UI 构建
    # =================================================================

    def _setup_fonts(self):
        """通过环境变量 THERMAL_DUAL_FONT_SIZE 控制全局字号 (默认 14).

        说明: tk scaling 在 Win DPI-aware 下对 TkDefaultFont 影响有限,
        直接调 size 才是稳定可见的放大方式.
        """
        from tkinter import font as tkfont
        try:
            size = int(os.environ.get("THERMAL_DUAL_FONT_SIZE", "12"))
        except ValueError:
            size = 12
        for name in (
            "TkDefaultFont", "TkTextFont", "TkHeadingFont", "TkMenuFont",
            "TkTooltipFont", "TkCaptionFont", "TkSmallCaptionFont", "TkIconFont",
        ):
            try:
                f = tkfont.nametofont(name)
                f.configure(family="Microsoft YaHei UI", size=size)
            except tk.TclError:
                pass
        try:
            tkfont.nametofont("TkFixedFont").configure(family="Consolas", size=size - 1)
        except tk.TclError:
            pass
        style = ttk.Style()
        style.configure(".", font=("Microsoft YaHei UI", size))
        style.configure("TLabelframe.Label", font=("Microsoft YaHei UI", size, "bold"))
        style.configure("TButton", padding=(int(size * 0.7), int(size * 0.35)))

    def _build_ui(self):
        """构建顶部控制栏 + 主内容区域. 主内容默认隐藏, 激活后显示."""
        # ---------- 顶部控制栏 ----------
        ctrl = ttk.Frame(self.root, padding=10)
        ctrl.grid(row=0, column=0, sticky="we")

        # 串口选择
        port_box = ttk.LabelFrame(ctrl, text="设备连接", padding=8)
        port_box.grid(row=0, column=0, sticky="we", pady=(0, 8))

        ttk.Label(port_box, text="串口:").grid(row=0, column=0, padx=(0, 4))
        self.port_var = tk.StringVar(value="自动搜索")
        self.port_combo = ttk.Combobox(port_box, textvariable=self.port_var,
                                       state="readonly", width=28)
        self.port_combo.grid(row=0, column=1, sticky="we", padx=(0, 8))
        self.port_combo.bind("<<ComboboxSelected>>", self._on_port_selected)
        ttk.Button(port_box, text="刷新", width=6,
                   command=self._refresh_ports).grid(row=0, column=2)

        # 投屏 / 滤波选项 (激活后才显示)
        self.options_frame = ttk.Frame(port_box)
        self.options_frame.grid(row=1, column=0, columnspan=3, sticky="we", pady=(8, 0))
        self.options_frame.grid_remove()

        self.var_filter = tk.BooleanVar(value=True)
        self.var_bilateral = tk.BooleanVar(value=HAS_CV2)
        self.var_thermal_on = tk.BooleanVar(value=True)
        self.var_visible_on = tk.BooleanVar(value=True)   # 默认开启可见光投屏

        ttk.Checkbutton(self.options_frame, text="卡尔曼滤波",
                        variable=self.var_filter).grid(row=0, column=0, padx=(0, 12))
        ttk.Checkbutton(self.options_frame, text="双边滤波",
                        variable=self.var_bilateral,
                        state=("normal" if HAS_CV2 else "disabled")).grid(row=0, column=1, padx=(0, 12))
        ttk.Checkbutton(self.options_frame, text="热成像投屏",
                        variable=self.var_thermal_on,
                        command=self._toggle_thermal).grid(row=0, column=2, padx=(0, 12))
        ttk.Checkbutton(self.options_frame, text="可见光投屏",
                        variable=self.var_visible_on,
                        command=self._toggle_visible).grid(row=0, column=3)

        port_box.columnconfigure(1, weight=1)

        # 状态条
        self.status_label = ttk.Label(ctrl, text="状态: 正在搜索设备...",
                                      foreground="orange")
        self.status_label.grid(row=1, column=0, sticky="we")

        # 设备信息 (激活后显示)
        self.dev_info_frame = ttk.Frame(ctrl)
        self.dev_info_frame.grid(row=2, column=0, sticky="we", pady=(4, 0))
        self.dev_info_frame.grid_remove()
        self._build_device_info(self.dev_info_frame)

        # 激活面板 (未激活才显示)
        self.activation_frame: ttk.LabelFrame | None = None
        self.activation_key_var: tk.StringVar | None = None

        ctrl.columnconfigure(0, weight=1)

        # ---------- 主内容: 左 (热) + 右 (可见光) ----------
        self.main_frame = ttk.Frame(self.root, padding=8)
        self.main_frame.grid(row=1, column=0, sticky="nsew")
        self.main_frame.grid_remove()

        left = ttk.Frame(self.main_frame)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        right = ttk.Frame(self.main_frame)
        right.grid(row=0, column=1, sticky="nsew")

        self._build_temperature_panel(left)
        self._build_thermal_canvas(left)
        self._build_chart_canvas(left)
        self._build_visible_panel(right)
        self._build_debug_panel(right)

        self.main_frame.columnconfigure(0, weight=1)
        self.main_frame.columnconfigure(1, weight=1)
        self.main_frame.rowconfigure(0, weight=1)
        left.rowconfigure(2, weight=1)
        right.rowconfigure(0, weight=2)
        right.rowconfigure(1, weight=1)
        right.columnconfigure(0, weight=1)

        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)

    # ----------- 设备信息条 -----------
    def _build_device_info(self, parent):
        ttk.Label(parent, text="序列号:").grid(row=0, column=0, padx=(0, 4))
        self.lbl_serial = ttk.Label(parent, text="--", foreground="gray")
        self.lbl_serial.grid(row=0, column=1, padx=(0, 12))

        ttk.Label(parent, text="版本:").grid(row=0, column=2, padx=(0, 4))
        self.lbl_version = ttk.Label(parent, text="--", foreground="gray")
        self.lbl_version.grid(row=0, column=3, padx=(0, 12))

        ttk.Label(parent, text="激活:").grid(row=0, column=4, padx=(0, 4))
        self.lbl_activation = ttk.Label(parent, text="未激活", foreground="red")
        self.lbl_activation.grid(row=0, column=5)

    # ----------- 温度数据面板 -----------
    def _build_temperature_panel(self, parent):
        f = ttk.LabelFrame(parent, text="实时温度", padding=8)
        f.grid(row=0, column=0, sticky="we", pady=(0, 6))

        ttk.Label(f, text="最高:").grid(row=0, column=0, sticky="w")
        # 温度数值字号 = 默认字号 + 6 (干多点, 突出关键读数)
        try:
            from tkinter import font as tkfont
            base_size = tkfont.nametofont("TkDefaultFont").cget("size")
        except Exception:
            base_size = 14
        big_size = base_size + 6
        self.lbl_tmax = ttk.Label(f, text="--°C", font=("Microsoft YaHei UI", big_size, "bold"), foreground="red")
        self.lbl_tmax.grid(row=0, column=1, padx=(0, 16))

        ttk.Label(f, text="最低:").grid(row=0, column=2, sticky="w")
        self.lbl_tmin = ttk.Label(f, text="--°C", font=("Microsoft YaHei UI", big_size, "bold"), foreground="blue")
        self.lbl_tmin.grid(row=0, column=3, padx=(0, 16))

        ttk.Label(f, text="平均:").grid(row=0, column=4, sticky="w")
        self.lbl_tavg = ttk.Label(f, text="--°C", font=("Microsoft YaHei UI", big_size, "bold"), foreground="green")
        self.lbl_tavg.grid(row=0, column=5)

    # ----------- 热成像画布 -----------
    def _build_thermal_canvas(self, parent):
        f = ttk.LabelFrame(parent, text="热成像画面", padding=4)
        f.grid(row=1, column=0, sticky="we", pady=(0, 6))

        self.thermal_fig = plt.figure(figsize=(4, 3))
        self.thermal_ax = self.thermal_fig.add_subplot(111)
        self.thermal_ax.axis("off")
        self.thermal_im = self.thermal_ax.imshow(self.thermal_data, cmap="coolwarm",
                                                 interpolation="bilinear", aspect="equal")
        self.thermal_fig.subplots_adjust(0, 0, 1, 1)
        self.thermal_canvas = FigureCanvasTkAgg(self.thermal_fig, f)
        self.thermal_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    # ----------- 温度曲线 -----------
    def _build_chart_canvas(self, parent):
        f = ttk.LabelFrame(parent, text="温度变化趋势", padding=4)
        f.grid(row=2, column=0, sticky="nsew")

        self.chart_fig = plt.figure(figsize=(5, 2.4))
        self.chart_ax = self.chart_fig.add_subplot(111)
        self.chart_ax.set_xlim(0, self.MAX_HISTORY_POINTS)
        self.chart_ax.set_ylim(0, 50)
        self.chart_ax.grid(True, alpha=0.3)
        kw = {"fontproperties": _font_prop} if _font_prop else {}
        self.chart_ax.set_title("温度曲线", **kw)
        self.line_max, = self.chart_ax.plot([], [], "r-", label="最高", linewidth=1.5)
        self.line_min, = self.chart_ax.plot([], [], "b-", label="最低", linewidth=1.5)
        self.line_avg, = self.chart_ax.plot([], [], "g-", label="平均", linewidth=1.5)
        if _font_prop:
            self.chart_ax.legend(prop=_font_prop, loc="upper left")
        else:
            self.chart_ax.legend(loc="upper left")
        self.chart_fig.subplots_adjust(left=0.1, right=0.97, top=0.85, bottom=0.18)
        self.chart_canvas = FigureCanvasTkAgg(self.chart_fig, f)
        self.chart_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    # ----------- 可见光面板 -----------
    def _build_visible_panel(self, parent):
        f = ttk.LabelFrame(parent, text="可见光画面 (默认关闭, 勾选启用)", padding=4)
        f.grid(row=0, column=0, sticky="nsew", pady=(0, 6))
        self.visible_view = VisibleView(f, initial_size=(360, 480))
        self.visible_view.pack(fill=tk.BOTH, expand=True)

    # ----------- 调试面板 -----------
    def _build_debug_panel(self, parent):
        f = ttk.LabelFrame(parent, text="串口调试", padding=4)
        f.grid(row=1, column=0, sticky="nsew")

        out_frame = ttk.Frame(f)
        out_frame.pack(fill=tk.BOTH, expand=True)
        try:
            from tkinter import font as tkfont
            _dbg_size = tkfont.nametofont("TkDefaultFont").cget("size")
        except Exception:
            _dbg_size = 14
        self.debug_text = tk.Text(out_frame, height=8, wrap=tk.WORD,
                                  font=("Consolas", _dbg_size - 2), bg="black", fg="lime")
        sb = ttk.Scrollbar(out_frame, command=self.debug_text.yview)
        self.debug_text.configure(yscrollcommand=sb.set)
        self.debug_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        sb.pack(side=tk.RIGHT, fill=tk.Y)

        cmd_frame = ttk.Frame(f)
        cmd_frame.pack(fill=tk.X, pady=(4, 0))
        ttk.Label(cmd_frame, text="命令:").pack(side=tk.LEFT)
        self.debug_cmd_var = tk.StringVar()
        e = ttk.Entry(cmd_frame, textvariable=self.debug_cmd_var)
        e.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)
        e.bind("<Return>", self._send_debug_command)
        ttk.Button(cmd_frame, text="发送", command=self._send_debug_command).pack(side=tk.LEFT)
        ttk.Button(cmd_frame, text="清空",
                   command=lambda: self.debug_text.delete("1.0", tk.END)).pack(side=tk.LEFT, padx=(4, 0))

        self._log("[启动] 双光上位机已启动", "info")
        self._log("[提示] 命令: GetSysInfo / activate <key> / stream / vstream", "info")

    # =================================================================
    # 串口监控与连接
    # =================================================================

    def _refresh_ports(self):
        self.available_ports = [(p.device, p.description) for p in serial.tools.list_ports.comports()]
        opts = ["自动搜索"] + [f"{d} ({desc})" for d, desc in self.available_ports]
        self.port_combo["values"] = opts
        if self.port_var.get() not in opts:
            self.port_var.set("自动搜索")

    def _on_port_selected(self, _evt=None):
        sel = self.port_var.get()
        self.manual_port = None if sel == "自动搜索" else sel.split(" (")[0]
        if self.is_connected and self.serial_port and self.manual_port:
            if self.serial_port.port != self.manual_port:
                self._disconnect()

    def _monitor_loop(self):
        """后台监控线程: 未连接 → 找设备并连; 已连接 → 检查活性."""
        while not self.stop_event.is_set():
            if not self.is_connected:
                target_port, info = self._find_target()
                if target_port and self._connect(target_port, info):
                    self.is_connected = True
                    self.msg_queue.put(("connected", target_port))
                else:
                    self.msg_queue.put(("searching", None))
                    self.stop_event.wait(2.0)
            else:
                # 已连接: 通过 reader_thread 自然检测断开
                self.stop_event.wait(2.0)

    def _find_target(self):
        """搜索目标端口. 返回 (port, device_info) 或 (None, None)."""
        # 用户手选优先
        if self.manual_port:
            ok, info = self._probe_device(self.manual_port)
            return (self.manual_port, info) if ok else (None, None)

        for port, desc in [(p.device, p.description) for p in serial.tools.list_ports.comports()]:
            if "USB" in desc.upper() or "CDC" in desc.upper() or "ACM" in desc.upper():
                ok, info = self._probe_device(port)
                if ok:
                    return port, info
        return None, None

    def _probe_device(self, port: str):
        """用一个临时串口对端口发 GetSysInfo, 解析 JSON 验证是否为目标设备."""
        try:
            with serial.Serial(port, self.BAUDRATE, timeout=2) as s:
                time.sleep(0.5)
                s.reset_input_buffer()
                s.write(b"GetSysInfo\n")
                deadline = time.time() + 3.0
                resp = b""
                while time.time() < deadline:
                    chunk = s.read(s.in_waiting or 1)
                    if chunk:
                        resp += chunk
                        if b"\n" in resp:
                            break
                # JSON 通常在 resp 中某行
                for line in resp.splitlines():
                    line = line.strip()
                    if line.startswith(b"{"):
                        try:
                            info = json.loads(line.decode("utf-8", errors="ignore"))
                            if all(k in info for k in ("version", "isActivated", "SerialNum")):
                                return True, info
                        except json.JSONDecodeError:
                            pass
        except Exception:
            pass
        return False, None

    def _connect(self, port: str, info: dict | None) -> bool:
        try:
            self.serial_port = serial.Serial(port, self.BAUDRATE, timeout=0.1)
            time.sleep(0.5)
        except Exception as e:
            self.msg_queue.put(("log", (f"[错误] 打开串口失败: {e}", "error")))
            return False

        if info is None:
            ok, info = self._probe_device(port)
            if not ok:
                try:
                    self.serial_port.close()
                except Exception:
                    pass
                self.serial_port = None
                return False

        self.device_info = info
        self.device_serial = info.get("SerialNum", "未获取")
        self.is_activated = bool(info.get("isActivated"))
        self.msg_queue.put(("device_info", info))

        # 启动接收线程
        self.parser.reset()
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()

        # 已激活 → 自动开热成像投屏
        if self.is_activated:
            self._start_thermal_streaming()
        return True

    def _disconnect(self):
        self.is_connected = False
        self.is_activated = False
        self._stop_thermal_streaming()
        self._stop_visible_streaming()
        if self.serial_port:
            try:
                self.serial_port.close()
            except Exception:
                pass
            self.serial_port = None
        self.msg_queue.put(("disconnected", None))

    # =================================================================
    # 串口字节流: 接收 + 派发到 FrameParser
    # =================================================================

    def _reader_loop(self):
        """单一接收线程, 把所有字节喂给 FrameParser. 任何异常 → 触发断开."""
        sp = self.serial_port
        while not self.stop_event.is_set() and sp and sp.is_open:
            try:
                data = sp.read(4096)
                if data:
                    self.parser.feed(data)
            except Exception as e:
                self.msg_queue.put(("log", (f"[错误] 串口读取异常: {e}", "error")))
                break
        # 触发断开 (UI 线程)
        self.msg_queue.put(("trigger_disconnect", None))

    def _on_thermal_frame_from_thread(self, t_max: float, t_min: float, t_avg: float, frame: np.ndarray):
        """**回调来自 reader 线程**, 不能直接动 UI, 转发到主线程."""
        # 拷贝以避免 frame_parser 内部缓冲被改 (实际上 parser 已经 copy 了)
        self.msg_queue.put(("thermal", (t_max, t_min, t_avg, frame)))

    def _on_visible_frame_from_thread(self, w: int, h: int, frame: np.ndarray):
        self.msg_queue.put(("visible", (w, h, frame)))

    # =================================================================
    # UI 主循环消息分发
    # =================================================================

    def _process_queue(self):
        # 退出中不再重新调度, 防止 "invalid command name" 错误
        if self.stop_event.is_set():
            return
        try:
            while True:
                kind, payload = self.msg_queue.get_nowait()
                if kind == "connected":
                    self.status_label.config(text=f"状态: 已连接 {payload}", foreground="green")
                    self._log(f"[连接] {payload}", "info")
                elif kind == "searching":
                    self.status_label.config(text="状态: 正在搜索设备...", foreground="orange")
                elif kind == "disconnected":
                    self.status_label.config(text="状态: 设备断开, 自动重连中...", foreground="red")
                    self._log("[断开] 设备已断开", "error")
                    self._reset_ui_after_disconnect()
                elif kind == "trigger_disconnect":
                    if self.is_connected:
                        self._disconnect()
                elif kind == "device_info":
                    self._handle_device_info(payload)
                elif kind == "thermal":
                    self._update_thermal(*payload)
                elif kind == "visible":
                    w, h, frame = payload
                    self.visible_view.update_frame(w, h, frame)
                elif kind == "log":
                    self._log(*payload)
                elif kind == "activation_response":
                    self._log(f"[激活] {payload}", "info")
        except queue.Empty:
            pass
        if not self.stop_event.is_set():
            self._after_id_queue = self.root.after(80, self._process_queue)

    def _reset_ui_after_disconnect(self):
        self.dev_info_frame.grid_remove()
        self.options_frame.grid_remove()
        self.main_frame.grid_remove()
        self.lbl_serial.config(text="--", foreground="gray")
        self.lbl_version.config(text="--", foreground="gray")
        self.lbl_activation.config(text="未激活", foreground="red")
        if self.activation_frame:
            self.activation_frame.destroy()
            self.activation_frame = None
        self.thermal_kalman.reset()
        self.history_max.clear()
        self.history_min.clear()
        self.history_avg.clear()
        self.visible_view.clear()
        w = int(640 * getattr(self, "_win_ratio", 1.0))
        h = int(360 * getattr(self, "_win_ratio", 1.0))
        self.root.geometry(f"{w}x{h}")

    def _handle_device_info(self, info: dict):
        """收到设备信息: 更新设备条 + 决定激活流程."""
        self.lbl_serial.config(text=info.get("SerialNum", "--"), foreground="black")
        self.lbl_version.config(text=info.get("version", "--"), foreground="black")
        self.is_activated = bool(info.get("isActivated"))
        self.lbl_activation.config(
            text="已激活" if self.is_activated else "未激活",
            foreground="green" if self.is_activated else "red",
        )
        self.dev_info_frame.grid()

        if self.is_activated:
            # 销毁激活面板, 显示主内容
            if self.activation_frame:
                self.activation_frame.destroy()
                self.activation_frame = None
            self.options_frame.grid()
            self.main_frame.grid()
            # 与老程一致的已激活尺寸 (可见光面板需多些横向空间 → 1300→ 1500)
            r = getattr(self, "_win_ratio", 1.0)
            # 屏幕可能装不下太大的主窗, 用屏幕分辨率裁顶
            sw = self.root.winfo_screenwidth()
            sh = self.root.winfo_screenheight()
            tw = min(int(1600 * r), sw - 80)
            th = min(int(900 * r), sh - 80)
            self.root.geometry(f"{tw}x{th}")
            self.root.minsize(min(int(1500 * r), tw), min(int(820 * r), th))
            self._start_thermal_streaming()
            # 默认开启可见光投屏 (var_visible_on=True)
            if self.var_visible_on.get():
                self._start_visible_streaming()
        else:
            self._show_activation_panel()

    # =================================================================
    # 激活流程
    # =================================================================

    def _show_activation_panel(self):
        if self.activation_frame:
            self.activation_frame.destroy()
        self.activation_frame = ttk.LabelFrame(self.root, text="设备激活", padding=10)
        self.activation_frame.grid(row=2, column=0, sticky="we", padx=10, pady=(0, 10))

        ttk.Label(self.activation_frame, text="序列号:").grid(row=0, column=0, sticky="w")
        ttk.Label(self.activation_frame, text=self.device_serial).grid(row=0, column=1, sticky="w", padx=(4, 16))

        ttk.Label(self.activation_frame, text="激活码:").grid(row=1, column=0, sticky="w", pady=(8, 0))
        self.activation_key_var = tk.StringVar()
        ttk.Entry(self.activation_frame, textvariable=self.activation_key_var, width=30).grid(
            row=1, column=1, sticky="we", padx=(4, 8), pady=(8, 0)
        )
        ttk.Button(self.activation_frame, text="激活", command=self._do_activate).grid(
            row=1, column=2, pady=(8, 0)
        )
        self.activation_frame.columnconfigure(1, weight=1)

    def _do_activate(self):
        if not (self.serial_port and self.serial_port.is_open):
            messagebox.showerror("错误", "设备未连接")
            return
        if self.activation_key_var is None:
            return
        key = self.activation_key_var.get().strip()
        if not key:
            messagebox.showerror("错误", "请输入激活码")
            return
        self._serial_write(f"activate {key}\n".encode("utf-8"))
        self._log(f"[激活] 已发送: activate {key}", "info")
        # 设备激活后会重启或直接返回 JSON, 我们 1.5s 后主动 GetSysInfo 一次
        self.root.after(1500, lambda: self._serial_write(b"GetSysInfo\n"))

    # =================================================================
    # 热成像 / 可见光投屏 (心跳 + UI 更新)
    # =================================================================

    def _serial_write(self, data: bytes):
        with self.serial_lock:
            if self.serial_port and self.serial_port.is_open:
                try:
                    self.serial_port.write(data)
                except Exception as e:
                    self.msg_queue.put(("log", (f"[错误] 串口写失败: {e}", "error")))

    # ---------- 热成像心跳 ----------
    def _start_thermal_streaming(self):
        if not self.var_thermal_on.get():
            return
        if self.thermal_hb_running:
            return
        self.thermal_hb_running = True
        self.thermal_hb_thread = threading.Thread(target=self._thermal_hb_loop, daemon=True)
        self.thermal_hb_thread.start()
        self._log("[热成像] 已启动推流", "info")

    def _stop_thermal_streaming(self):
        self.thermal_hb_running = False

    def _thermal_hb_loop(self):
        while self.thermal_hb_running and self.is_activated:
            self._serial_write(b"stream\n")
            if self.stop_event.wait(self.THERMAL_HB_INTERVAL):
                break

    def _toggle_thermal(self):
        if self.var_thermal_on.get():
            if self.is_activated:
                self._start_thermal_streaming()
        else:
            self._stop_thermal_streaming()
            self._log("[热成像] 已停止推流", "info")

    # ---------- 可见光心跳 ----------
    def _start_visible_streaming(self):
        if self.visible_hb_running:
            return
        self.visible_hb_running = True
        self.visible_hb_thread = threading.Thread(target=self._visible_hb_loop, daemon=True)
        self.visible_hb_thread.start()
        self._log("[可见光] 已启动推流", "info")

    def _stop_visible_streaming(self):
        self.visible_hb_running = False

    def _visible_hb_loop(self):
        while self.visible_hb_running and self.is_activated:
            self._serial_write(b"vstream\n")
            if self.stop_event.wait(self.VISIBLE_HB_INTERVAL):
                break

    def _toggle_visible(self):
        if self.var_visible_on.get():
            if not self.is_activated:
                messagebox.showwarning("提示", "设备尚未激活, 无法启动可见光推流")
                self.var_visible_on.set(False)
                return
            self._start_visible_streaming()
        else:
            self._stop_visible_streaming()
            self.visible_view.clear()
            self._log("[可见光] 已停止推流", "info")

    # ---------- 热帧 UI 更新 (主线程) ----------
    def _update_thermal(self, t_max, t_min, t_avg, frame: np.ndarray):
        if not self.var_thermal_on.get():
            return
        if t_avg is not None and t_avg > 1000:
            return  # 设备异常值, 跳过

        self.thermal_data = frame

        # 滤波
        if self.var_filter.get():
            tmax_s = self.kf_max.update(t_max)
            tmin_s = self.kf_min.update(t_min)
            tavg_s = self.kf_avg.update(t_avg)
            arr = self.thermal_kalman.filter(frame)
        else:
            tmax_s, tmin_s, tavg_s = t_max, t_min, t_avg
            arr = frame

        if HAS_CV2 and self.var_bilateral.get():
            arr = self._apply_bilateral(arr)

        # 与老程一致的显示朝向: 旋转 180 + 水平翻转
        display = np.fliplr(np.rot90(arr, 2))

        self.thermal_im.set_array(display)
        vmin, vmax = float(np.min(display)), float(np.max(display))
        self.thermal_im.set_clim(vmin=vmin, vmax=vmax)
        self.thermal_canvas.draw_idle()

        # 温度数值
        self.lbl_tmax.config(text=f"{tmax_s:.1f}°C")
        self.lbl_tmin.config(text=f"{tmin_s:.1f}°C")
        self.lbl_tavg.config(text=f"{tavg_s:.1f}°C")

        # 历史曲线
        self.history_max.append(tmax_s)
        self.history_min.append(tmin_s)
        self.history_avg.append(tavg_s)
        if len(self.history_max) > self.MAX_HISTORY_POINTS:
            self.history_max.pop(0)
            self.history_min.pop(0)
            self.history_avg.pop(0)
        x = list(range(len(self.history_max)))
        self.line_max.set_data(x, self.history_max)
        self.line_min.set_data(x, self.history_min)
        self.line_avg.set_data(x, self.history_avg)
        self.chart_ax.set_xlim(0, max(self.MAX_HISTORY_POINTS, len(x)))
        all_t = self.history_max + self.history_min + self.history_avg
        self.chart_ax.set_ylim(min(all_t) - 3, max(all_t) + 3)
        self.chart_canvas.draw_idle()

    @staticmethod
    def _apply_bilateral(arr: np.ndarray) -> np.ndarray:
        """对热帧做双边滤波保边降噪. cv2 处理 uint8, 这里需要先归一化."""
        lo, hi = float(arr.min()), float(arr.max())
        if hi - lo <= 1e-6:
            return arr
        norm = ((arr - lo) / (hi - lo) * 255.0).astype(np.uint8)
        out = cv2.bilateralFilter(norm, d=5, sigmaColor=50, sigmaSpace=50)
        return out.astype(np.float32) / 255.0 * (hi - lo) + lo

    # =================================================================
    # 调试区
    # =================================================================

    def _log(self, msg: str, kind: str = "normal"):
        colors = {"normal": "lime", "info": "cyan", "error": "red", "sent": "white"}
        ts = time.strftime("%H:%M:%S")
        line = f"[{ts}] {msg}\n"
        self.debug_text.insert(tk.END, line, kind)
        self.debug_text.tag_config(kind, foreground=colors.get(kind, "lime"))
        self.debug_text.see(tk.END)

    def _send_debug_command(self, _evt=None):
        cmd = self.debug_cmd_var.get().strip()
        if not cmd:
            return
        if not (self.serial_port and self.serial_port.is_open):
            self._log("[错误] 设备未连接", "error")
            return
        self._serial_write((cmd + "\n").encode("utf-8"))
        self._log(f">>> {cmd}", "sent")
        self.debug_cmd_var.set("")

    # =================================================================
    # 退出
    # =================================================================

    def shutdown(self):
        self.stop_event.set()
        # 取消待执行的 after 回调避免 Tk 销毁后被调用
        if self._after_id_queue is not None:
            try:
                self.root.after_cancel(self._after_id_queue)
            except Exception:
                pass
            self._after_id_queue = None
        self._stop_thermal_streaming()
        self._stop_visible_streaming()
        if self.serial_port:
            try:
                self.serial_port.close()
            except Exception:
                pass


def main():
    # Windows: 关闭系统 DPI 自动缩放, 让 tk scaling 真正生效
    # (默认 DPI-unaware, Windows 会按 96dpi 渲染再做位图缩放, 字体糊;
    #  DPI-aware 后 Tk 内部按真实 DPI 计算 scaling, 我们再覆写它)
    if os.name == "nt":
        try:
            import ctypes
            ctypes.windll.shcore.SetProcessDpiAwareness(2)  # PROCESS_PER_MONITOR_DPI_AWARE
        except Exception:
            try:
                ctypes.windll.user32.SetProcessDPIAware()
            except Exception:
                pass

    root = tk.Tk()
    # tk scaling 默认值 = 系统 DPI / 72. 1080p 96dpi → 1.333; 4K 192dpi → 2.667.
    # 我们直接覆写, 取一个明显比默认大的值. 环境变量 THERMAL_DUAL_SCALE 可覆写.
    try:
        scale_env = os.environ.get("THERMAL_DUAL_SCALE")
        scale_val = float(scale_env) if scale_env else 2.0
        root.tk.call("tk", "scaling", scale_val)
        print(f"[scaling] tk scaling set to {scale_val}, "
              f"actual = {root.tk.call('tk', 'scaling')}")
    except Exception as e:
        print(f"[scaling] failed: {e}")
    app = ThermalDualApp(root)

    def on_closing():
        app.shutdown()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()
