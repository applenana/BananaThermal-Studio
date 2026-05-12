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
import fusion_utils
from PIL import Image, ImageTk


# ============================================================================
# 字体加载 (matplotlib 中文支持). 兼容 PyInstaller 单文件 exe.
# ============================================================================
def _resource_base() -> str:
    """PyInstaller --onefile 时返回 sys._MEIPASS, 普通运行返回脚本目录."""
    if getattr(sys, "frozen", False) and hasattr(sys, "_MEIPASS"):
        return sys._MEIPASS  # type: ignore[attr-defined]
    return os.path.dirname(os.path.abspath(__file__))


def _find_bundled_font() -> str | None:
    """按优先级搜索 ttf/otf 字体. 用于 matplotlib 中文渲染."""
    base = _resource_base()
    # 候选目录: 资源根 / font 子目录 / 同级 fonts 目录
    candidates: list[str] = []
    for sub in ("", "font", "fonts"):
        d = os.path.join(base, sub) if sub else base
        if os.path.isdir(d):
            try:
                for fn in os.listdir(d):
                    low = fn.lower()
                    if low.endswith((".ttf", ".otf", ".ttc")):
                        candidates.append(os.path.join(d, fn))
            except OSError:
                pass
    # 优先 HarmonyOS / Smiley / 任意
    for prefer in ("harmonyos", "smileysans", "notosans", "msyh"):
        for c in candidates:
            if prefer in os.path.basename(c).lower():
                return c
    return candidates[0] if candidates else None


_FONT_PATH = _find_bundled_font()
if _FONT_PATH:
    _font_prop = fm.FontProperties(fname=_FONT_PATH)
    fm.fontManager.addfont(_FONT_PATH)
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

    def __init__(self, root: tk.Tk, win_ratio: float = 1.0):
        self.root = root
        self.root.title("热成像 + 可见光 双光上位机")

        # 字体 / 缩放: 由 main() 统一处理 DPI awareness + tk scaling.
        # win_ratio 用于把 geometry/minsize 像素值同步放大, 避免内容装不下.
        self._win_ratio = win_ratio
        self.root.geometry(self._scaled_geom(520, 110))
        self.root.minsize(*self._scaled(420, 90))
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
        self.reader_paused = False  # 图片下载 tab 激活时设为 True, 暂停 frame parser 喂数据

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

        # ---- 融合状态 (PC 端实时融合, 与 photo_download_tab 同款算法) ----
        self.fusion_mode_var = tk.StringVar(value="blend")  # off / blend / edge
        self.fusion_alpha = tk.DoubleVar(value=0.5)
        self.fusion_gamma = tk.DoubleVar(value=1.0)
        self.fusion_edge_strength = tk.DoubleVar(value=0.6)
        self.fusion_edge_thresh = tk.DoubleVar(value=0.082)
        self.fusion_edge_width = tk.IntVar(value=1)
        self.edge_color = "#333333"
        # ---- 伪彩 (与 photo_download_tab 同款) ----
        self.colormap_name = tk.StringVar(value="jet")
        self.mapping_curve = tk.StringVar(value="linear")  # linear / nonlinear
        self.use_custom_colors = tk.BooleanVar(value=False)
        self.cold_color = "#0000ff"
        self.mid_color = "#00ff00"
        self.hot_color = "#ff0000"
        self._latest_visible_pil: Image.Image | None = None  # 最近一帧可见光 (RGB), 主线程访问
        self._latest_thermal_arr: np.ndarray | None = None   # 最近一帧热像滤波后矩阵
        self._latest_clim: tuple[float, float] | None = None # 当前 vmin/vmax
        self._fusion_photo: ImageTk.PhotoImage | None = None  # 防 GC
        self._fusion_redraw_pending = False
        self._saved_fusion_mode: str | None = None  # tab 切换时暂存, 切回恢复

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

    def _scaled(self, *vals):
        r = self._win_ratio
        return tuple(int(v * r) for v in vals)

    def _scaled_geom(self, w, h):
        sw, sh = self._scaled(w, h)
        return f"{sw}x{sh}"

    def _setup_fonts(self):
        """统一基础字体大小, 与旧上位机视觉一致 (DPI-unaware, 9pt 系列).

        旧上位机/图片下载工具在 96 DPI 下渲染，控件字 9~10pt 视觉刚好.
        我们若不主动设, Tk 默认 MS Shell Dlg 2 size 8 偏小, 控件显得拥挤.
        """
        from tkinter import font as tkfont
        for name in ("TkDefaultFont", "TkTextFont", "TkMenuFont",
                     "TkHeadingFont", "TkCaptionFont", "TkSmallCaptionFont",
                     "TkIconFont", "TkTooltipFont"):
            try:
                tkfont.nametofont(name).configure(family="Microsoft YaHei UI", size=10)
            except Exception:
                pass

    def _build_ui(self):
        """UI 总入口: 单行顶栏 + Notebook 主内容. 激活前 Notebook 隐藏."""
        self._setup_fonts()
        # ============= 顶栏 (单行: 端口 / 状态 / 设备信息) =============
        top = ttk.Frame(self.root, padding=(10, 6))
        top.grid(row=0, column=0, sticky="we")

        # --- 串口选择 ---
        ttk.Label(top, text="串口:").pack(side=tk.LEFT)
        self.port_var = tk.StringVar(value="自动搜索")
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var,
                                       state="readonly", width=24)
        self.port_combo.pack(side=tk.LEFT, padx=(4, 4))
        self.port_combo.bind("<<ComboboxSelected>>", self._on_port_selected)
        ttk.Button(top, text="刷新", width=6,
                   command=self._refresh_ports).pack(side=tk.LEFT)

        ttk.Separator(top, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=10)

        # --- 状态 ---
        self.status_label = ttk.Label(top, text="正在搜索设备...", foreground="orange")
        self.status_label.pack(side=tk.LEFT)

        # --- 右侧设备信息条 (激活后显示) ---
        self.dev_info_frame = ttk.Frame(top)
        self._build_device_info(self.dev_info_frame)
        # 默认不 pack, 等 _handle_device_info 调用

        # ============= Notebook (实时投屏 / 图片下载) =============
        self.notebook = ttk.Notebook(self.root)
        self.notebook.grid(row=1, column=0, sticky="nsew", padx=8, pady=(2, 8))
        self.notebook.grid_remove()  # 激活前隐藏

        self.main_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.main_frame, text="实时投屏")
        self._build_realtime_tab(self.main_frame)

        self.photo_tab_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.photo_tab_frame, text="图片下载")
        self.photo_tab: object | None = None
        self.notebook.bind("<<NotebookTabChanged>>", self._on_tab_changed)

        # 激活面板占位
        self.activation_frame: ttk.LabelFrame | None = None
        self.activation_key_var: tk.StringVar | None = None

        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)

    def _build_realtime_tab(self, parent):
        """实时投屏 tab 布局:
            row 0: [融合主画面 | 控制面板(温度/曲线/显示/伪彩/融合)]
            row 1: 串口调试 (固定低高度)
        主画面高度跟随右侧控制面板内容自然高度 (row 0 不设 weight, 由控件自身决定).
        """
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(0, weight=1)   # 主区: 占满
        parent.rowconfigure(1, weight=0)   # 调试: 固定

        # ---------- row 0: 主画面 + 控制面板 ----------
        top_row = ttk.Frame(parent)
        top_row.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
        top_row.columnconfigure(0, weight=1, minsize=self._scaled(300)[0])
        top_row.columnconfigure(1, weight=0)   # 控制面板固定宽度, 不拉伸
        top_row.rowconfigure(0, weight=1)

        # 主画面 (左, 拉伸)
        view_box = ttk.LabelFrame(top_row, text="融合主画面", padding=4)
        view_box.grid(row=0, column=0, sticky="nsew", padx=(0, 4))
        self.fusion_label = tk.Label(view_box, anchor=tk.CENTER, background="black")
        self.fusion_label.pack(fill=tk.BOTH, expand=True)

        # 控制面板 (右, 固定较大宽度, 主画面相应变小)
        ctrl_panel_w = self._scaled(420)[0]
        ctrl_panel = ttk.Frame(top_row, width=ctrl_panel_w)
        ctrl_panel.grid(row=0, column=1, sticky="ns")
        ctrl_panel.grid_propagate(False)
        self._build_control_panel(ctrl_panel)

        # ---------- row 1: 调试 ----------
        self._build_debug_panel(parent)

        # 解析帧用的隐藏 VisibleView
        self.visible_view = VisibleView(parent, initial_size=(1, 1))

    def _build_control_panel(self, parent):
        """右侧控制面板: 温度/曲线/显示/伪彩/融合 5 张卡, 全部自然高度纵向叠加."""
        parent.columnconfigure(0, weight=1)

        # 1) 温度卡
        temp_card = ttk.LabelFrame(parent, text="实时温度", padding=6)
        temp_card.grid(row=0, column=0, sticky="ew", padx=4, pady=(4, 2))
        self._build_temperature_card(temp_card)

        # 2) 温度曲线 (与温度卡等宽)
        chart_card = ttk.LabelFrame(parent, text="温度曲线", padding=4)
        chart_card.grid(row=1, column=0, sticky="ew", padx=4, pady=2)
        self._build_chart_canvas(chart_card)

        # 3) 显示控制
        self.options_frame = ttk.LabelFrame(parent, text="显示控制", padding=6)
        self.options_frame.grid(row=2, column=0, sticky="ew", padx=4, pady=2)
        self._build_display_card(self.options_frame)

        # 4) 颜色映射
        cmap_card = ttk.LabelFrame(parent, text="颜色映射", padding=6)
        cmap_card.grid(row=3, column=0, sticky="ew", padx=4, pady=2)
        self._build_colormap_controls(cmap_card)

        # 5) 可见光融合
        fuse_card = ttk.LabelFrame(parent, text="可见光融合", padding=6)
        fuse_card.grid(row=4, column=0, sticky="ew", padx=4, pady=(2, 4))
        self._build_fusion_controls(fuse_card)

    def _build_temperature_card(self, parent):
        """三个温度数值水平紧凑排列. 字号与旧上位机一致 (Arial 12 bold)."""
        items = [("最高", "lbl_tmax", "red"),
                 ("最低", "lbl_tmin", "blue"),
                 ("平均", "lbl_tavg", "green")]
        for i, (label, attr, fg) in enumerate(items):
            parent.columnconfigure(i, weight=1)
            cell = ttk.Frame(parent)
            cell.grid(row=0, column=i, sticky="nsew")
            ttk.Label(cell, text=label).pack(side=tk.TOP)
            lbl = ttk.Label(cell, text="--°C",
                            font=("Arial", 14, "bold"), foreground=fg)
            lbl.pack(side=tk.TOP)
            setattr(self, attr, lbl)

    def _build_display_card(self, parent):
        """滤波加投屏共 4 个开关, 2 行 2 列网格."""
        self.var_filter = tk.BooleanVar(value=False)
        self.var_bilateral = tk.BooleanVar(value=HAS_CV2)
        self.var_thermal_on = tk.BooleanVar(value=True)
        self.var_visible_on = tk.BooleanVar(value=True)
        opts = [
            (0, 0, "卡尔曼滤波", self.var_filter, None, True),
            (0, 1, "双边滤波", self.var_bilateral, None, HAS_CV2),
            (1, 0, "热成像投屏", self.var_thermal_on, self._toggle_thermal, True),
            (1, 1, "可见光投屏", self.var_visible_on, self._toggle_visible, True),
        ]
        for r, c, txt, var, cmd, enabled in opts:
            kwargs = {"text": txt, "variable": var}
            if cmd is not None:
                kwargs["command"] = cmd
            if not enabled:
                kwargs["state"] = "disabled"
            ttk.Checkbutton(parent, **kwargs).grid(row=r, column=c, sticky="w", padx=2, pady=2)
        parent.columnconfigure(0, weight=1)
        parent.columnconfigure(1, weight=1)

    def _on_tab_changed(self, _evt=None):
        """切到图片下载 tab → 暂停实时投屏并交出串口; 切回 → 恢复"""
        try:
            current = self.notebook.index(self.notebook.select())
        except tk.TclError:
            return
        if current == 1:  # 图片下载 tab
            # 首次切入时延迟构建 PhotoDownloadTab (避免启动开销)
            if self.photo_tab is None:
                try:
                    from photo_download_tab import PhotoDownloadTab
                    self.photo_tab = PhotoDownloadTab(self.photo_tab_frame, self)
                    self.photo_tab.pack(fill=tk.BOTH, expand=True)
                except Exception as e:
                    self._log(f"[错误] 加载图片下载 tab 失败: {e}", "error")
                    import traceback
                    self._log(traceback.format_exc(), "error")
                    return
            # 关闭实时投屏的边缘融合 (与可见光投屏一样, tab 切走自动关)
            if self._saved_fusion_mode is None:
                self._saved_fusion_mode = self.fusion_mode_var.get()
            if self.fusion_mode_var.get() != "off":
                self.fusion_mode_var.set("off")
                self._on_fusion_mode_changed()
            # 暂停实时投屏 (心跳 + reader)
            self.acquire_serial(reason="图片下载 tab 激活")
        else:  # 实时投屏 tab
            self.release_serial(reason="切回实时投屏 tab")
            # 恢复融合模式
            if self._saved_fusion_mode is not None:
                self.fusion_mode_var.set(self._saved_fusion_mode)
                self._saved_fusion_mode = None
                self._on_fusion_mode_changed()

    # =================================================================
    # 串口让出 / 归还 (供图片下载 tab 使用)
    # =================================================================

    def acquire_serial(self, reason: str = ""):
        """让出串口给图片下载: 停心跳 + 暂停 reader. 返回 self.serial_port (可能 None)."""
        if not self.is_connected:
            return None
        self._log(f"[串口] 让出 ({reason})", "info") if reason else None
        # 停心跳
        self._stop_thermal_streaming()
        self._stop_visible_streaming()
        # 暂停 reader (loop 顶部会检查 reader_paused)
        self.reader_paused = True
        # 等 reader 进入空转 (read timeout=0.1, 留 0.2s 余量)
        time.sleep(0.2)
        # 清掉串口 buffer 里的残余推流字节, 避免污染下载响应
        try:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.reset_input_buffer()
                self.serial_port.reset_output_buffer()
        except Exception:
            pass
        # 重置 parser, 避免下次 release 时残留 state 错位
        self.parser.reset()
        return self.serial_port

    def release_serial(self, reason: str = ""):
        """图片下载完成 → 恢复 reader + 心跳."""
        if not self.is_connected:
            return
        self._log(f"[串口] 归还 ({reason})", "info") if reason else None
        # 清空串口 buffer 中下载残余 (e.g. 设备保存的图片字节)
        try:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.reset_input_buffer()
        except Exception:
            pass
        self.parser.reset()
        # 恢复 reader
        self.reader_paused = False
        # 恢复心跳 (按用户配置)
        if self.is_activated:
            if self.var_thermal_on.get():
                self._start_thermal_streaming()
            if self.var_visible_on.get():
                self._start_visible_streaming()

    # ----------- 温度曲线 -----------
    def _build_chart_canvas(self, parent):
        self.chart_fig = plt.figure(figsize=(4.2, 1.8), dpi=80)
        self.chart_ax = self.chart_fig.add_subplot(111)
        self.chart_ax.set_xlim(0, self.MAX_HISTORY_POINTS)
        self.chart_ax.set_ylim(0, 50)
        self.chart_ax.grid(True, alpha=0.3)
        self.line_max, = self.chart_ax.plot([], [], "r-", label="Max", linewidth=1.2)
        self.line_min, = self.chart_ax.plot([], [], "b-", label="Min", linewidth=1.2)
        self.line_avg, = self.chart_ax.plot([], [], "g-", label="Avg", linewidth=1.2)
        self.chart_ax.legend(loc="upper left", fontsize=7)
        self.chart_ax.tick_params(labelsize=7)
        self.chart_fig.subplots_adjust(left=0.10, right=0.99, top=0.95, bottom=0.18)
        self.chart_canvas = FigureCanvasTkAgg(self.chart_fig, parent)
        self.chart_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    # ----------- 伪彩 (颜色映射) 控件 -----------
    def _build_colormap_controls(self, parent):
        """伪彩控件: 与图片下载 tab 一致. parent 已是 LabelFrame, 不重复标题."""
        # 行1: 映射曲线 + 调色盘
        r1 = ttk.Frame(parent)
        r1.pack(side=tk.TOP, fill=tk.X, pady=2)
        ttk.Label(r1, text="曲线:").pack(side=tk.LEFT, padx=(0, 2))
        ttk.Radiobutton(r1, text="线性", variable=self.mapping_curve, value="linear",
                        command=self._schedule_fusion_redraw).pack(side=tk.LEFT, padx=2)
        ttk.Radiobutton(r1, text="S曲线", variable=self.mapping_curve, value="nonlinear",
                        command=self._schedule_fusion_redraw).pack(side=tk.LEFT, padx=2)
        ttk.Label(r1, text="调色盘:").pack(side=tk.LEFT, padx=(10, 2))
        cmaps = ['jet', 'hot', 'cool', 'rainbow', 'viridis', 'plasma',
                 'inferno', 'magma', 'cividis', 'turbo', 'coolwarm']
        cb = ttk.Combobox(r1, textvariable=self.colormap_name, values=cmaps,
                          width=10, state='readonly')
        cb.pack(side=tk.LEFT, padx=2)
        cb.bind('<<ComboboxSelected>>', lambda _e: self._schedule_fusion_redraw())

        # 行2: 自定义三色
        r2 = ttk.Frame(parent)
        r2.pack(side=tk.TOP, fill=tk.X, pady=2)
        ttk.Checkbutton(r2, text="使用自定义颜色", variable=self.use_custom_colors,
                        command=self._schedule_fusion_redraw).pack(side=tk.LEFT)
        ttk.Label(r2, text="冷:").pack(side=tk.LEFT, padx=(10, 2))
        self.cold_color_btn = tk.Button(r2, text="  ", bg=self.cold_color,
                                        width=3, command=lambda: self._pick_cmap_color('cold'))
        self.cold_color_btn.pack(side=tk.LEFT, padx=2)
        ttk.Label(r2, text="中:").pack(side=tk.LEFT, padx=(6, 2))
        self.mid_color_btn = tk.Button(r2, text="  ", bg=self.mid_color,
                                       width=3, command=lambda: self._pick_cmap_color('mid'))
        self.mid_color_btn.pack(side=tk.LEFT, padx=2)
        ttk.Label(r2, text="热:").pack(side=tk.LEFT, padx=(6, 2))
        self.hot_color_btn = tk.Button(r2, text="  ", bg=self.hot_color,
                                       width=3, command=lambda: self._pick_cmap_color('hot'))
        self.hot_color_btn.pack(side=tk.LEFT, padx=2)

    def _pick_cmap_color(self, which: str):
        from tkinter import colorchooser
        cur = {"cold": self.cold_color, "mid": self.mid_color, "hot": self.hot_color}[which]
        title = {"cold": "选择最低温颜色", "mid": "选择中间温颜色", "hot": "选择最高温颜色"}[which]
        c = colorchooser.askcolor(color=cur, title=title)
        if c and c[1]:
            if which == "cold":
                self.cold_color = c[1]; self.cold_color_btn.config(bg=c[1])
            elif which == "mid":
                self.mid_color = c[1]; self.mid_color_btn.config(bg=c[1])
            else:
                self.hot_color = c[1]; self.hot_color_btn.config(bg=c[1])
            self._schedule_fusion_redraw()

    def _build_fusion_controls(self, parent):
        """融合参数控件: 模式 + gamma + alpha + edge 系列. parent 已是 LabelFrame."""
        ctrl = ttk.Frame(parent)
        ctrl.pack(side=tk.TOP, fill=tk.X)

        # 行1: 模式
        r1 = ttk.Frame(ctrl)
        r1.pack(side=tk.TOP, fill=tk.X, pady=2)
        ttk.Label(r1, text="模式:").pack(side=tk.LEFT, padx=(0, 4))
        for txt, val in (("纯热像", "off"), ("混合", "blend"), ("边缘", "edge")):
            ttk.Radiobutton(r1, text=txt, variable=self.fusion_mode_var, value=val,
                            command=self._on_fusion_mode_changed).pack(side=tk.LEFT, padx=2)

        # 行2: blend
        self.fr_blend = ttk.Frame(ctrl)
        ttk.Label(self.fr_blend, text="可见光比例:").pack(side=tk.LEFT, padx=(0, 2))
        ttk.Scale(self.fr_blend, from_=0.0, to=1.0, variable=self.fusion_alpha,
                  orient=tk.HORIZONTAL, length=120,
                  command=lambda _e: self._schedule_fusion_redraw()).pack(side=tk.LEFT, padx=2)
        ttk.Label(self.fr_blend, text="伽马:").pack(side=tk.LEFT, padx=(8, 2))
        ttk.Scale(self.fr_blend, from_=0.3, to=3.0, variable=self.fusion_gamma,
                  orient=tk.HORIZONTAL, length=100,
                  command=lambda _e: self._schedule_fusion_redraw()).pack(side=tk.LEFT, padx=2)

        # 行3: edge
        self.fr_edge = ttk.Frame(ctrl)
        ttk.Label(self.fr_edge, text="强度:").pack(side=tk.LEFT, padx=(0, 2))
        ttk.Scale(self.fr_edge, from_=0.0, to=1.0, variable=self.fusion_edge_strength,
                  orient=tk.HORIZONTAL, length=80,
                  command=lambda _e: self._schedule_fusion_redraw()).pack(side=tk.LEFT, padx=2)
        ttk.Label(self.fr_edge, text="阈值:").pack(side=tk.LEFT, padx=(6, 2))
        ttk.Scale(self.fr_edge, from_=0.0, to=1.0, variable=self.fusion_edge_thresh,
                  orient=tk.HORIZONTAL, length=80,
                  command=lambda _e: self._schedule_fusion_redraw()).pack(side=tk.LEFT, padx=2)
        ttk.Label(self.fr_edge, text="粗细:").pack(side=tk.LEFT, padx=(6, 2))
        ttk.Spinbox(self.fr_edge, from_=0, to=6, width=3, textvariable=self.fusion_edge_width,
                    command=self._schedule_fusion_redraw).pack(side=tk.LEFT, padx=2)
        self.edge_color_btn = tk.Button(self.fr_edge, text="边缘色", bg=self.edge_color, fg="white",
                                        command=self._pick_edge_color, width=6)
        self.edge_color_btn.pack(side=tk.LEFT, padx=(6, 2))

        # edge 模式可见光伽马
        self.fr_edge_gamma = ttk.Frame(ctrl)
        ttk.Label(self.fr_edge_gamma, text="可见光伽马:").pack(side=tk.LEFT, padx=(0, 2))
        ttk.Scale(self.fr_edge_gamma, from_=0.3, to=3.0, variable=self.fusion_gamma,
                  orient=tk.HORIZONTAL, length=120,
                  command=lambda _e: self._schedule_fusion_redraw()).pack(side=tk.LEFT, padx=2)

        self._on_fusion_mode_changed()

    def _on_fusion_mode_changed(self):
        m = self.fusion_mode_var.get()
        for fr in (getattr(self, "fr_blend", None),
                   getattr(self, "fr_edge", None),
                   getattr(self, "fr_edge_gamma", None)):
            if fr is not None:
                fr.pack_forget()
        if m == "blend" and hasattr(self, "fr_blend"):
            self.fr_blend.pack(side=tk.TOP, fill=tk.X, pady=2)
        elif m == "edge" and hasattr(self, "fr_edge"):
            self.fr_edge.pack(side=tk.TOP, fill=tk.X, pady=2)
            self.fr_edge_gamma.pack(side=tk.TOP, fill=tk.X, pady=2)
        self._schedule_fusion_redraw()

    def _pick_edge_color(self):
        from tkinter import colorchooser
        c = colorchooser.askcolor(color=self.edge_color, title="选择边缘颜色")
        if c and c[1]:
            self.edge_color = c[1]
            self.edge_color_btn.config(bg=c[1])
            self._schedule_fusion_redraw()

    # ----------- 设备信息条 -----------
    def _build_device_info(self, parent):
        ttk.Label(parent, text="序列号:").pack(side=tk.LEFT, padx=(0, 2))
        self.lbl_serial = ttk.Label(parent, text="--", foreground="gray")
        self.lbl_serial.pack(side=tk.LEFT, padx=(0, 10))
        ttk.Label(parent, text="版本:").pack(side=tk.LEFT, padx=(0, 2))
        self.lbl_version = ttk.Label(parent, text="--", foreground="gray")
        self.lbl_version.pack(side=tk.LEFT, padx=(0, 10))
        ttk.Label(parent, text="激活:").pack(side=tk.LEFT, padx=(0, 2))
        self.lbl_activation = ttk.Label(parent, text="未激活", foreground="red")
        self.lbl_activation.pack(side=tk.LEFT)

    # ----------- 调试面板 -----------
    def _build_debug_panel(self, parent):
        f = ttk.LabelFrame(parent, text="串口调试", padding=4)
        f.grid(row=1, column=0, sticky="nsew", padx=4, pady=(0, 4))

        out_frame = ttk.Frame(f)
        out_frame.pack(fill=tk.BOTH, expand=True)
        self.debug_text = tk.Text(out_frame, height=4, wrap=tk.WORD,
                                  font=("Consolas", 9), bg="black", fg="lime")
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
        """单一接收线程, 把所有字节喂给 FrameParser. 任何异常 → 触发断开.
        当 reader_paused=True 时不读取串口, 让出给图片下载 tab.
        """
        sp = self.serial_port
        while not self.stop_event.is_set() and sp and sp.is_open:
            if self.reader_paused:
                # 让出串口给图片下载 tab. 不读, 短暂 sleep
                time.sleep(0.05)
                continue
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
                    # 缓存最新可见光 PIL (含旋转/镜像), 供融合主画面使用
                    try:
                        self._latest_visible_pil = self.visible_view.get_latest_image_pil()
                    except Exception:
                        self._latest_visible_pil = None
                    self._schedule_fusion_redraw()
                elif kind == "log":
                    self._log(*payload)
                elif kind == "activation_response":
                    self._log(f"[激活] {payload}", "info")
        except queue.Empty:
            pass
        if not self.stop_event.is_set():
            self._after_id_queue = self.root.after(80, self._process_queue)

    def _reset_ui_after_disconnect(self):
        self.dev_info_frame.pack_forget()
        self.notebook.grid_remove()
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
        self._latest_visible_pil = None
        self._latest_thermal_arr = None
        if hasattr(self, "fusion_label"):
            self.fusion_label.configure(image="")
            self._fusion_photo = None
        self.root.geometry(self._scaled_geom(520, 110))
        self.root.minsize(*self._scaled(420, 90))

    def _handle_device_info(self, info: dict):
        """收到设备信息: 更新设备条 + 决定激活流程."""
        self.lbl_serial.config(text=info.get("SerialNum", "--"), foreground="black")
        self.lbl_version.config(text=info.get("version", "--"), foreground="black")
        self.is_activated = bool(info.get("isActivated"))
        self.lbl_activation.config(
            text="已激活" if self.is_activated else "未激活",
            foreground="green" if self.is_activated else "red",
        )
        if not self.dev_info_frame.winfo_ismapped():
            self.dev_info_frame.pack(side=tk.RIGHT)

        if self.is_activated:
            # 销毁激活面板, 显示主内容
            if self.activation_frame:
                self.activation_frame.destroy()
                self.activation_frame = None
            self.notebook.grid()
            # 与图片下载工具同源, 多了右侧控制面板, 扩宽一点. 按 DPI 比例放大.
            sw = self.root.winfo_screenwidth()
            sh = self.root.winfo_screenheight()
            tw_target, th_target = self._scaled(1280, 820)
            tw = min(tw_target, sw - 80)
            th = min(th_target, sh - 80)
            self.root.geometry(f"{tw}x{th}")
            self.root.minsize(*self._scaled(1100, 700))
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
            self._latest_visible_pil = None
            self._schedule_fusion_redraw()
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
        self._latest_thermal_arr = display
        vmin, vmax = float(np.min(display)), float(np.max(display))
        self._latest_clim = (vmin, vmax)

        # 重绘融合主画面 (走节流, 30ms 内合并)
        self._schedule_fusion_redraw()

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
        # 曲线 5 帧重绘一次, 避免 matplotlib 每帧 draw 占满 UI 线程
        self._chart_frame_counter = getattr(self, "_chart_frame_counter", 0) + 1
        if self._chart_frame_counter % 5 == 0:
            x = list(range(len(self.history_max)))
            self.line_max.set_data(x, self.history_max)
            self.line_min.set_data(x, self.history_min)
            self.line_avg.set_data(x, self.history_avg)
            self.chart_ax.set_xlim(0, max(self.MAX_HISTORY_POINTS, len(x)))
            all_t = self.history_max + self.history_min + self.history_avg
            self.chart_ax.set_ylim(min(all_t) - 3, max(all_t) + 3)
            self.chart_canvas.draw_idle()

    # ---------- 融合主画面渲染 ----------
    def _schedule_fusion_redraw(self):
        """节流: 30ms 内合并多次重绘请求."""
        if self._fusion_redraw_pending:
            return
        self._fusion_redraw_pending = True
        try:
            self.root.after(30, self._redraw_fusion)
        except Exception:
            self._fusion_redraw_pending = False

    def _redraw_fusion(self):
        """用 colormap 把热像染色, 上采样, 叠加可见光融合后显示."""
        self._fusion_redraw_pending = False
        if self._latest_thermal_arr is None or self._latest_clim is None:
            return
        if not hasattr(self, "fusion_label"):
            return
        try:
            display = self._latest_thermal_arr
            vmin, vmax = self._latest_clim
            denom = max(vmax - vmin, 1e-6)
            norm = np.clip((display - vmin) / denom, 0.0, 1.0)
            # 用 colorize 染色 (含线性/非线性 + 自定义颜色)
            therm_rgb = fusion_utils.colorize(
                norm,
                colormap_name=self.colormap_name.get(),
                mapping_curve=self.mapping_curve.get(),
                use_custom_colors=self.use_custom_colors.get(),
                cold_color=self.cold_color,
                mid_color=self.mid_color,
                hot_color=self.hot_color,
            )
            therm_pil = Image.fromarray(therm_rgb, mode="RGB")

            # 计算目标显示尺寸: 跟 fusion_label 实际大小, 保持热像宽高比
            w_now = self.fusion_label.winfo_width()
            h_now = self.fusion_label.winfo_height()
            if w_now <= 1 or h_now <= 1:
                w_now, h_now = 360, 270
            th, tw = display.shape
            scale = min((w_now - 4) / tw, (h_now - 4) / th)
            if scale > 1:
                new_size = (max(int(tw * scale), 1), max(int(th * scale), 1))
                therm_pil = therm_pil.resize(new_size, Image.Resampling.BILINEAR)

            # 融合
            mode = self.fusion_mode_var.get()
            vis_pil = self._latest_visible_pil if self.var_visible_on.get() else None
            fused = fusion_utils.fuse(
                therm_pil, vis_pil,
                mode=mode,
                gamma=float(self.fusion_gamma.get()),
                alpha=float(self.fusion_alpha.get()),
                edge_strength=float(self.fusion_edge_strength.get()),
                edge_thresh=float(self.fusion_edge_thresh.get()),
                edge_width=int(self.fusion_edge_width.get()),
                edge_color=self.edge_color,
            )

            self._fusion_photo = ImageTk.PhotoImage(fused)
            self.fusion_label.configure(image=self._fusion_photo)
        except Exception as e:
            self._log(f"[错误] 融合渲染失败: {e}", "error")

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
    # 强制 125% 缩放: 启用 DPI 感知 + 固定 scaling = 1.25
    # 窗口尺寸也会乘以 _win_ratio, 保证内容装得下
    DPI_SCALE = 1.50
    if os.name == "nt":
        try:
            import ctypes
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except Exception:
            try:
                ctypes.windll.user32.SetProcessDPIAware()
            except Exception:
                pass

    root = tk.Tk()
    try:
        root.tk.call("tk", "scaling", DPI_SCALE * 96.0 / 72.0)
    except Exception:
        pass
    app = ThermalDualApp(root, win_ratio=DPI_SCALE)

    def on_closing():
        app.shutdown()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()
