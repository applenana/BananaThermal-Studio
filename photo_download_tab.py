import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import serial
import serial.tools.list_ports
import threading
import time
import os
import sys
import json
import struct
import numpy as np
from datetime import datetime
from pathlib import Path
from PIL import Image, ImageTk, ImageFilter
import matplotlib.pyplot as plt
from matplotlib import cm


def get_base_dir():
    """获取程序的基础目录（兼容打包后的exe）"""
    if getattr(sys, 'frozen', False):
        return Path(sys.executable).parent
    else:
        return Path(__file__).parent


def get_resource_dir():
    """获取打包资源目录（单文件EXE模式下为临时解压目录）"""
    if getattr(sys, 'frozen', False) and hasattr(sys, '_MEIPASS'):
        return Path(sys._MEIPASS)
    return get_base_dir()


class PhotoDownloadTab(ttk.Frame):
    """图片下载与渲染面板 (作为上位机 ttk.Notebook 的一个 tab).

    依赖外部 ``app`` 对象提供串口: ``app.serial_port`` (pyserial.Serial 或 None);
    以及 ``app.acquire_serial(reason)`` / ``app.release_serial(reason)`` 协调与
    实时投屏 reader/heartbeat 的互斥. 此 tab 不自己管理串口连接, 不发激活命令.
    """

    def __init__(self, parent, app):
        super().__init__(parent)
        self.parent = parent
        self.app = app
        # 顶层窗口引用 (用于 after 调度 / 弹框父窗口)
        self.root = parent.winfo_toplevel()
        # 兼容 ThermalImageManager 旧接口: is_connected 仅用于 UI 状态展示
        self.is_connected = False
        self.auto_keywords = []  # 不再自动扫描串口

        # 创建保存目录
        self.base_dir = get_base_dir()  # 使用新的获取目录函数
        self.create_directories()
        
        # 信息条位置选项
        self.info_position = tk.StringVar(value="bottom")  # 默认底部
        
        # 颜色映射配置
        self.mapping_curve = tk.StringVar(value="linear")  # 映射曲线: linear/nonlinear，默认线性
        self.colormap_name = tk.StringVar(value="jet")  # 调色盘名称
        self.use_custom_colors = tk.BooleanVar(value=False)  # 是否使用自定义颜色
        self.hot_color = "#ff0000"  # 最高温颜色（红色）
        self.mid_color = "#00ff00"  # 中间温颜色（绿色）
        self.cold_color = "#0000ff"  # 最低温颜色（蓝色）
        
        # 滤波配置
        self.filter_strength = tk.StringVar(value="medium")  # 卡方滤波强度: off/light/medium/strong
        self.area_denoise = tk.StringVar(value="light")  # 区域降噪滤波: off/light/medium/strong，默认轻度

        # ===== 可见光融合配置 (仅作用于 v2 HTPH 双光文件; 旧 v1 文件忽略) =====
        self.visible_enable = tk.BooleanVar(value=False)        # 启用可见光融合显示 (默认关闭)
        self.fusion_mode_var = tk.StringVar(value="off")        # off / edge / blend (默认 off)
        self.fusion_alpha = tk.DoubleVar(value=0.45)            # blend 混合度 0~1
        self.fusion_gamma = tk.DoubleVar(value=1.0)             # 可见光伽马 0.3~3.0
        self.fusion_edge_strength = tk.DoubleVar(value=0.6)     # edge 叠加强度 0~1
        self.fusion_edge_thresh = tk.DoubleVar(value=0.082)     # edge 阈值: Sobel 归一化幅值高于此才算边缘
        self.fusion_edge_width = tk.IntVar(value=1)             # edge 粗细 (膨胀像素 0~6)
        self.edge_color = "#333333"                             # 边缘融合颜色 (默认深灰, 设备端同款)
        self._last_edge_log_t = 0.0                             # 边缘参数日志去抖时间戳
        self.current_visible_image = None                       # 当前帧解码后的可见光 PIL.RGB (热成像同尺寸前的原始大小)
        
        # 温度数据缓存（用于鼠标悬浮显示）
        self.current_temp_matrix = None  # 当前温度矩阵
        self.current_t_min = None  # 当前最低温
        self.current_t_max = None  # 当前最高温
        self.temp_label = None  # 温度显示标签
        self.thermal_image_size = None  # 原始热成像图片尺寸（用于坐标映射）
        
        # 温度标记点列表 [(matrix_x, matrix_y, temp), ...]
        self.temp_markers = []
        # 拖拽状态
        self.dragging_marker_index = None  # 正在拖拽的标记索引
        self.drag_started = False  # 是否已开始拖拽（区分点击和拖拽）
        
        # 创建GUI
        self.create_gui()
        
        # 搜索并加载字体（在GUI创建之后）
        self.font = self.load_font()

        # 串口由 self.app 提供, 不在此自动连接
        # 监听 app 串口连接变化, 更新本 tab 的 status_label
        self._sync_status_from_app()

    # ---- 共享串口属性: 桥接到 app.serial_port ----
    @property
    def serial_port(self):
        return self.app.serial_port if self.app else None

    @serial_port.setter
    def serial_port(self, value):
        # 只允许置 None (用于本 tab 内部断开标记); 不影响 app 持有的真实串口
        if value is None:
            return
        # 若上游真要替换, 委派给 app
        if self.app:
            self.app.serial_port = value

    def _sync_status_from_app(self):
        """每秒同步一次 app 的连接状态到本 tab 的 status_label"""
        try:
            connected = bool(self.app and self.app.is_connected
                             and self.app.serial_port
                             and self.app.serial_port.is_open)
            was_connected = self.is_connected
            self.is_connected = connected
            if hasattr(self, 'status_label'):
                if connected:
                    self.status_label.config(text=f"已连接 {self.app.serial_port.port}",
                                             foreground="green")
                else:
                    self.status_label.config(text="未连接", foreground="red")
            # 串口刚变可用 + 本 tab 已可见 (notebook 当前选中) → 自动获取一次列表
            if connected and not was_connected:
                try:
                    nb = self.app.notebook
                    if nb.index(nb.select()) == 1 and not getattr(self, 'list_fetched_successfully', False):
                        self.start_auto_get_list()
                except Exception:
                    pass
        except Exception:
            pass
        try:
            self.root.after(1000, self._sync_status_from_app)
        except Exception:
            pass

    def create_directories(self):
        """创建必要的目录结构"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.work_dir = self.base_dir / f"photo_download_{timestamp}"
        self.raw_dat_dir = self.work_dir / "raw_dat"
        self.rendered_jpg_dir = self.work_dir / "rendered_jpg"
        self.final_collage_dir = self.work_dir / "final_collage"
        self.visible_png_dir = self.work_dir / "visible_png"

        for directory in [self.raw_dat_dir, self.rendered_jpg_dir, self.final_collage_dir, self.visible_png_dir]:
            directory.mkdir(parents=True, exist_ok=True)
    
    def load_font(self):
        """自动搜索并加载字体文件"""
        try:
            from PIL import ImageFont
            
            # 资源目录（打包时字体在此）和程序目录都搜索
            search_dirs = [get_resource_dir(), self.base_dir,
                           self.base_dir / "font",  # 上位机自带 font/ 目录
                           Path(__file__).parent / "font"]
            ttf_files = []
            for d in search_dirs:
                try:
                    if d.exists():
                        ttf_files.extend(d.glob("*.ttf"))
                        ttf_files.extend(d.glob("*.otf"))
                except Exception:
                    pass
            
            if ttf_files:
                font_path = ttf_files[0]
                self.log(f"找到字体文件: {font_path.name}")
                try:
                    font = ImageFont.truetype(str(font_path), 24)  # 增大字体从16到24
                    self.log(f"成功加载字体: {font_path.name}")
                    return font
                except Exception as e:
                    self.log(f"加载字体失败: {e}, 使用默认字体")
                    return ImageFont.load_default()
            else:
                self.log("未找到 .ttf 字体文件，使用默认字体")
                return ImageFont.load_default()
                
        except Exception as e:
            self.log(f"字体加载错误: {e}")
            return None
            
    def create_gui(self):
        """创建GUI界面"""
        # 主框架: 直接挂在 self (ttk.Frame) 上, 不动顶层 root
        main_frame = ttk.Frame(self, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # 顶部状态条 (替代原"串口连接"区域. 串口由上位机统一管理)
        connection_frame = ttk.Frame(main_frame)
        connection_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        ttk.Label(connection_frame, text="串口状态:").grid(row=0, column=0, padx=(0, 5))
        self.status_label = ttk.Label(connection_frame, text="未连接", foreground="red")
        self.status_label.grid(row=0, column=1, padx=(0, 16))
        ttk.Label(connection_frame, text="(由实时投屏 tab 统一管理串口)",
                  foreground="gray").grid(row=0, column=2)

        # ---- 旧版串口控件占位变量 (保持 ThermalImageManager 旧代码兼容) ----
        self.port_var = tk.StringVar(value="")
        self.port_combo = None
        self.refresh_btn = None
        self.connect_btn = None

        # 图片管理区域 - 左右布局
        image_frame = ttk.LabelFrame(main_frame, text="图片管理", padding="10")
        image_frame.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        
        # 左侧 - 图片列表
        left_frame = ttk.Frame(image_frame)
        left_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 5))
        
        ttk.Label(left_frame, text="图片列表").pack(anchor=tk.W, pady=(0, 5))
        
        # 图片列表框
        list_frame = ttk.Frame(left_frame)
        list_frame.pack(fill=tk.BOTH, expand=True)
        
        self.image_listbox = tk.Listbox(list_frame, width=25, height=20)
        self.image_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.image_listbox.bind('<<ListboxSelect>>', self.on_image_selected)
        
        list_scrollbar = ttk.Scrollbar(list_frame, orient="vertical", command=self.image_listbox.yview)
        list_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.image_listbox.configure(yscrollcommand=list_scrollbar.set)
        
        # 右侧 - 图片画布
        right_frame = ttk.Frame(image_frame)
        right_frame.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        ttk.Label(right_frame, text="图片预览").pack(anchor=tk.W, pady=(0, 5))
        
        # 图片画布
        canvas_frame = ttk.Frame(right_frame)
        canvas_frame.pack(fill=tk.BOTH, expand=True)
        
        self.image_canvas = tk.Canvas(canvas_frame, bg='#2b2b2b', highlightthickness=1, highlightbackground='#555555')
        self.image_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        canvas_scrollbar_y = ttk.Scrollbar(canvas_frame, orient="vertical", command=self.image_canvas.yview)
        canvas_scrollbar_y.pack(side=tk.RIGHT, fill=tk.Y)
        
        canvas_scrollbar_x = ttk.Scrollbar(right_frame, orient="horizontal", command=self.image_canvas.xview)
        canvas_scrollbar_x.pack(side=tk.BOTTOM, fill=tk.X)
        
        self.image_canvas.configure(yscrollcommand=canvas_scrollbar_y.set, xscrollcommand=canvas_scrollbar_x.set)
        
        # 绑定鼠标事件用于显示温度
        self.image_canvas.bind('<Motion>', self.on_canvas_mouse_move)
        self.image_canvas.bind('<Leave>', self.on_canvas_mouse_leave)
        self.image_canvas.bind('<Button-1>', self.on_canvas_click)
        self.image_canvas.bind('<B1-Motion>', self.on_canvas_drag)
        self.image_canvas.bind('<ButtonRelease-1>', self.on_canvas_release)
        self.image_canvas.bind('<Button-3>', self.on_canvas_right_click)
        
        # 创建温度显示标签
        self.temp_label = tk.Label(right_frame, text="", bg="yellow", fg="black", 
                                   font=('Arial', 10, 'bold'), relief=tk.RAISED, padx=5, pady=2)
        # 初始隐藏
        
        # 配置图片管理区域的列权重
        image_frame.columnconfigure(0, weight=1, minsize=200)  # 左侧列表,最小宽度200
        image_frame.columnconfigure(1, weight=4, minsize=400)  # 右侧画布,最小宽度400
        image_frame.rowconfigure(0, weight=1)
        
        # 图片管理控制按钮区域(在图片管理下方)
        control_frame = ttk.Frame(image_frame)
        control_frame.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(5, 0))
        
        ttk.Button(control_frame, text="获取图片列表", command=self.get_image_list).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_frame, text="清空列表", command=self.clear_image_list).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_frame, text="下载图片", command=self.download_selected_image).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_frame, text="打开保存目录", command=self.open_save_directory).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_frame, text="清除标记", command=self.clear_temp_markers).pack(side=tk.LEFT, padx=5)
        
        # 信息条位置选择
        ttk.Label(control_frame, text="信息条位置:").pack(side=tk.LEFT, padx=(20, 5))
        positions = [("上方", "top"), ("下方", "bottom"), ("左侧", "left"), ("右侧", "right")]
        for text, value in positions:
            ttk.Radiobutton(control_frame, text=text, variable=self.info_position, value=value, 
                          command=self.refresh_current_image).pack(side=tk.LEFT, padx=2)
        
        # 颜色映射配置区域
        color_frame = ttk.LabelFrame(main_frame, text="颜色映射配置", padding="10")
        color_frame.grid(row=3, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        # 第一行：映射曲线和调色盘
        row1_frame = ttk.Frame(color_frame)
        row1_frame.pack(fill=tk.X, pady=(0, 5))
        
        # 映射曲线选择
        ttk.Label(row1_frame, text="映射曲线:").pack(side=tk.LEFT, padx=5)
        ttk.Radiobutton(row1_frame, text="线性", variable=self.mapping_curve, value="linear",
                       command=self.on_color_config_changed).pack(side=tk.LEFT, padx=2)
        ttk.Radiobutton(row1_frame, text="非线性(S曲线)", variable=self.mapping_curve, value="nonlinear",
                       command=self.on_color_config_changed).pack(side=tk.LEFT, padx=2)
        
        # 调色盘选择
        ttk.Label(row1_frame, text="调色盘:").pack(side=tk.LEFT, padx=(20, 5))
        colormaps = ['jet', 'hot', 'cool', 'rainbow', 'viridis', 'plasma', 'inferno', 'magma', 'cividis', 'turbo']
        self.colormap_combo = ttk.Combobox(row1_frame, textvariable=self.colormap_name, 
                                           values=colormaps, width=12, state='readonly')
        self.colormap_combo.pack(side=tk.LEFT, padx=5)
        self.colormap_combo.bind('<<ComboboxSelected>>', lambda e: self.on_color_config_changed())
        
        # 第二行：自定义颜色
        row2_frame = ttk.Frame(color_frame)
        row2_frame.pack(fill=tk.X)
        
        # 自定义颜色开关
        self.custom_color_check = ttk.Checkbutton(row2_frame, text="使用自定义颜色", 
                                                  variable=self.use_custom_colors,
                                                  command=self.on_color_config_changed)
        self.custom_color_check.pack(side=tk.LEFT, padx=5)
        
        # 最低温颜色选择
        ttk.Label(row2_frame, text="最低温:").pack(side=tk.LEFT, padx=(20, 5))
        self.cold_color_btn = tk.Button(row2_frame, text="  ", bg=self.cold_color, 
                                        width=3, command=lambda: self.choose_color('cold'))
        self.cold_color_btn.pack(side=tk.LEFT, padx=2)
        
        # 中间温颜色选择
        ttk.Label(row2_frame, text="中间温:").pack(side=tk.LEFT, padx=(10, 5))
        self.mid_color_btn = tk.Button(row2_frame, text="  ", bg=self.mid_color, 
                                       width=3, command=lambda: self.choose_color('mid'))
        self.mid_color_btn.pack(side=tk.LEFT, padx=2)
        
        # 最高温颜色选择
        ttk.Label(row2_frame, text="最高温:").pack(side=tk.LEFT, padx=(10, 5))
        self.hot_color_btn = tk.Button(row2_frame, text="  ", bg=self.hot_color, 
                                       width=3, command=lambda: self.choose_color('hot'))
        self.hot_color_btn.pack(side=tk.LEFT, padx=2)
        
        # 第三行：滤波强度
        row3_frame = ttk.Frame(color_frame)
        row3_frame.pack(fill=tk.X, pady=(5, 0))
        
        # 卡方滤波强度选择
        ttk.Label(row3_frame, text="卡方滤波强度:").pack(side=tk.LEFT, padx=5)
        ttk.Radiobutton(row3_frame, text="关闭", variable=self.filter_strength, value="off",
                       command=self.on_color_config_changed).pack(side=tk.LEFT, padx=2)
        ttk.Radiobutton(row3_frame, text="轻度", variable=self.filter_strength, value="light",
                       command=self.on_color_config_changed).pack(side=tk.LEFT, padx=2)
        ttk.Radiobutton(row3_frame, text="中度", variable=self.filter_strength, value="medium",
                       command=self.on_color_config_changed).pack(side=tk.LEFT, padx=2)
        ttk.Radiobutton(row3_frame, text="强度", variable=self.filter_strength, value="strong",
                       command=self.on_color_config_changed).pack(side=tk.LEFT, padx=2)
        
        # 区域降噪滤波选择
        ttk.Label(row3_frame, text="区域降噪:").pack(side=tk.LEFT, padx=(20, 5))
        ttk.Radiobutton(row3_frame, text="关闭", variable=self.area_denoise, value="off",
                       command=self.on_color_config_changed).pack(side=tk.LEFT, padx=2)
        ttk.Radiobutton(row3_frame, text="轻度", variable=self.area_denoise, value="light",
                       command=self.on_color_config_changed).pack(side=tk.LEFT, padx=2)
        ttk.Radiobutton(row3_frame, text="中度", variable=self.area_denoise, value="medium",
                       command=self.on_color_config_changed).pack(side=tk.LEFT, padx=2)
        ttk.Radiobutton(row3_frame, text="强度", variable=self.area_denoise, value="strong",
                       command=self.on_color_config_changed).pack(side=tk.LEFT, padx=2)
        
        # 第四行：可见光融合 (仅对 v2 双光文件有效)
        row4_frame = ttk.Frame(color_frame)
        row4_frame.pack(fill=tk.X, pady=(5, 0))

        self.visible_enable_chk = ttk.Checkbutton(row4_frame, text="启用可见光融合", variable=self.visible_enable,
                       command=self._on_fusion_mode_change)
        self.visible_enable_chk.pack(side=tk.LEFT, padx=5)

        ttk.Label(row4_frame, text="模式:").pack(side=tk.LEFT, padx=(15, 2))
        self.fusion_mode_rbs = []
        for _txt, _val in (("关闭", "off"), ("边缘", "edge"), ("混合", "blend")):
            _rb = ttk.Radiobutton(row4_frame, text=_txt, variable=self.fusion_mode_var, value=_val,
                                 command=self._on_fusion_mode_change)
            _rb.pack(side=tk.LEFT, padx=2)
            self.fusion_mode_rbs.append(_rb)

        # blend 模式专属: 混合度 + 伽马
        self.fr_blend_alpha = ttk.Frame(row4_frame)
        ttk.Label(self.fr_blend_alpha, text="混合度:").pack(side=tk.LEFT, padx=(15, 2))
        ttk.Scale(self.fr_blend_alpha, from_=0.0, to=1.0, variable=self.fusion_alpha,
                  orient=tk.HORIZONTAL, length=110,
                  command=lambda v: self._on_edge_param_change('alpha', v)).pack(side=tk.LEFT, padx=2)

        self.fr_blend_gamma = ttk.Frame(row4_frame)
        ttk.Label(self.fr_blend_gamma, text="伽马:").pack(side=tk.LEFT, padx=(15, 2))
        ttk.Scale(self.fr_blend_gamma, from_=0.3, to=3.0, variable=self.fusion_gamma,
                  orient=tk.HORIZONTAL, length=110,
                  command=lambda v: self._on_edge_param_change('gamma', v)).pack(side=tk.LEFT, padx=2)

        # edge 模式专属: 边缘强度 + 边缘色
        self.fr_edge_strength = ttk.Frame(row4_frame)
        ttk.Label(self.fr_edge_strength, text="边缘强度:").pack(side=tk.LEFT, padx=(15, 2))
        ttk.Scale(self.fr_edge_strength, from_=0.0, to=1.0, variable=self.fusion_edge_strength,
                  orient=tk.HORIZONTAL, length=90,
                  command=lambda v: self._on_edge_param_change('strength', v)).pack(side=tk.LEFT, padx=2)

        self.fr_edge_color = ttk.Frame(row4_frame)
        ttk.Label(self.fr_edge_color, text="边缘色:").pack(side=tk.LEFT, padx=(10, 2))
        self.edge_color_btn = tk.Button(self.fr_edge_color, text="  ", bg=self.edge_color,
                                        width=3, command=lambda: self.choose_color('edge'))
        self.edge_color_btn.pack(side=tk.LEFT, padx=2)

        # 第五行: 边缘融合细节 (阈值 + 粗细) -- edge 模式专属
        self.row5_frame = ttk.Frame(color_frame)
        self.row5_frame.pack(fill=tk.X, pady=(5, 0))

        ttk.Label(self.row5_frame, text="边缘阈值:").pack(side=tk.LEFT, padx=5)
        ttk.Scale(self.row5_frame, from_=0.0, to=1.0, variable=self.fusion_edge_thresh,
                  orient=tk.HORIZONTAL, length=160,
                  command=lambda v: self._on_edge_param_change('thresh', v)).pack(side=tk.LEFT, padx=2)

        ttk.Label(self.row5_frame, text="边缘粗细:").pack(side=tk.LEFT, padx=(20, 2))
        ttk.Scale(self.row5_frame, from_=0, to=6, variable=self.fusion_edge_width,
                  orient=tk.HORIZONTAL, length=140,
                  command=lambda v: self._on_edge_param_change('width', v)).pack(side=tk.LEFT, padx=2)

        # 初次按当前模式刷新可见性
        self._update_fusion_visibility()

        # 日志区域
        log_frame = ttk.LabelFrame(main_frame, text="操作日志", padding="10")
        log_frame.grid(row=4, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        
        self.log_text = tk.Text(log_frame, height=15, width=70)
        self.log_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        log_scrollbar = ttk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        log_scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        self.log_text.configure(yscrollcommand=log_scrollbar.set)
        
        # 配置权重使窗口可调整大小 (作用在 self 这个 Frame 上, 不动顶层 root)
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)
        main_frame.rowconfigure(1, weight=3)  # 图片管理区域权重更大
        main_frame.rowconfigure(3, weight=0)  # 颜色映射配置区域
        main_frame.rowconfigure(4, weight=1)  # 日志区域权重较小
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        
        # 存储图片数据
        self.image_list = []  # 存储图片信息 [(名称, 图片对象), ...]
        self.current_image = None  # 当前显示的图片
        self.current_photo_index = None  # 当前显示图片的索引
        self.current_filename = None  # 当前显示图片的文件名
        self.current_raw_image = None  # 当前原始图片（未添加信息条，用于刷新）
        self.current_info_text = None  # 当前信息文本（用于刷新）
        self.photo_metadata = {}  # 存储图片元数据 {index: metadata_dict}
        
        # 接收数据相关
        self.receiving_thread = None
        self.is_receiving = False
        
        # 自动获取列表相关
        self.auto_get_list_thread = None
        self.stop_auto_get_list = False
        self.list_fetched_successfully = False
        
    def log(self, message):
        """添加日志信息"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        log_message = f"[{timestamp}] {message}\n"
        self.log_text.insert(tk.END, log_message)
        self.log_text.see(tk.END)
        
    def get_available_ports(self):
        """获取可用的串口列表"""
        ports = serial.tools.list_ports.comports()
        return [(port.device, port.description, port.hwid) for port in ports]
    
    def refresh_ports(self):
        """[已禁用] 串口管理由上位机统一负责"""
        self.log("[提示] 串口由实时投屏 tab 管理, 此处不再单独刷新串口列表")

    def auto_connect_serial(self):
        """[已禁用] 串口管理由上位机统一负责"""
        return

    def on_port_selected(self, event):
        """[已禁用] 串口管理由上位机统一负责"""
        return

    def connect_to_port(self, port_device):
        """[已禁用] 串口管理由上位机统一负责"""
        return

    def disconnect_serial(self):
        """[已禁用] 串口管理由上位机统一负责"""
        # 仍需停止自动获取列表 (避免后台线程残留)
        self.stop_auto_get_list_task()

    def toggle_connection(self):
        """[已禁用] 串口管理由上位机统一负责"""
        return

    def start_auto_get_list(self):
        """开始自动获取图片列表"""
        # 重置标志
        self.stop_auto_get_list = False
        self.list_fetched_successfully = False
        
        # 启动自动获取线程
        if self.auto_get_list_thread is None or not self.auto_get_list_thread.is_alive():
            self.auto_get_list_thread = threading.Thread(target=self._auto_get_list_loop, daemon=True)
            self.auto_get_list_thread.start()
            self.log("开始自动获取图片列表...")
    
    def stop_auto_get_list_task(self):
        """停止自动获取图片列表任务"""
        self.stop_auto_get_list = True
        if self.auto_get_list_thread and self.auto_get_list_thread.is_alive():
            self.log("停止自动获取图片列表")
    
    def _auto_get_list_loop(self):
        """自动获取图片列表的循环"""
        retry_count = 0
        
        while not self.stop_auto_get_list and self.is_connected:
            retry_count += 1
            self.log(f"第 {retry_count} 次尝试获取图片列表...")
            
            # 尝试获取列表
            success = self._try_get_image_list()
            
            if success:
                self.list_fetched_successfully = True
                self.log("图片列表获取成功!")
                break
            else:
                if not self.stop_auto_get_list and self.is_connected:
                    self.log("获取失败,3秒后重试...")
                    # 等待3秒,同时检查是否需要停止
                    for _ in range(30):  # 分成30个0.1秒,便于快速响应停止信号
                        if self.stop_auto_get_list or not self.is_connected:
                            break
                        time.sleep(0.1)
    
    def _try_get_image_list(self):
        """尝试获取图片列表,返回是否成功"""
        try:
            if not self.serial_port or not self.serial_port.is_open:
                return False
            
            # 清空接收缓冲区
            self.serial_port.reset_input_buffer()
            
            # 发送check命令
            self.serial_port.write(b"check\n")
            self.log("已发送 'check' 命令")
            
            # 等待并读取响应
            response_lines = []
            start_time = time.time()
            timeout = 3.0  # 3秒超时
            json_started = False
            json_data = ""
            
            while time.time() - start_time < timeout:
                if self.stop_auto_get_list or not self.is_connected:
                    return False
                
                if self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    
                    if line:
                        response_lines.append(line)
                        self.log(f"接收: {line}")
                        
                        # 检测JSON数据开始
                        if line.startswith('{'):
                            json_started = True
                            json_data = line
                        elif json_started:
                            json_data += line
                        
                        # 检测JSON数据结束
                        if json_started and line.endswith('}'):
                            break
                
                time.sleep(0.01)
            
            # 解析JSON数据
            if json_data:
                return self.parse_image_list_json(json_data)
            else:
                self.log("未收到有效的JSON响应")
                return False
                
        except Exception as e:
            self.log(f"获取图片列表错误: {e}")
            return False
    
    def get_image_list(self):
        """手动获取图片列表"""
        if not self.is_connected:
            messagebox.showwarning("未连接", "请先连接串口设备")
            return
        
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showerror("错误", "串口未打开")
            return
        
        self.log("手动获取图片列表...")
        
        # 在新线程中执行,避免阻塞GUI
        thread = threading.Thread(target=self._manual_get_image_list_thread, daemon=True)
        thread.start()
    
    def _manual_get_image_list_thread(self):
        """手动获取图片列表的线程"""
        success = self._try_get_image_list()
        
        if not success:
            self.root.after(0, lambda: messagebox.showwarning("获取失败", "未能获取图片列表,请检查设备连接"))
    
    def parse_image_list_json(self, json_str):
        """解析图片列表JSON数据,兼容两种格式,返回是否成功"""
        try:
            data = json.loads(json_str)
            
            if 'photos' not in data:
                self.log("JSON格式错误: 缺少 'photos' 字段")
                return False
            
            photos = data['photos']
            total = data.get('total', len(photos))
            
            self.log(f"找到 {total} 张图片")
            
            # 清空当前元数据和列表
            self.photo_metadata.clear()
            self.root.after(0, lambda: self.image_listbox.delete(0, tk.END))
            
            # 解析每张图片的信息
            for photo in photos:
                index = photo.get('index')
                filename = photo.get('filename', f"photo_{index}.dat")
                size = photo.get('size', 0)
                
                # 构建元数据字典
                metadata = {
                    'index': index,
                    'filename': filename,
                    'size': size
                }
                
                # 检查是否有扩展字段(第二种格式)
                if 'mode' in photo:
                    metadata['mode'] = photo.get('mode', 'unknown')
                    metadata['dataFormat'] = photo.get('dataFormat', 'unknown')
                    metadata['temperatureMax'] = photo.get('temperatureMax', 0.0)
                    metadata['temperatureMin'] = photo.get('temperatureMin', 0.0)
                    
                    display_name = f"[{index}] {filename} ({size}B) - {metadata['mode']}"
                    self.log(f"  图片 {index}: {filename} ({size}B)")
                    self.log(f"    模式: {metadata['mode']}, 格式: {metadata['dataFormat']}")
                    self.log(f"    温度范围: {metadata['temperatureMin']:.2f}°C ~ {metadata['temperatureMax']:.2f}°C")
                else:
                    # 第一种格式(简单格式)
                    display_name = f"[{index}] {filename} ({size}B)"
                    self.log(f"  图片 {index}: {filename} ({size}B)")
                
                # 保存元数据
                self.photo_metadata[index] = metadata
                
                # 添加到GUI列表(暂时不加载实际图片)
                self.root.after(0, lambda name=display_name, idx=index: 
                    self.add_photo_item_to_list(name, idx))
            
            self.log("图片列表解析完成")
            return True
            
        except json.JSONDecodeError as e:
            self.log(f"JSON解析错误: {e}")
            self.log(f"原始数据: {json_str}")
            return False
        except Exception as e:
            self.log(f"解析图片列表错误: {e}")
            return False
    
    def add_photo_item_to_list(self, display_name, photo_index):
        """添加图片项到列表(不加载实际图片)"""
        self.image_listbox.insert(tk.END, display_name)
    
    def download_and_display_photo(self, photo_index):
        """下载并显示指定索引的图片"""
        if not self.is_connected:
            messagebox.showwarning("未连接", "请先连接串口设备")
            return
        
        # 获取元数据
        if photo_index not in self.photo_metadata:
            self.log(f"未找到图片 {photo_index} 的元数据")
            return
        
        metadata = self.photo_metadata[photo_index]
        filename = metadata['filename']
        
        self.log(f"开始下载图片: {filename}")
        
        # 在新线程中下载
        thread = threading.Thread(
            target=self._download_photo_thread, 
            args=(filename, photo_index, metadata),
            daemon=True
        )
        thread.start()
    
    def _download_photo_thread(self, filename, photo_index, metadata):
        """在线程中下载图片"""
        try:
            if not self.serial_port or not self.serial_port.is_open:
                self.log("串口未打开")
                return
            
            # 清空接收缓冲区
            self.serial_port.reset_input_buffer()
            
            # 发送download命令
            command = f"download {filename}\n"
            self.serial_port.write(command.encode('utf-8'))
            self.log(f"已发送命令: download {filename}")
            
            # 接收数据
            raw_data = self._receive_file_data(filename, metadata['size'])
            
            if raw_data is None:
                self.log("下载失败")
                self.root.after(0, lambda: messagebox.showerror("错误", "下载图片失败"))
                return
            
            # 解析数据
            self.log(f"收到 {len(raw_data)} 字节数据")
            
            # 保存原始数据
            raw_file = self.raw_dat_dir / filename
            with open(raw_file, 'wb') as f:
                f.write(raw_data)
            self.log(f"原始数据已保存到: {raw_file}")
            
            # 解析并渲染图片
            image = self._parse_thermal_data(raw_data, metadata)
            
            if image:
                # 在GUI中显示（不保存渲染后的图片）
                self.root.after(0, lambda: self._display_downloaded_image(image, filename, photo_index))
            else:
                self.log("解析图片数据失败")
                
        except Exception as e:
            self.log(f"下载图片错误: {e}")
            import traceback
            self.log(traceback.format_exc())
    
    def _receive_file_data(self, filename, expected_size):
        """接收文件数据"""
        try:
            start_time = time.time()
            timeout = 10.0  # 10秒超时
            
            # 等待开始标记
            data_started = False
            hex_data = []
            
            while time.time() - start_time < timeout:
                if self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    
                    if not line:
                        continue
                    
                    # 不输出每行接收的数据,只记录关键信息
                    
                    # 检测开始标记
                    if "BEGIN FILE DATA" in line:
                        data_started = True
                        self.log("开始接收文件数据...")
                        continue
                    
                    # 检测结束标记
                    if "END FILE DATA" in line:
                        self.log("文件数据接收完成")
                        break
                    
                    # 解析十六进制数据行
                    if data_started:
                        # 格式: "00000000: 41 20 00 00 41 10 00 00 ... |A ..A ...|"
                        if ':' in line and '|' in line:
                            try:
                                # 提取十六进制部分
                                hex_part = line.split(':')[1].split('|')[0].strip()
                                # 分割并转换每个字节
                                hex_bytes = hex_part.split()
                                for hex_byte in hex_bytes:
                                    if len(hex_byte) == 2:
                                        hex_data.append(int(hex_byte, 16))
                            except Exception:
                                pass  # 静默跳过解析错误
                
                # 不再延迟,立即继续读取
            
            if not hex_data:
                self.log("未接收到有效数据")
                return None
            
            # 转换为字节数组
            raw_data = bytes(hex_data)
            self.log(f"接收完成: {len(raw_data)} 字节")
            
            return raw_data
            
        except Exception as e:
            self.log(f"接收文件数据错误: {e}")
            return None
    
    def _parse_thermal_data(self, raw_data, metadata):
        """解析热成像数据并生成图片"""
        try:
            # ===== v2 HTPH 格式探测 (向后兼容: 不影响旧设备 v1 路径) =====
            if raw_data and len(raw_data) >= 4 and raw_data[0:4] == b"HTPH":
                return self._parse_thermal_data_v2(raw_data, metadata)

            # 判断是简约版本还是完整版本
            has_metadata = 'mode' in metadata and 'dataFormat' in metadata
            
            if has_metadata:
                # 完整版本: 包含模式信息
                return self._parse_thermal_data_full_version(raw_data, metadata)
            else:
                # 简约版本: 只有基本信息
                return self._parse_thermal_data_simple_version(raw_data, metadata)
                
        except Exception as e:
            self.log(f"解析热成像数据错误: {e}")
            return None
    
    def _parse_thermal_data_simple_version(self, raw_data, metadata):
        """解析简约版本的热成像数据(旧格式,假设全屏模式)"""
        try:
            # 读取温度范围 (前8个字节: 2个float)
            if len(raw_data) < 8:
                self.log("数据长度不足,无法读取温度信息")
                return None
            
            t_max = struct.unpack('f', raw_data[0:4])[0]
            t_min = struct.unpack('f', raw_data[4:8])[0]
            
            # 读取温度数组(从第8字节开始)
            float_data = raw_data[8:]
            num_floats = len(float_data) // 4
            
            # 解析float数组
            temps = []
            for i in range(num_floats):
                offset = i * 4
                if offset + 4 <= len(float_data):
                    temp = struct.unpack('f', float_data[offset:offset+4])[0]
                    temps.append(temp)
            
            temps_array = np.array(temps)
            
            # 简约版本默认为24x32裁剪模式
            height, width = 24, 32
            
            # 重塑数组
            try:
                temp_matrix = temps_array[:768].reshape(height, width)
            except ValueError as e:
                if len(temps_array) < 768:
                    temps_array = np.pad(temps_array, (0, 768 - len(temps_array)))
                temp_matrix = temps_array[:768].reshape(height, width)
            
            # 垂直翻转 (上下翻转)
            temp_matrix = np.flipud(temp_matrix)
            
            # 1. 边界检查：裁剪温度值防止溢出
            temp_matrix = np.clip(temp_matrix, t_min, t_max)
            
            # 归一化到0-1
            if t_max > t_min:
                normalized = (temp_matrix - t_min) / (t_max - t_min)
            else:
                normalized = np.zeros_like(temp_matrix)
            
            # 2. 应用卡方二次滤波减少噪点
            normalized = self._apply_chi_square_filter(normalized)
            
            # 2.5. 应用区域降噪滤波（针对大面积噪点）
            normalized = self._apply_area_denoise(normalized)
            
            # 3. 应用映射曲线（根据用户选择）
            if self.mapping_curve.get() == "nonlinear":
                normalized = self._apply_nonlinear_mapping(normalized)
            # 如果是linear，则保持normalized不变
            
            # 应用颜色映射（根据用户选择）
            rgb_data = self._apply_colormap(normalized)
            
            # 创建PIL图像
            image = Image.fromarray(rgb_data)
            
            # 使用高质量LANCZOS插倿放大图像以提高分辨率
            scale_factor = 20  # 提高到20倍获得更高分辨率
            new_size = (width * scale_factor, height * scale_factor)
            image = image.resize(new_size, Image.Resampling.LANCZOS)  # 使用LANCZOS高质量插值
            
            # 转换为numpy数组进行高级滤波处理
            img_array = np.array(image)
            
            # 应用优化的双边滤波保持边缘清晰度
            try:
                import cv2
                # 参数优化：d=9增加滤波范围，sigmaColor=50保持颜色平滑，sigmaSpace=50保持空间连续性
                filtered = cv2.bilateralFilter(img_array, d=9, sigmaColor=50, sigmaSpace=50)
                image = Image.fromarray(filtered)
            except ImportError:
                pass  # 静默跳过
            except Exception:
                pass  # 静默跳过
            
            # 计算平均温度
            t_avg = np.mean(temp_matrix)
            
            # 创建信息文本（不包含模式信息）
            info_text = f"最高温: {t_max:.1f}°C  |  最低温: {t_min:.1f}°C  |  平均温: {t_avg:.1f}°C"
            
            # 保存原始图片和信息文本（用于刷新）
            self.current_raw_image = image
            self.current_info_text = info_text
            # v1 旧设备无可见光数据, 清空避免串扰
            self.current_visible_image = None
            # 保存温度数据（用于重新渲染和鼠标悬浮）
            self.current_temp_matrix = temp_matrix
            self.current_t_min = t_min
            self.current_t_max = t_max
            # 保存热成像图片尺寸（放大后的尺寸）
            self.thermal_image_size = image.size  # (width, height)
            
            # 添加信息条和图例
            new_image = self._add_info_bar_and_legend(image, info_text)
            
            self.log(f"简约版本解析完成: {t_min:.1f}°C ~ {t_max:.1f}°C (平均: {t_avg:.1f}°C)")
            return new_image
            
        except Exception as e:
            self.log(f"解析简约版本数据错误: {e}")
            import traceback
            self.log(traceback.format_exc())
            return None
    
    def _parse_thermal_data_full_version(self, raw_data, metadata):
        """解析完整版本的热成像数据(新格式,包含模式标志)"""
        try:
            # 读取温度范围 (前8个字节: 2个float)
            if len(raw_data) < 9:  # 至少需要8字节温度 + 1字节标志
                self.log("数据长度不足,无法读取完整版本信息")
                return None
            
            t_max = struct.unpack('f', raw_data[0:4])[0]
            t_min = struct.unpack('f', raw_data[4:8])[0]
            
            # 读取模式标志 (第9个字节: 1个bool)
            is_full_screen = struct.unpack('?', raw_data[8:9])[0]
            
            # 读取温度数组(从第9字节开始)
            float_data = raw_data[9:]
            num_floats = len(float_data) // 4
            
            # 解析float数组
            temps = []
            for i in range(num_floats):
                offset = i * 4
                if offset + 4 <= len(float_data):
                    temp = struct.unpack('f', float_data[offset:offset+4])[0]
                    temps.append(temp)
            
            temps_array = np.array(temps)
            
            # 根据模式标志和元数据确定格式
            if is_full_screen:
                height, width = 24, 32
                expected_points = 768
            else:
                height, width = 32, 32
                expected_points = 1024
            
            # 重塑数组
            try:
                if len(temps_array) >= expected_points:
                    temp_matrix = temps_array[:expected_points].reshape(height, width)
                else:
                    temps_array = np.pad(temps_array, (0, expected_points - len(temps_array)))
                    temp_matrix = temps_array.reshape(height, width)
            except ValueError:
                return None
            
            # 垂直翻转 (上下翻转)
            temp_matrix = np.flipud(temp_matrix)
            
            # 1. 边界检查：裁剪温度值防止溢出
            temp_matrix = np.clip(temp_matrix, t_min, t_max)
            
            # 归一化到0-1
            if t_max > t_min:
                normalized = (temp_matrix - t_min) / (t_max - t_min)
            else:
                normalized = np.zeros_like(temp_matrix)
            
            # 2. 应用卡方二次滤波减少噪点
            normalized = self._apply_chi_square_filter(normalized)
            
            # 2.5. 应用区域降噪滤波（针对大面积噪点）
            normalized = self._apply_area_denoise(normalized)
            
            # 3. 应用映射曲线（根据用户选择）
            if self.mapping_curve.get() == "nonlinear":
                normalized = self._apply_nonlinear_mapping(normalized)
            # 如果是linear，则保持normalized不变
            
            # 应用颜色映射（根据用户选择）
            rgb_data = self._apply_colormap(normalized)
            
            # 创建PIL图像
            image = Image.fromarray(rgb_data)
            
            # 使用高质量LANCZOS插倿放大图像以提高分辨率
            scale_factor = 20  # 提高到20倍获得更高分辨率
            new_size = (width * scale_factor, height * scale_factor)
            image = image.resize(new_size, Image.Resampling.LANCZOS)  # 使用LANCZOS高质量插值
            
            # 转换为numpy数组进行高级滤波处理
            img_array = np.array(image)
            
            # 应用优化的双边滤波保持边缘清晰度
            try:
                import cv2
                # 参数优化：d=9增加滤波范围，sigmaColor=50保持颜色平滑，sigmaSpace=50保持空间连续性
                filtered = cv2.bilateralFilter(img_array, d=9, sigmaColor=50, sigmaSpace=50)
                image = Image.fromarray(filtered)
            except ImportError:
                pass  # 静默跳过
            except Exception:
                pass  # 静默跳过
            
            # 计算平均温度
            t_avg = np.mean(temp_matrix)
            
            # 获取模式信息用于日志
            mode_str = "全屏" if is_full_screen else "方形"
            
            # 创建信息文本（不包含模式信息）
            info_text = f"最高温: {t_max:.1f}°C  |  最低温: {t_min:.1f}°C  |  平均温: {t_avg:.1f}°C"
            
            # 保存原始图片和信息文本（用于刷新）
            self.current_raw_image = image
            self.current_info_text = info_text
            # v1 旧设备无可见光数据, 清空避免串扰
            self.current_visible_image = None
            # 保存温度数据（用于重新渲染和鼠标悬浮）
            self.current_temp_matrix = temp_matrix
            self.current_t_min = t_min
            self.current_t_max = t_max
            # 保存热成像图片尺寸（放大后的尺寸）
            self.thermal_image_size = image.size  # (width, height)
            
            # 添加信息条和图例
            new_image = self._add_info_bar_and_legend(image, info_text)
            
            self.log(f"完整版本解析完成: {mode_str}模式 {t_min:.1f}°C ~ {t_max:.1f}°C (平均: {t_avg:.1f}°C)")
            return new_image
            
        except Exception as e:
            self.log(f"解析完整版本数据错误: {e}")
            return None

    # ====================================================================
    # 可见光与热成像融合 (仅作用于 20× 放大后的纯热成像内容图)
    # 输入: thermal_pil = 纯热成像内容图 (尚未添加信息条/图例)
    #       visible_pil = 可见光 RGB Image (设备端原始分辨率)
    # 返回: 与 thermal_pil 同尺寸的融合 RGB 图
    # 严格对齐: 因为只融合在热成像内容区, 信息条与图例由 _add_info_bar_and_legend
    # 后续叠加, 永远不会错位.
    # ====================================================================
    def _fuse_thermal_with_visible(self, thermal_pil, visible_pil):
        if visible_pil is None:
            return thermal_pil
        if not self.visible_enable.get():
            return thermal_pil
        mode = self.fusion_mode_var.get()
        if mode == "off":
            return thermal_pil
        try:
            tw, th = thermal_pil.size
            # 缩放可见光到热成像内容区尺寸
            vis_resized = visible_pil.resize((tw, th), Image.Resampling.LANCZOS)
            vis_arr = np.array(vis_resized, dtype=np.float32) / 255.0  # 0~1 RGB

            # 伽马校正
            gamma = float(self.fusion_gamma.get())
            if gamma <= 0:
                gamma = 1.0
            vis_arr = np.clip(vis_arr, 0.0, 1.0) ** (1.0 / gamma)

            therm_arr = np.array(thermal_pil, dtype=np.float32) / 255.0

            if mode == "blend":
                alpha = float(self.fusion_alpha.get())
                alpha = max(0.0, min(1.0, alpha))
                out = therm_arr * (1.0 - alpha) + vis_arr * alpha
            elif mode == "edge":
                # 关键改进: 在可见光原始分辨率 (120x160 等) 做 Sobel,
                # 否则上采样到热成像尺寸后再算梯度, 边缘会被插值模糊.
                # 流程: 原图灰度 → Sobel L1 → 阈值化二值化 → 上采样到热成像尺寸 (NEAREST 保锐) → 膨胀加粗 → 颜色叠加
                tw, th = thermal_pil.size
                src_arr = np.array(visible_pil)   # 原始分辨率 RGB uint8
                try:
                    import cv2
                    gray0 = cv2.cvtColor(src_arr, cv2.COLOR_RGB2GRAY)
                    gx0 = cv2.Sobel(gray0, cv2.CV_32F, 1, 0, ksize=3)
                    gy0 = cv2.Sobel(gray0, cv2.CV_32F, 0, 1, ksize=3)
                    has_cv2 = True
                except Exception:
                    gray0 = src_arr.astype(np.float32).mean(axis=2)
                    gx0 = np.zeros_like(gray0); gy0 = np.zeros_like(gray0)
                    gx0[1:-1, 1:-1] = (
                        -gray0[:-2, :-2] + gray0[:-2, 2:]
                        - 2*gray0[1:-1, :-2] + 2*gray0[1:-1, 2:]
                        - gray0[2:, :-2] + gray0[2:, 2:]
                    )
                    gy0[1:-1, 1:-1] = (
                        -gray0[:-2, :-2] - 2*gray0[:-2, 1:-1] - gray0[:-2, 2:]
                        + gray0[2:, :-2] + 2*gray0[2:, 1:-1] + gray0[2:, 2:]
                    )
                    has_cv2 = False

                # L1 范数 (设备端一致), 范围 0~1020
                mag0 = np.abs(gx0) + np.abs(gy0)
                # 阈值化 → 二值边缘 mask (锐利)
                thresh = float(self.fusion_edge_thresh.get())
                thresh = max(0.0, min(1.0, thresh))
                thresh_val = thresh * 1020.0
                mask0 = (mag0 > thresh_val).astype(np.float32)

                # 上采样到热成像尺寸: NEAREST 保锐 (二值边缘不会因插值变虚)
                if has_cv2:
                    mask = cv2.resize(mask0, (tw, th), interpolation=cv2.INTER_NEAREST)
                else:
                    # PIL 邻近: 反过来用 PIL.Image 做 NEAREST resize
                    mask_img = Image.fromarray((mask0 * 255).astype(np.uint8))
                    mask_img = mask_img.resize((tw, th), Image.Resampling.NEAREST)
                    mask = np.array(mask_img).astype(np.float32) / 255.0

                # 膨胀加粗 (上位机增强)
                width = int(self.fusion_edge_width.get())
                width = max(0, min(6, width))
                if width > 0:
                    if has_cv2:
                        ksize = 2 * width + 1
                        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksize, ksize))
                        mask = cv2.dilate(mask, kernel, iterations=1)
                    else:
                        try:
                            from scipy.ndimage import maximum_filter
                            mask = maximum_filter(mask, size=2 * width + 1)
                        except Exception:
                            pass

                strength = float(self.fusion_edge_strength.get())
                strength = max(0.0, min(1.0, strength))
                try:
                    er, eg, eb = self._hex_to_rgb(self.edge_color)
                except Exception:
                    er, eg, eb = (255, 255, 255)
                er_n, eg_n, eb_n = er / 255.0, eg / 255.0, eb / 255.0
                weight = mask * strength  # [H,W], 锐利二值×强度
                w3 = np.dstack([weight, weight, weight])
                color_layer = np.dstack([
                    np.full_like(mask, er_n),
                    np.full_like(mask, eg_n),
                    np.full_like(mask, eb_n),
                ])
                out = therm_arr * (1.0 - w3) + color_layer * w3
                out = np.clip(out, 0.0, 1.0)
            else:
                return thermal_pil

            out_u8 = np.clip(out * 255.0, 0, 255).astype(np.uint8)
            return Image.fromarray(out_u8, mode='RGB')
        except Exception as fe:
            self.log(f"融合失败: {fe}")
            return thermal_pil

    # ====================================================================
    # v2 HTPH 双光格式解析: 热成像 + 可见光 (向后兼容旧设备时不走此路径)
    # 文件布局:
    #   [0..3]   magic 'HTPH'
    #   [4]      version (uint8, 当前=2)
    #   [5]      flags  (bit0=full_screen, bit1=has_visible, bit2-3=fusion_mode)
    #   [6..9]   T_max (float LE)
    #   [10..13] T_min (float LE)
    #   [14..15] visible_w (uint16 LE)   仅当 has_visible
    #   [16..17] visible_h (uint16 LE)   仅当 has_visible
    #   thermal: 768 (full_screen=24x32) 或 1024 (square=32x32) float LE
    #   visible: visible_w*visible_h 个 uint16 RGB565 LE  (仅当 has_visible)
    # ====================================================================
    def _parse_thermal_data_v2(self, raw_data, metadata):
        try:
            if len(raw_data) < 14:
                self.log("v2 数据头长度不足")
                return None

            version = raw_data[4]
            flags = raw_data[5]
            is_full_screen = (flags & 0x01) != 0
            has_visible    = (flags & 0x02) != 0
            fusion_mode    = (flags >> 2) & 0x03

            t_max = struct.unpack('<f', raw_data[6:10])[0]
            t_min = struct.unpack('<f', raw_data[10:14])[0]

            cursor = 14
            visible_w = visible_h = 0
            if has_visible:
                if len(raw_data) < cursor + 4:
                    self.log("v2 标记 has_visible 但缺少宽高字段")
                    return None
                visible_w = struct.unpack('<H', raw_data[cursor:cursor+2])[0]
                visible_h = struct.unpack('<H', raw_data[cursor+2:cursor+4])[0]
                cursor += 4

            # 热成像浮点数据
            expected_points = 768 if is_full_screen else 1024
            thermal_bytes = expected_points * 4
            if len(raw_data) < cursor + thermal_bytes:
                self.log(f"v2 热成像数据不足: 需要 {thermal_bytes} 实际剩余 {len(raw_data)-cursor}")
                return None

            thermal_buf = raw_data[cursor:cursor+thermal_bytes]
            cursor += thermal_bytes
            temps_array = np.frombuffer(thermal_buf, dtype='<f4').copy()

            # 可见光数据
            visible_image = None
            if has_visible and visible_w > 0 and visible_h > 0:
                vis_len = visible_w * visible_h * 2
                if len(raw_data) >= cursor + vis_len:
                    vis_buf = raw_data[cursor:cursor+vis_len]
                    try:
                        # RGB565 LE → RGB888
                        arr = np.frombuffer(vis_buf, dtype='<u2').reshape(visible_h, visible_w)
                        r = ((arr >> 11) & 0x1F).astype(np.uint8)
                        g = ((arr >> 5)  & 0x3F).astype(np.uint8)
                        b = (arr        & 0x1F).astype(np.uint8)
                        # 5/6 bit → 8 bit 线性扩展
                        r = (r << 3) | (r >> 2)
                        g = (g << 2) | (g >> 4)
                        b = (b << 3) | (b >> 2)
                        rgb888 = np.dstack([r, g, b])
                        visible_image = Image.fromarray(rgb888, mode='RGB')
                        # 设备保存方向需要顺时针 90° 才与热成像视角一致
                        visible_image = visible_image.transpose(Image.ROTATE_270)
                    except Exception as ve:
                        self.log(f"可见光解码失败: {ve}")
                        visible_image = None
                else:
                    self.log(f"v2 可见光数据不足: 需要 {vis_len} 实际剩余 {len(raw_data)-cursor}")

            # ===== 热成像渲染 (镜像 full_version 流程) =====
            height, width = (24, 32) if is_full_screen else (32, 32)
            if len(temps_array) < expected_points:
                temps_array = np.pad(temps_array, (0, expected_points - len(temps_array)))
            temp_matrix = temps_array[:expected_points].reshape(height, width)
            temp_matrix = np.flipud(temp_matrix)
            temp_matrix = np.clip(temp_matrix, t_min, t_max)
            if t_max > t_min:
                normalized = (temp_matrix - t_min) / (t_max - t_min)
            else:
                normalized = np.zeros_like(temp_matrix)
            normalized = self._apply_chi_square_filter(normalized)
            normalized = self._apply_area_denoise(normalized)
            if self.mapping_curve.get() == "nonlinear":
                normalized = self._apply_nonlinear_mapping(normalized)
            rgb_data = self._apply_colormap(normalized)
            thermal_pil = Image.fromarray(rgb_data)
            scale_factor = 20
            thermal_pil = thermal_pil.resize((width * scale_factor, height * scale_factor),
                                             Image.Resampling.LANCZOS)
            img_array = np.array(thermal_pil)
            try:
                import cv2
                filtered = cv2.bilateralFilter(img_array, d=9, sigmaColor=50, sigmaSpace=50)
                thermal_pil = Image.fromarray(filtered)
            except Exception:
                pass

            t_avg = float(np.mean(temp_matrix))
            mode_str = "全屏" if is_full_screen else "方形"
            fusion_name = {0: "OFF", 1: "EDGE", 2: "BLEND"}.get(fusion_mode, f"M{fusion_mode}")
            info_text = f"最高温: {t_max:.1f}°C  |  最低温: {t_min:.1f}°C  |  平均温: {t_avg:.1f}°C"

            # 保存原始热成像图(融合前)与刷新所需状态
            self.current_visible_image = visible_image  # 可能为 None
            self.current_raw_image = thermal_pil
            self.current_info_text = info_text
            self.current_temp_matrix = temp_matrix
            self.current_t_min = t_min
            self.current_t_max = t_max
            self.thermal_image_size = thermal_pil.size

            # ===== 在加信息条之前与可见光融合 (保证对齐热成像内容区, 信息条与图例后置不错位) =====
            fused_thermal = self._fuse_thermal_with_visible(thermal_pil, visible_image)

            # 添加信息条与图例
            composed = self._add_info_bar_and_legend(fused_thermal, info_text)

            # ===== 同时保存独立可见光 PNG (用户后续可单独查看) =====
            if visible_image is not None:
                try:
                    filename = metadata.get('filename', 'photo.dat')
                    stem = filename.rsplit('.', 1)[0]
                    vis_path = self.visible_png_dir / f"{stem}_visible.png"
                    visible_image.save(vis_path)
                    self.log(f"可见光图已保存: {vis_path} ({visible_w}x{visible_h})")
                except Exception as se:
                    self.log(f"可见光保存失败: {se}")

            self.log(f"v2(HTPH) 解析完成: {mode_str}模式 设备保存fusion={fusion_name} "
                     f"{t_min:.1f}°C~{t_max:.1f}°C avg={t_avg:.1f}°C "
                     f"visible={'YES '+str(visible_w)+'x'+str(visible_h) if has_visible else 'NO'}")
            return composed

        except Exception as e:
            self.log(f"解析 v2 HTPH 数据错误: {e}")
            import traceback
            self.log(traceback.format_exc())
            return None

    
    def _apply_chi_square_filter(self, normalized_data, kernel_size=None, threshold=None):
        """应用卡方二次滤波减少噪点
        
        Args:
            normalized_data: 归一化后的温度数据 (0-1)
            kernel_size: 滤波核大小（自动根据强度设置）
            threshold: 卡方阈值（自动根据强度设置）
            
        Returns:
            滤波后的数据
        """
        # 根据用户选择的滤波强度设置参数
        strength = self.filter_strength.get()
        
        if strength == "off":
            # 关闭滤波，直接返回原数据
            return normalized_data
        elif strength == "light":
            # 轻度滤波
            kernel_size = kernel_size or 3
            threshold = threshold or 0.8
        elif strength == "medium":
            # 中度滤波（默认）
            kernel_size = kernel_size or 3
            threshold = threshold or 0.5
        elif strength == "strong":
            # 强度滤波
            kernel_size = kernel_size or 5
            threshold = threshold or 0.3
        else:
            # 默认中度
            kernel_size = kernel_size or 3
            threshold = threshold or 0.5
        
        try:
            from scipy.ndimage import uniform_filter
            
            # 计算局部均值
            local_mean = uniform_filter(normalized_data, size=kernel_size, mode='reflect')
            
            # 计算局部方差
            local_variance = uniform_filter(normalized_data**2, size=kernel_size, mode='reflect') - local_mean**2
            local_variance = np.maximum(local_variance, 1e-10)  # 防止除零
            
            # 卡方检验：计算偏差
            deviation = np.abs(normalized_data - local_mean)
            chi_square = (deviation**2) / local_variance
            
            # 根据卡方值决定是否保留原值或使用均值
            # 卡方值小（相似区域）使用滤波后的值，卡方值大（边缘）保留原值
            weight = np.exp(-chi_square / threshold)
            filtered_data = weight * local_mean + (1 - weight) * normalized_data
            
            return filtered_data
            
        except ImportError:
            # 如果scipy不可用，使用简单的均值滤波
            try:
                import cv2
                # 使用OpenCV的高斯滤波作为替代
                kernel = kernel_size
                filtered = cv2.GaussianBlur(normalized_data, (kernel, kernel), 0)
                return filtered
            except:
                # 如果cv2也不可用，返回原数据
                return normalized_data
        except Exception as e:
            self.log(f"卡方滤波错误: {e}")
            return normalized_data
    
    def _apply_area_denoise(self, normalized_data):
        """应用区域降噪滤波（针对大面积噪点）
        
        使用形态学滤波和中值滤波组合来减少区域噪点
        
        Args:
            normalized_data: 归一化后的温度数据 (0-1)
            
        Returns:
            降噪后的数据
        """
        strength = self.area_denoise.get()
        
        if strength == "off":
            return normalized_data
        
        # 根据强度设置参数（进一步减小以获得更温和的效果）
        if strength == "light":
            median_size = 3
            morph_size = 0  # 不使用形态学滤波
        elif strength == "medium":
            median_size = 3
            morph_size = 2  # 轻微的形态学处理
        elif strength == "strong":
            median_size = 3  # 只使用小核中值滤波
            morph_size = 3
        else:
            return normalized_data
        
        try:
            # 优先使用scipy的中值滤波
            from scipy.ndimage import median_filter, grey_opening, grey_closing
            
            # 步骤1: 中值滤波去除孤立噪点
            denoised = median_filter(normalized_data, size=median_size)
            
            # 步骤2和3: 形态学处理（仅当morph_size > 0时）
            if morph_size > 0:
                # 步骤2: 形态学开运算（先腐蚀后膨胀）去除小的亮噪点
                denoised = grey_opening(denoised, size=morph_size)
                
                # 步骤3: 形态学闭运算（先膨胀后腐蚀）填充小的暗噪点
                denoised = grey_closing(denoised, size=morph_size)
            
            return denoised
            
        except ImportError:
            # 如果scipy不可用，尝试使用cv2
            try:
                import cv2
                # 转换为uint8以使用OpenCV
                temp_data = (normalized_data * 255).astype(np.uint8)
                
                # 中值滤波
                denoised = cv2.medianBlur(temp_data, median_size)
                
                # 形态学操作（仅当morph_size > 0时）
                if morph_size > 0:
                    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (morph_size, morph_size))
                    denoised = cv2.morphologyEx(denoised, cv2.MORPH_OPEN, kernel)
                    denoised = cv2.morphologyEx(denoised, cv2.MORPH_CLOSE, kernel)
                
                # 转换回0-1范围
                return denoised.astype(np.float64) / 255.0
                
            except:
                # 两者都不可用，返回原数据
                return normalized_data
                
        except Exception as e:
            self.log(f"区域降噪错误: {e}")
            return normalized_data
    
    def _apply_nonlinear_mapping(self, normalized_data, power=2.5):
        """应用非线性颜色映射（S曲线）
        
        使用S曲线映射使最高温和最低温附近的颜色更相近，中间区域差异更大
        
        Args:
            normalized_data: 归一化后的数据 (0-1)
            power: 控制曲线陡峭程度，越大中间差异越明显
            
        Returns:
            非线性映射后的数据
        """
        # 使用S型曲线（平滑阶跃函数）
        # 在0和1附近变化缓慢，在0.5附近变化快速
        # 公式: f(x) = x^power / (x^power + (1-x)^power)
        
        # 避免除零和数值不稳定
        normalized_data = np.clip(normalized_data, 1e-10, 1 - 1e-10)
        
        # 应用S曲线映射
        numerator = np.power(normalized_data, power)
        denominator = numerator + np.power(1 - normalized_data, power)
        s_curve = numerator / denominator
        
        return s_curve
    
    def _apply_colormap(self, normalized_data):
        """应用颜色映射（支持预设和自定义）
        
        Args:
            normalized_data: 归一化后的数据 (0-1)
            
        Returns:
            RGB图像数据 (0-255)
        """
        if self.use_custom_colors.get():
            # 使用自定义颜色
            return self._create_custom_colormap(normalized_data)
        else:
            # 使用预设调色盘
            colormap_name = self.colormap_name.get()
            try:
                colormap = cm.get_cmap(colormap_name)
                # 裁剪数据到高饱和度范围 (0.05-0.95)，避免两端变黑
                # 将 [0, 1] 映射到 [0.05, 0.95]
                clipped_data = normalized_data * 0.9 + 0.05
                colored = colormap(clipped_data)
                rgb_data = (colored[:, :, :3] * 255).astype(np.uint8)
                return rgb_data
            except:
                # 如果调色盘不存在，使用jet作为后备
                colormap = cm.get_cmap('jet')
                clipped_data = normalized_data * 0.9 + 0.05
                colored = colormap(clipped_data)
                rgb_data = (colored[:, :, :3] * 255).astype(np.uint8)
                return rgb_data
    
    def _create_custom_colormap(self, normalized_data):
        """创建自定义颜色映射（三色渐变）
        
        Args:
            normalized_data: 归一化后的数据 (0-1)
            
        Returns:
            RGB图像数据 (0-255)
        """
        # 解析颜色
        cold_rgb = np.array(self._hex_to_rgb(self.cold_color))
        mid_rgb = np.array(self._hex_to_rgb(self.mid_color))
        hot_rgb = np.array(self._hex_to_rgb(self.hot_color))
        
        # 创建三色渐变
        height, width = normalized_data.shape
        rgb_data = np.zeros((height, width, 3), dtype=np.uint8)
        
        # 使用分段线性插值：0-0.5为cold到mid，0.5-1为mid到hot
        mask_low = normalized_data <= 0.5
        mask_high = normalized_data > 0.5
        
        for i in range(3):  # R, G, B
            # 低温区域 (0-0.5): cold -> mid
            t_low = normalized_data[mask_low] * 2  # 映射到0-1
            rgb_data[mask_low, i] = (cold_rgb[i] * (1 - t_low) + mid_rgb[i] * t_low).astype(np.uint8)
            
            # 高温区域 (0.5-1): mid -> hot
            t_high = (normalized_data[mask_high] - 0.5) * 2  # 映射到0-1
            rgb_data[mask_high, i] = (mid_rgb[i] * (1 - t_high) + hot_rgb[i] * t_high).astype(np.uint8)
        
        return rgb_data
    
    def _hex_to_rgb(self, hex_color):
        """将十六进制颜色转换为RGB
        
        Args:
            hex_color: 十六进制颜色字符串，如 "#ff0000"
            
        Returns:
            (R, G, B) 元组
        """
        hex_color = hex_color.lstrip('#')
        return tuple(int(hex_color[i:i+2], 16) for i in (0, 2, 4))
    
    def choose_color(self, color_type):
        """打开颜色选择器
        
        Args:
            color_type: 'hot', 'mid' 或 'cold'
        """
        from tkinter import colorchooser
        
        # 获取当前颜色
        if color_type == 'hot':
            current_color = self.hot_color
            title = "选择最高温颜色"
        elif color_type == 'mid':
            current_color = self.mid_color
            title = "选择中间温颜色"
        elif color_type == 'edge':
            current_color = self.edge_color
            title = "选择可见光边缘融合颜色"
        else:
            current_color = self.cold_color
            title = "选择最低温颜色"
        
        # 打开颜色选择器
        color = colorchooser.askcolor(color=current_color, title=title)
        
        if color[1]:  # color[1]是十六进制颜色值
            if color_type == 'hot':
                self.hot_color = color[1]
                self.hot_color_btn.config(bg=color[1])
            elif color_type == 'mid':
                self.mid_color = color[1]
                self.mid_color_btn.config(bg=color[1])
            elif color_type == 'edge':
                self.edge_color = color[1]
                self.edge_color_btn.config(bg=color[1])
            else:
                self.cold_color = color[1]
                self.cold_color_btn.config(bg=color[1])
            
            # 刷新当前图片
            self.on_color_config_changed()
    
    def _add_info_bar(self, image, info_text):
        """在图片周围添加白底黑字的信息条
        
        Args:
            image: PIL Image对象
            info_text: 要显示的信息文本（格式：最高温: XX°C | 最低温: XX°C | 平均温: XX°C）
            
        Returns:
            添加了信息条的新图片
        """
        from PIL import ImageDraw
        
        # 获取当前图片尺寸
        img_width, img_height = image.size
        
        # 获取用户选择的位置
        position = self.info_position.get()
        
        # 解析信息文本，提取各部分
        parts = [part.strip() for part in info_text.split('|')]
        
        if position == "bottom" or position == "top":
            # 上下方：分两行显示（去掉模式信息）
            bar_height = 50  # 增加高度以容纳两行文字
            
            if position == "bottom":
                new_image = Image.new('RGB', (img_width, img_height + bar_height), color=(255, 255, 255))
                new_image.paste(image, (0, 0))
                text_y_base = img_height
            else:  # top
                new_image = Image.new('RGB', (img_width, img_height + bar_height), color=(255, 255, 255))
                new_image.paste(image, (0, bar_height))
                text_y_base = 0
            
            draw = ImageDraw.Draw(new_image)
            
            # 第一行：温度信息（显示全部3个温度）
            if len(parts) >= 3:
                line1 = f"{parts[0]}  |  {parts[1]}  |  {parts[2]}"
            else:
                line1 = info_text
            
            # 绘制居中文字
            try:
                if self.font:
                    bbox = draw.textbbox((0, 0), line1, font=self.font)
                    text_width = bbox[2] - bbox[0]
                    text_x = (img_width - text_width) // 2
                    draw.text((text_x, text_y_base + 15), line1, fill=(0, 0, 0), font=self.font)
                else:
                    bbox = draw.textbbox((0, 0), line1)
                    text_width = bbox[2] - bbox[0]
                    text_x = (img_width - text_width) // 2
                    draw.text((text_x, text_y_base + 15), line1, fill=(0, 0, 0))
            except:
                if self.font:
                    draw.text((10, text_y_base + 15), line1, fill=(0, 0, 0), font=self.font)
                else:
                    draw.text((10, text_y_base + 15), line1, fill=(0, 0, 0))
                
        elif position == "left" or position == "right":
            # 左右侧：标题和数值分开显示
            bar_width = 50  # 侧边信息条宽度（减小到100像素）
            line_height = 20  # 每行高度
            
            if position == "left":
                new_image = Image.new('RGB', (img_width + bar_width, img_height), color=(255, 255, 255))
                new_image.paste(image, (bar_width, 0))
                text_x = 5
            else:  # right
                new_image = Image.new('RGB', (img_width + bar_width, img_height), color=(255, 255, 255))
                new_image.paste(image, (0, 0))
                text_x = img_width + 5
            
            draw = ImageDraw.Draw(new_image)
            
            # 提取温度值
            lines = []
            for part in parts:
                if ':' in part:
                    label, value = part.split(':', 1)
                    lines.append(label.strip() + ':')
                    lines.append(value.strip())
            
            # 计算总文本高度
            total_height = len(lines) * line_height
            start_y = (img_height - total_height) // 2
            
            # 逐行绘制
            for i, line in enumerate(lines):
                text_y = start_y + i * line_height
                if self.font:
                    draw.text((text_x, text_y), line, fill=(0, 0, 0), font=self.font)
                else:
                    draw.text((text_x, text_y), line, fill=(0, 0, 0))
        else:
            # 默认底部
            new_image = image
        
        return new_image
    
    def _display_downloaded_image(self, image, filename, photo_index):
        """显示下载的图片"""
        # 切换图片时清除标记
        self.temp_markers.clear()
        # 保存原始渲染图片（用于后续保存和刷新）
        self.current_original_image = image
        self.current_filename = filename
        self.current_photo_index = photo_index

        # 加载新图后, 根据是否有可见光画面刷新融合控件锁定状态 (主线程上下文)
        self._update_fusion_visibility()

        # 在画布上显示
        self.display_image(image)
        self.log(f"图片 {filename} 显示完成（未保存）")
    
    def refresh_current_image(self):
        """刷新当前显示的图片（当信息条位置或颜色配置改变时）"""
        # 每次刷新前都重新评估融合控件可见性 (current_visible_image 可能已变化)
        self._update_fusion_visibility()
        if self.current_raw_image is not None and self.current_info_text is not None:
            # 若是 v2 双光帧, 先与可见光融合, 再叠加 marker 与信息条
            fused = self._fuse_thermal_with_visible(self.current_raw_image, self.current_visible_image)
            # 在融合后的热成像图上绘制标记
            marked_image = self._draw_markers_on_image(fused)
            # 重新添加信息条和图例
            new_image = self._add_info_bar_and_legend(marked_image, self.current_info_text)
            # 更新 current_original_image 用于保存
            self.current_original_image = new_image
            # 在画布上显示
            self.display_image(new_image)
            self.log(f"已刷新图片显示")
    
    def on_color_config_changed(self):
        """颜色配置改变时的回调"""
        # 如果有温度数据，重新渲染图片
        if self.current_temp_matrix is not None:
            self.rerender_with_new_colormap()
        else:
            # 否则只刷新当前显示
            self.refresh_current_image()

    def _update_fusion_visibility(self):
        """根据当前融合模式 / 启用状态 / 是否有可见光数据, 控制控件显隐与启用状态"""
        if not hasattr(self, 'fr_blend_alpha'):
            return  # 控件尚未构建完毕

        # 1) 检测当前是否存在可见光画面 (无 → 强制锁定 off 并禁用)
        has_visible = self.current_visible_image is not None
        if not has_visible:
            # 强制关闭, 避免使用陈旧/缺失的可见光数据
            if self.visible_enable.get():
                self.visible_enable.set(False)
            if self.fusion_mode_var.get() != "off":
                self.fusion_mode_var.set("off")
            try:
                self.visible_enable_chk.state(['disabled'])
                for rb in self.fusion_mode_rbs:
                    rb.state(['disabled'])
            except Exception:
                pass
        else:
            try:
                self.visible_enable_chk.state(['!disabled'])
                for rb in self.fusion_mode_rbs:
                    rb.state(['!disabled'])
            except Exception:
                pass

        # 2) 计算实际生效模式
        enabled = self.visible_enable.get() and has_visible
        mode = self.fusion_mode_var.get() if enabled else "off"

        # 3) 先把所有可变控件 forget, 再按需 pack (不依赖 winfo_ismapped, 启动时也生效)
        for fr in (self.fr_blend_alpha, self.fr_blend_gamma,
                   self.fr_edge_strength, self.fr_edge_color):
            fr.pack_forget()
        self.row5_frame.pack_forget()

        if mode == "blend":
            self.fr_blend_alpha.pack(side=tk.LEFT)
            self.fr_blend_gamma.pack(side=tk.LEFT)
        elif mode == "edge":
            self.fr_edge_strength.pack(side=tk.LEFT)
            self.fr_edge_color.pack(side=tk.LEFT)
            self.row5_frame.pack(fill=tk.X, pady=(5, 0))
        # else off: 全部保持隐藏

    def _on_fusion_mode_change(self):
        """融合模式 / 启用复选框变化: 更新控件可见性 + 触发重渲"""
        self._update_fusion_visibility()
        self.on_color_config_changed()

    def _on_edge_param_change(self, kind, v):
        """边缘融合滑块回调: 去抖打印当前数值, 然后触发重渲"""
        import time
        now = time.time()
        if now - self._last_edge_log_t > 0.15:
            self._last_edge_log_t = now
            try:
                if kind == 'thresh':
                    f = float(v)
                    self.log(f"[融合] 边缘阈值: {f:.3f} (|gx|+|gy| 截断值 {f*1020:.0f}/1020)")
                elif kind == 'width':
                    self.log(f"[融合] 边缘粗细: {int(float(v))} 像素膨胀")
                elif kind == 'strength':
                    self.log(f"[融合] 边缘强度: {float(v):.2f}")
                elif kind == 'alpha':
                    self.log(f"[融合] 混合度 α: {float(v):.2f}")
                elif kind == 'gamma':
                    self.log(f"[融合] 伽马: {float(v):.2f}")
            except Exception:
                pass
        self.on_color_config_changed()
    def rerender_with_new_colormap(self):
        """使用新的颜色映射重新渲染图片"""
        if self.current_temp_matrix is None:
            return
        
        try:
            temp_matrix = self.current_temp_matrix
            t_min = self.current_t_min
            t_max = self.current_t_max
            
            # 归一化到0-1
            if t_max > t_min:
                normalized = (temp_matrix - t_min) / (t_max - t_min)
            else:
                normalized = np.zeros_like(temp_matrix)
            
            # 应用卡方二次滤波减少噪点
            normalized = self._apply_chi_square_filter(normalized)
            
            # 应用区域降噪滤波（针对大面积噪点）
            normalized = self._apply_area_denoise(normalized)
            
            # 应用映射曲线（根据用户选择）
            if self.mapping_curve.get() == "nonlinear":
                normalized = self._apply_nonlinear_mapping(normalized)
            
            # 应用颜色映射（根据用户选择）
            rgb_data = self._apply_colormap(normalized)
            
            # 创建PIL图像
            image = Image.fromarray(rgb_data)
            
            # 使用高质量LANCZOS插值放大图像以提高分辨率
            height, width = temp_matrix.shape
            scale_factor = 20
            new_size = (width * scale_factor, height * scale_factor)
            image = image.resize(new_size, Image.Resampling.LANCZOS)
            
            # 转换为numpy数组进行高级滤波处理
            img_array = np.array(image)
            
            # 应用优化的双边滤波保持边缘清晰度
            try:
                import cv2
                filtered = cv2.bilateralFilter(img_array, d=9, sigmaColor=50, sigmaSpace=50)
                image = Image.fromarray(filtered)
            except:
                pass
            
            # 计算平均温度
            t_avg = np.mean(temp_matrix)
            
            # 创建信息文本
            info_text = f"最高温: {t_max:.1f}°C  |  最低温: {t_min:.1f}°C  |  平均温: {t_avg:.1f}°C"
            
            # 保存原始图片和信息文本
            self.current_raw_image = image
            self.current_info_text = info_text
            # 保存热成像图片尺寸
            self.thermal_image_size = image.size

            # ===== 若是 v2 双光帧, 在 marker 与信息条之前与可见光融合 =====
            fused = self._fuse_thermal_with_visible(image, self.current_visible_image)

            # 在融合后的热成像图上绘制标记
            marked_image = self._draw_markers_on_image(fused)

            # 添加信息条和图例
            new_image = self._add_info_bar_and_legend(marked_image, info_text)
            
            # 更新显示
            self.current_original_image = new_image
            self.display_image(new_image)
            
            self.log(f"已使用新配置重新渲染图片")
            
        except Exception as e:
            self.log(f"重新渲染错误: {e}")
    
    def on_canvas_mouse_move(self, event):
        """鼠标在画布上移动时显示对应温度（画布上绘制悬浮标记）"""
        if self.current_temp_matrix is None or self.thermal_image_size is None:
            return
        
        # 拖拽过程中不显示悬浮标记
        if self.drag_started:
            self._clear_hover_marker()
            return
        
        try:
            # 检测是否悬浮在已有标记上 → 切换光标
            hover_idx = self._find_nearest_marker_canvas(event)
            if hover_idx is not None:
                self.image_canvas.config(cursor='hand2')
                self._clear_hover_marker()
                return
            else:
                self.image_canvas.config(cursor='crosshair')
            
            coords = self._canvas_to_matrix_coords(event)
            if coords is None:
                self._clear_hover_marker()
                self.image_canvas.config(cursor='')
                return
            
            matrix_x, matrix_y = coords
            temp = self.current_temp_matrix[matrix_y, matrix_x]
            
            canvas_x = self.image_canvas.canvasx(event.x)
            canvas_y = self.image_canvas.canvasy(event.y)
            
            # 清除旧的悬浮标记
            self._clear_hover_marker()
            
            # 绘制悬浮十字线 + 温度文本
            cross_size = 10
            tag = '_hover_marker'
            
            # 白色轮廓
            self.image_canvas.create_line(canvas_x - cross_size, canvas_y, canvas_x + cross_size, canvas_y,
                                          fill='white', width=3, tags=tag)
            self.image_canvas.create_line(canvas_x, canvas_y - cross_size, canvas_x, canvas_y + cross_size,
                                          fill='white', width=3, tags=tag)
            # 黑色主体
            self.image_canvas.create_line(canvas_x - cross_size, canvas_y, canvas_x + cross_size, canvas_y,
                                          fill='black', width=1, tags=tag)
            self.image_canvas.create_line(canvas_x, canvas_y - cross_size, canvas_x, canvas_y + cross_size,
                                          fill='black', width=1, tags=tag)
            
            # 温度文本
            text = f"{temp:.2f}°C"
            tx = canvas_x + cross_size + 6
            ty = canvas_y - 8
            
            # 文本背景
            self.image_canvas.create_rectangle(tx - 2, ty - 2, tx + len(text) * 8 + 4, ty + 16,
                                                fill='black', outline='white', tags=tag)
            self.image_canvas.create_text(tx + 2, ty + 7, text=text, fill='white',
                                           anchor=tk.W, font=('Microsoft YaHei', 9), tags=tag)
                
        except Exception:
            pass
    
    def on_canvas_mouse_leave(self, event):
        """鼠标离开画布时清除悬浮标记"""
        self._clear_hover_marker()
        self.image_canvas.config(cursor='')
        self._clear_hover_marker()
    
    def _clear_hover_marker(self):
        """清除画布上的悬浮标记"""
        self.image_canvas.delete('_hover_marker')
    
    def _canvas_to_matrix_coords(self, event):
        """将画布事件坐标转换为温度矩阵坐标，返回 (matrix_x, matrix_y) 或 None"""
        if self.current_temp_matrix is None or self.thermal_image_size is None:
            return None
        
        try:
            canvas_x = self.image_canvas.canvasx(event.x)
            canvas_y = self.image_canvas.canvasy(event.y)
            
            if not hasattr(self, 'current_image_id') or self.current_image_id is None:
                return None
            
            bbox = self.image_canvas.bbox(self.current_image_id)
            if bbox is None:
                return None
            
            img_x0, img_y0, img_x1, img_y1 = bbox
            
            if canvas_x < img_x0 or canvas_x > img_x1 or canvas_y < img_y0 or canvas_y > img_y1:
                return None
            
            rel_x = canvas_x - img_x0
            rel_y = canvas_y - img_y0
            display_width = img_x1 - img_x0
            display_height = img_y1 - img_y0
            thermal_width, thermal_height = self.thermal_image_size
            
            position = self.info_position.get()
            legend_width = 140
            
            if position == "bottom":
                info_bar_height = 50
                thermal_x0, thermal_y0 = 0, 0
                thermal_x1, thermal_y1 = thermal_width, thermal_height
                combined_width = thermal_width + legend_width
                combined_height = thermal_height + info_bar_height
            elif position == "top":
                info_bar_height = 50
                thermal_x0, thermal_y0 = 0, info_bar_height
                thermal_x1, thermal_y1 = thermal_width, info_bar_height + thermal_height
                combined_width = thermal_width + legend_width
                combined_height = thermal_height + info_bar_height
            elif position == "left":
                info_bar_width = 50
                thermal_x0, thermal_y0 = info_bar_width, 0
                thermal_x1, thermal_y1 = info_bar_width + thermal_width, thermal_height
                combined_width = info_bar_width + thermal_width + legend_width
                combined_height = thermal_height
            else:  # right
                info_bar_width = 50
                thermal_x0, thermal_y0 = 0, 0
                thermal_x1, thermal_y1 = thermal_width, thermal_height
                combined_width = info_bar_width + thermal_width + legend_width
                combined_height = thermal_height
            
            scale_x = display_width / combined_width
            scale_y = display_height / combined_height
            
            display_thermal_x0 = thermal_x0 * scale_x
            display_thermal_y0 = thermal_y0 * scale_y
            display_thermal_x1 = thermal_x1 * scale_x
            display_thermal_y1 = thermal_y1 * scale_y
            
            if rel_x < display_thermal_x0 or rel_x > display_thermal_x1:
                return None
            if rel_y < display_thermal_y0 or rel_y > display_thermal_y1:
                return None
            
            thermal_rel_x = (rel_x - display_thermal_x0) / (display_thermal_x1 - display_thermal_x0)
            thermal_rel_y = (rel_y - display_thermal_y0) / (display_thermal_y1 - display_thermal_y0)
            
            height, width = self.current_temp_matrix.shape
            matrix_x = max(0, min(int(thermal_rel_x * width), width - 1))
            matrix_y = max(0, min(int(thermal_rel_y * height), height - 1))
            
            return (matrix_x, matrix_y)
        except Exception:
            return None
    
    def _matrix_to_canvas_coords(self, matrix_x, matrix_y):
        """将温度矩阵坐标转换为画布显示像素坐标，返回 (canvas_x, canvas_y) 或 None"""
        if self.thermal_image_size is None or self.current_temp_matrix is None:
            return None
        if not hasattr(self, 'current_image_id') or self.current_image_id is None:
            return None
        
        try:
            bbox = self.image_canvas.bbox(self.current_image_id)
            if bbox is None:
                return None
            img_x0, img_y0, img_x1, img_y1 = bbox
            display_width = img_x1 - img_x0
            display_height = img_y1 - img_y0
            thermal_width, thermal_height = self.thermal_image_size
            height, width = self.current_temp_matrix.shape
            
            position = self.info_position.get()
            legend_width = 140
            if position == "bottom":
                thermal_x0, thermal_y0 = 0, 0
                combined_width = thermal_width + legend_width
                combined_height = thermal_height + 50
            elif position == "top":
                thermal_x0, thermal_y0 = 0, 50
                combined_width = thermal_width + legend_width
                combined_height = thermal_height + 50
            elif position == "left":
                thermal_x0, thermal_y0 = 50, 0
                combined_width = 50 + thermal_width + legend_width
                combined_height = thermal_height
            else:
                thermal_x0, thermal_y0 = 0, 0
                combined_width = 50 + thermal_width + legend_width
                combined_height = thermal_height
            
            scale_x = display_width / combined_width
            scale_y = display_height / combined_height
            
            # 矩阵坐标 → 热成像图片像素 → 组合图像素 → 画布像素
            thermal_px = (matrix_x + 0.5) / width * thermal_width
            thermal_py = (matrix_y + 0.5) / height * thermal_height
            display_rx = (thermal_x0 + thermal_px) * scale_x
            display_ry = (thermal_y0 + thermal_py) * scale_y
            
            return (img_x0 + display_rx, img_y0 + display_ry)
        except Exception:
            return None
    
    def _find_nearest_marker_canvas(self, event):
        """检测鼠标是否在任一标记的可视区域内（十字线+文本框），返回索引或 None"""
        if not self.temp_markers or self.current_temp_matrix is None or self.thermal_image_size is None:
            return None
        
        canvas_x = self.image_canvas.canvasx(event.x)
        canvas_y = self.image_canvas.canvasy(event.y)
        
        # 计算图片空间到画布空间的缩放比
        if not hasattr(self, 'current_image_id') or self.current_image_id is None:
            return None
        bbox = self.image_canvas.bbox(self.current_image_id)
        if bbox is None:
            return None
        img_x0, img_y0, img_x1, img_y1 = bbox
        display_width = img_x1 - img_x0
        display_height = img_y1 - img_y0
        thermal_width, thermal_height = self.thermal_image_size
        
        position = self.info_position.get()
        legend_width = 140
        if position == "bottom":
            combined_width = thermal_width + legend_width
            combined_height = thermal_height + 50
        elif position == "top":
            combined_width = thermal_width + legend_width
            combined_height = thermal_height + 50
        else:
            combined_width = 50 + thermal_width + legend_width
            combined_height = thermal_height
        
        # 图片像素 → 画布像素的缩放比
        img_to_canvas_x = display_width / combined_width
        img_to_canvas_y = display_height / combined_height
        
        height, width = self.current_temp_matrix.shape
        img_width, img_height = thermal_width, thermal_height
        
        # 复用绘制参数计算标记的可视尺寸（图片像素空间）
        cross_size = max(8, int(img_width / 60))
        font_size = max(16, min(int(img_width / 25), 32))
        pad = max(3, int(font_size * 0.25))
        # 估算文本宽高（字符数 × 字号 × 0.6）
        char_width = font_size * 0.6
        text_height = font_size
        
        sx = img_width / width
        sy = img_height / height
        
        for i, (mx, my, temp) in enumerate(self.temp_markers):
            center_pos = self._matrix_to_canvas_coords(mx, my)
            if center_pos is None:
                continue
            cpx, cpy = center_pos
            
            # 标记中心在图片像素空间的位置
            px_img = (mx + 0.5) * sx
            py_img = (my + 0.5) * sy
            
            # 十字线区域（图片像素）
            cross_x0 = px_img - cross_size
            cross_y0 = py_img - cross_size
            cross_x1 = px_img + cross_size
            cross_y1 = py_img + cross_size
            
            # 文本框区域（图片像素）
            text = f"{temp:.1f}°C"
            tw = len(text) * char_width
            th = text_height
            
            if px_img < img_width // 2:
                tx = px_img + cross_size + pad
            else:
                tx = px_img - cross_size - pad - tw
            ty = py_img - th // 2
            
            rect_x0 = tx - pad
            rect_y0 = ty - pad // 2
            rect_x1 = tx + tw + pad
            rect_y1 = ty + th + pad // 2
            
            # 合并十字线和文本框的边界（图片像素）
            total_x0 = min(cross_x0, rect_x0)
            total_y0 = min(cross_y0, rect_y0)
            total_x1 = max(cross_x1, rect_x1)
            total_y1 = max(cross_y1, rect_y1)
            
            # 转换为画布像素空间的偏移量
            half_w = (total_x1 - total_x0) / 2 * img_to_canvas_x
            half_h = (total_y1 - total_y0) / 2 * img_to_canvas_y
            center_offset_x = ((total_x0 + total_x1) / 2 - px_img) * img_to_canvas_x
            center_offset_y = ((total_y0 + total_y1) / 2 - py_img) * img_to_canvas_y
            
            box_cx = cpx + center_offset_x
            box_cy = cpy + center_offset_y
            
            if abs(canvas_x - box_cx) <= half_w and abs(canvas_y - box_cy) <= half_h:
                return i
        
        return None
    
    def on_canvas_click(self, event):
        """鼠标按下：检测是否点中已有标记（准备拖拽），否则新建"""
        self.drag_started = False
        self.dragging_marker_index = None
        
        coords = self._canvas_to_matrix_coords(event)
        if coords is None:
            return
        
        # 检测是否点中已有标记（画布像素空间）
        idx = self._find_nearest_marker_canvas(event)
        if idx is not None:
            self.dragging_marker_index = idx
            self.image_canvas.config(cursor='fleur')
            return
        
        # 没有命中已有标记，新建一个
        matrix_x, matrix_y = coords
        temp = self.current_temp_matrix[matrix_y, matrix_x]
        self.temp_markers.append((matrix_x, matrix_y, temp))
        self._refresh_with_markers()
        self.log(f"添加温度标记: ({matrix_x},{matrix_y}) = {temp:.2f}°C")
    
    def on_canvas_drag(self, event):
        """鼠标拖拽：移动标记"""
        if self.dragging_marker_index is None:
            return
        
        coords = self._canvas_to_matrix_coords(event)
        if coords is None:
            return
        
        self.drag_started = True
        matrix_x, matrix_y = coords
        temp = self.current_temp_matrix[matrix_y, matrix_x]
        self.temp_markers[self.dragging_marker_index] = (matrix_x, matrix_y, temp)
        self._refresh_with_markers(dragging_index=self.dragging_marker_index)
    
    def on_canvas_release(self, event):
        """鼠标释放：结束拖拽，恢复标记颜色"""
        if self.drag_started:
            self._refresh_with_markers()  # 不传 dragging_index，恢复普通颜色
        self.dragging_marker_index = None
        self.drag_started = False
    
    def on_canvas_right_click(self, event):
        """右键点击：删除最近的标记"""
        idx = self._find_nearest_marker_canvas(event)
        if idx is not None:
            removed = self.temp_markers.pop(idx)
            self._refresh_with_markers()
            self.log(f"删除温度标记: ({removed[0]},{removed[1]}) = {removed[2]:.2f}°C")
    
    def clear_temp_markers(self):
        """清除所有温度标记点"""
        self.temp_markers.clear()
        self._refresh_with_markers()
        self.log("已清除所有温度标记")
    
    def _draw_markers_on_image(self, image, dragging_index=None):
        """在热成像原始图片上绘制所有标记点
        
        Args:
            image: 热成像原始图片（放大后，未加信息条和图例）
            dragging_index: 正在拖拽的标记索引（高亮显示）
            
        Returns:
            绘制了标记的新图片
        """
        if not self.temp_markers:
            return image
        
        from PIL import ImageDraw, ImageFont
        
        img = image.copy()
        draw = ImageDraw.Draw(img)
        img_width, img_height = img.size
        height, width = self.current_temp_matrix.shape
        
        # 缩放比例：温度矩阵坐标 → 放大后的图片坐标
        sx = img_width / width
        sy = img_height / height
        
        # 标记字体：使用系统字体
        marker_font = None
        font_size = max(16, min(int(img_width / 25), 32))
        try:
            for sys_font in ['C:/Windows/Fonts/msyh.ttc', 'C:/Windows/Fonts/simhei.ttf', 'C:/Windows/Fonts/simsun.ttc']:
                try:
                    marker_font = ImageFont.truetype(sys_font, font_size)
                    break
                except Exception:
                    continue
        except Exception:
            pass
        if marker_font is None:
            marker_font = ImageFont.load_default()
        
        # 十字线尺寸适中
        cross_size = max(8, int(img_width / 60))
        cross_width = max(1, int(img_width / 400))
        outline_width = cross_width + 2
        pad = max(3, int(font_size * 0.25))
        
        for i, (mx, my, temp) in enumerate(self.temp_markers):
            px = int((mx + 0.5) * sx)
            py = int((my + 0.5) * sy)
            
            is_dragging = (dragging_index is not None and i == dragging_index)
            # 拖拽中的标记用橙色，普通标记用黑色
            main_color = (255, 140, 0) if is_dragging else (0, 0, 0)  # 橙色 vs 黑色
            outline_color = 'cyan' if is_dragging else 'white'
            bg_color = (80, 40, 0) if is_dragging else (0, 0, 0)
            text_color = (255, 200, 50) if is_dragging else (255, 255, 255)
            
            # 十字线
            draw.line([(px - cross_size, py), (px + cross_size, py)], fill=outline_color, width=outline_width)
            draw.line([(px, py - cross_size), (px, py + cross_size)], fill=outline_color, width=outline_width)
            draw.line([(px - cross_size, py), (px + cross_size, py)], fill=main_color, width=cross_width)
            draw.line([(px, py - cross_size), (px, py + cross_size)], fill=main_color, width=cross_width)
            
            # 中心点
            dot_r = max(2, cross_width)
            draw.ellipse([(px - dot_r, py - dot_r), (px + dot_r, py + dot_r)], fill=main_color, outline=outline_color)
            
            # 温度文本
            text = f"{temp:.1f}°C"
            bbox = draw.textbbox((0, 0), text, font=marker_font)
            tw = bbox[2] - bbox[0]
            th = bbox[3] - bbox[1]
            t_offset_x = bbox[0]
            t_offset_y = bbox[1]
            
            # 文本位置：左半边图片放右侧，右半边图片放左侧
            if px < img_width // 2:
                tx = px + cross_size + pad
            else:
                tx = px - cross_size - pad - tw
            ty = py - th // 2
            if tx + tw + pad > img_width:
                tx = px - cross_size - pad - tw
            if tx < pad:
                tx = px + cross_size + pad
            if ty < pad:
                ty = pad
            if ty + th + pad > img_height:
                ty = img_height - th - pad
            
            # 背景矩形
            rect_x0 = tx - pad
            rect_y0 = ty - pad // 2
            rect_x1 = tx + tw + pad
            rect_y1 = ty + th + pad // 2
            draw.rectangle([rect_x0, rect_y0, rect_x1, rect_y1], fill=bg_color, outline=outline_color, width=max(1, cross_width // 2))
            
            # 文本绘制
            draw.text((tx - t_offset_x, ty - t_offset_y), text, fill=text_color, font=marker_font)
        
        return img
    
    def _refresh_with_markers(self, dragging_index=None):
        """根据标记点重新刷新显示（在原始热成像图上画标记，再添加信息条和图例）"""
        if self.current_raw_image is None or self.current_info_text is None:
            return

        # 若是 v2 双光帧, 先与可见光融合, 再叠加 marker 与信息条
        fused = self._fuse_thermal_with_visible(self.current_raw_image, self.current_visible_image)

        # 在融合后的热成像图上绘制标记
        marked_image = self._draw_markers_on_image(fused, dragging_index=dragging_index)

        # 添加信息条和图例
        new_image = self._add_info_bar_and_legend(marked_image, self.current_info_text)

        # 更新用于保存的图片
        self.current_original_image = new_image

        # 显示
        self.display_image(new_image)
    
    def _create_legend(self, width=40, height=400):
        """创建颜色图例
        
        Args:
            width: 图例宽度（从60减小到40）
            height: 图例高度（从300增加到400）
            
        Returns:
            PIL Image对象
        """
        if self.current_t_min is None or self.current_t_max is None:
            return None
        
        # 创建垂直渐变
        gradient = np.linspace(1, 0, height).reshape(-1, 1)  # 从上到下：热到冷
        gradient = np.tile(gradient, (1, width))
        
        # 如果当前使用非线性映射，图例也要同步应用
        if self.mapping_curve.get() == "nonlinear":
            gradient = self._apply_nonlinear_mapping(gradient)
        
        # 应用当前颜色映射
        rgb_data = self._apply_colormap(gradient)
        legend_img = Image.fromarray(rgb_data)
        
        # 创建带刻度的图例（调整宽度以容纳更大的文字）
        from PIL import ImageDraw, ImageFont
        legend_with_labels = Image.new('RGB', (width + 100, height + 20), 'white')  # 增加右侧空间
        legend_with_labels.paste(legend_img, (10, 10))
        
        draw = ImageDraw.Draw(legend_with_labels)
        
        # 绘制边框
        draw.rectangle([(10, 10), (10 + width, 10 + height)], outline='black', width=2)
        
        # 添加温度刻度
        t_min = self.current_t_min
        t_max = self.current_t_max
        num_ticks = 5
        
        # 为图例创建稍小的字体（18号）
        try:
            search_dirs = [get_resource_dir(), self.base_dir,
                           self.base_dir / "font",
                           Path(__file__).parent / "font"]
            ttf_files = []
            for d in search_dirs:
                try:
                    if d.exists():
                        ttf_files.extend(d.glob("*.ttf"))
                        ttf_files.extend(d.glob("*.otf"))
                except Exception:
                    pass
            if ttf_files:
                legend_font = ImageFont.truetype(str(ttf_files[0]), 18)
            else:
                legend_font = self.font
        except:
            legend_font = self.font
        
        for i in range(num_ticks):
            # 计算温度值（从上到下：高到低）
            temp = t_max - (t_max - t_min) * i / (num_ticks - 1)
            y_pos = 10 + height * i / (num_ticks - 1)
            
            # 绘制刻度线
            draw.line([(10 + width, y_pos), (10 + width + 5, y_pos)], fill='black', width=2)
            
            # 绘制温度文本
            text = f"{temp:.1f}°C"
            if legend_font:
                draw.text((10 + width + 10, y_pos - 10), text, fill='black', font=legend_font)
            else:
                draw.text((10 + width + 10, y_pos - 5), text, fill='black')
        
        return legend_with_labels
    
    def _add_info_bar_and_legend(self, image, info_text):
        """添加信息条和图例
        
        Args:
            image: PIL Image对象
            info_text: 信息文本
            
        Returns:
            添加了信息条和图例的新图片
        """
        # 先添加信息条
        image_with_info = self._add_info_bar(image, info_text)
        
        # 如果有温度数据，添加图例
        if self.current_t_min is not None and self.current_t_max is not None:
            legend = self._create_legend()
            if legend:
                # 在图片右侧添加图例
                img_width, img_height = image_with_info.size
                legend_width, legend_height = legend.size
                
                # 创建新画布
                new_width = img_width + legend_width + 10
                new_height = max(img_height, legend_height)
                combined = Image.new('RGB', (new_width, new_height), 'white')
                
                # 粘贴原图
                combined.paste(image_with_info, (0, 0))
                
                # 粘贴图例（垂直居中）
                legend_y = (new_height - legend_height) // 2
                combined.paste(legend, (img_width + 10, legend_y))
                
                return combined
        
        return image_with_info
                
    def show_port_info(self, port_device):
        """显示串口详细信息"""
        ports = self.get_available_ports()
        
        for device, desc, hwid in ports:
            if device == port_device:
                info = f"已连接到: {device} - {desc}"
                self.log(info)
                break
                
    def start_receiving(self):
        """开始接收图片"""
        if not self.is_connected:
            messagebox.showwarning("未连接", "请先连接串口设备")
            return
            
        self.log("开始接收图片数据...")
        # 这里添加接收图片的逻辑
        # 实际实现需要根据您的热成像设备协议来定制
        
    def stop_receiving(self):
        """停止接收图片"""
        self.log("停止接收图片数据")
        # 这里添加停止接收的逻辑
        
    def open_save_directory(self):
        """打开保存目录"""
        if os.path.exists(self.work_dir):
            os.startfile(self.work_dir)
            self.log(f"打开目录: {self.work_dir}")
        else:
            messagebox.showwarning("目录不存在", "保存目录不存在")
    
    def clear_image_list(self):
        """清空图片列表"""
        self.image_listbox.delete(0, tk.END)
        self.image_list.clear()
        self.current_image = None
        self.image_canvas.delete("all")
        self.log("已清空图片列表")
    
    def on_image_selected(self, event):
        """当用户选择图片时触发"""
        selection = self.image_listbox.curselection()
        if not selection:
            return
        
        list_index = selection[0]
        
        # 从列表框文本中提取photo索引
        display_text = self.image_listbox.get(list_index)
        # 格式: "[索引] filename (大小) - 模式"
        try:
            photo_index = int(display_text.split(']')[0].split('[')[1])
            self.log(f"用户选择图片索引: {photo_index}")
            
            # 自动从设备获取并渲染显示，但不保存到磁盘
            self.download_and_display_photo(photo_index)
            
        except Exception as e:
            self.log(f"解析图片索引错误: {e}")
    
    def download_selected_image(self):
        """保存当前显示的图片到电脑"""
        if self.current_image is None:
            messagebox.showwarning("未选择", "请先选择并显示一张图片")
            return
        
        if self.current_photo_index is None or self.current_filename is None:
            messagebox.showwarning("错误", "当前没有可保存的图片信息")
            return
        
        try:
            # 保存渲染后的图片
            jpg_file = self.rendered_jpg_dir / f"photo_{self.current_photo_index}.jpg"
            
            # 从PhotoImage获取原始PIL Image
            # 注意：self.current_image 现在是缩放后的显示图片
            # 我们需要保存 self.current_original_image (原始渲染图片)
            if hasattr(self, 'current_original_image') and self.current_original_image:
                self.current_original_image.save(jpg_file, 'JPEG', quality=95)
                self.log(f"✓ 图片已保存到: {jpg_file}")
                messagebox.showinfo("保存成功", f"图片已保存到:\n{jpg_file}")
            else:
                self.log("错误: 找不到原始图片数据")
                messagebox.showerror("错误", "找不到原始图片数据")
            
        except Exception as e:
            self.log(f"保存图片错误: {e}")
            messagebox.showerror("保存失败", f"保存图片时出错:\n{e}")
    
    def display_image(self, image):
        """在画布上显示图片"""
        try:
            # 清空画布
            self.image_canvas.delete("all")
            
            # 转换为PhotoImage
            if isinstance(image, Image.Image):
                # 获取画布大小
                canvas_width = self.image_canvas.winfo_width()
                canvas_height = self.image_canvas.winfo_height()
                
                # 如果画布还没有渲染,使用默认大小
                if canvas_width <= 1:
                    canvas_width = 600
                if canvas_height <= 1:
                    canvas_height = 400
                
                # 计算缩放比例以占满画布(允许放大和缩小)
                img_width, img_height = image.size
                scale_w = canvas_width / img_width
                scale_h = canvas_height / img_height
                scale = min(scale_w, scale_h)  # 移除1.0限制,允许放大
                
                # 缩放图片到占满画布
                new_width = int(img_width * scale)
                new_height = int(img_height * scale)
                display_image = image.resize((new_width, new_height), Image.Resampling.LANCZOS)
                
                # 保存引用防止被垃圾回收
                self.current_image = ImageTk.PhotoImage(display_image)
                
                # 在画布中央显示图片
                x = canvas_width // 2
                y = canvas_height // 2
                self.current_image_id = self.image_canvas.create_image(x, y, image=self.current_image, anchor=tk.CENTER)
                
                # 更新滚动区域
                self.image_canvas.configure(scrollregion=self.image_canvas.bbox("all"))
                
        except Exception as e:
            self.log(f"显示图片错误: {e}")
    
    def add_image_to_list(self, image_name, image):
        """添加图片到列表
        
        Args:
            image_name: 图片名称(显示在列表中)
            image: PIL Image对象
        """
        self.image_list.append((image_name, image))
        self.image_listbox.insert(tk.END, image_name)
        self.log(f"添加图片到列表: {image_name}")
        
        # 自动选择最新添加的图片
        last_index = self.image_listbox.size() - 1
        self.image_listbox.selection_clear(0, tk.END)
        self.image_listbox.selection_set(last_index)
        self.image_listbox.see(last_index)
        self.display_image(image)
            
    def on_closing(self):
        """关闭窗口时的处理"""
        # 停止自动获取列表
        self.stop_auto_get_list_task()

        if self.is_connected:
            self.disconnect_serial()
        # 不 destroy 顶层 root, 此 frame 关闭由 app 控制
