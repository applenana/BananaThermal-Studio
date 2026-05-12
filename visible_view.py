"""
visible_view.py
================

可见光画面 Tkinter 显示组件.

功能
----

* 接收 ``frame_parser.FrameParser`` 推送的 RGB565 帧 ``(H, W) uint16``
* 转换到 RGB888 后用 PIL.Image 显示在 Tkinter Label 上
* 自动按窗口尺寸放大 (LANCZOS), 保持宽高比
* 支持设备方向旋转 (90/180/270 度) 与水平/垂直镜像
* 提供 FPS 显示, 方便调试

线程安全
--------

``update_frame`` 必须在主线程 (Tk mainloop) 调用. 串口线程拿到帧后,
应通过 ``root.after(0, view.update_frame, ...)`` 或 queue 派发到主线程.
"""

from __future__ import annotations

import time
import tkinter as tk
from tkinter import ttk
from typing import Optional

import numpy as np
from PIL import Image, ImageTk

from frame_parser import rgb565_to_rgb888


class VisibleView(ttk.Frame):
    """嵌入式 Tkinter 可见光显示组件.

    Parameters
    ----------
    master : tk.Misc
        父容器
    initial_size : tuple[int, int]
        初始显示尺寸 (width, height) 像素. 用户可拖动改变窗口后自动跟随.
    """

    # 旋转角度选项 (与设备屏旋转一致, 方便对位)
    ROTATIONS = (0, 90, 180, 270)

    def __init__(self, master: tk.Misc, initial_size: tuple = (360, 480)):
        super().__init__(master)

        # ---- 内部状态 ----
        self._latest_frame: Optional[np.ndarray] = None  # 最近一帧 RGB888
        self._photo: Optional[ImageTk.PhotoImage] = None # 防 GC
        self._fps_count = 0
        self._fps_t0 = time.monotonic()
        self._fps_value = 0.0

        # ---- 控件 ----
        self.rotation_var = tk.IntVar(value=90)
        self.mirror_h_var = tk.BooleanVar(value=False)
        self.mirror_v_var = tk.BooleanVar(value=False)

        toolbar = ttk.Frame(self)
        toolbar.pack(side=tk.TOP, fill=tk.X, padx=4, pady=2)

        ttk.Label(toolbar, text="旋转:").pack(side=tk.LEFT)
        self.rotation_combo = ttk.Combobox(
            toolbar,
            values=[f"{a}°" for a in self.ROTATIONS],
            state="readonly",
            width=5,
        )
        self.rotation_combo.set("90°")
        self.rotation_combo.pack(side=tk.LEFT, padx=(2, 8))
        self.rotation_combo.bind("<<ComboboxSelected>>", self._on_rotation_changed)

        ttk.Checkbutton(toolbar, text="水平镜像", variable=self.mirror_h_var,
                        command=self._redraw).pack(side=tk.LEFT, padx=2)
        ttk.Checkbutton(toolbar, text="垂直镜像", variable=self.mirror_v_var,
                        command=self._redraw).pack(side=tk.LEFT, padx=2)

        self.fps_label = ttk.Label(toolbar, text="FPS: --", width=10)
        self.fps_label.pack(side=tk.RIGHT)

        # 显示画布: 用原生 tk.Label, ttk.Label 不支持像素级 width/height
        self.image_label = tk.Label(self, anchor=tk.CENTER, background="black")
        self.image_label.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # 记下初始尺寸, 给首帧 fallback 使用 (winfo_width 在布局完成前为 1)
        self._init_w, self._init_h = initial_size

        # 重绘节流: 每 30ms (~33 fps) 触发一次实际重绘, 即使收到更高帧率
        self._redraw_pending = False
        self.bind("<Configure>", lambda e: self._schedule_redraw())

    # ------------------------------------------------------------------
    # 公共 API
    # ------------------------------------------------------------------

    def update_frame(self, width: int, height: int, rgb565: np.ndarray) -> None:
        """收到一帧新数据 (主线程调用).

        Parameters
        ----------
        width, height : int
            像素宽高 (来自帧头)
        rgb565 : np.ndarray, shape=(H, W), dtype=uint16
            RGB565 像素数据
        """
        # 颜色域转换 → RGB888
        self._latest_frame = rgb565_to_rgb888(rgb565)

        # FPS 统计 (1 秒滑动窗口)
        self._fps_count += 1
        now = time.monotonic()
        if now - self._fps_t0 >= 1.0:
            self._fps_value = self._fps_count / (now - self._fps_t0)
            self._fps_count = 0
            self._fps_t0 = now
            self.fps_label.configure(text=f"FPS: {self._fps_value:4.1f}")

        self._schedule_redraw()

    def clear(self) -> None:
        """清除显示 (例如断开连接时)."""
        self._latest_frame = None
        self._photo = None
        self.image_label.configure(image="")
        self.fps_label.configure(text="FPS: --")

    def get_latest_image_pil(self) -> "Image.Image | None":
        """返回应用了旋转/镜像之后的最新 PIL.Image (RGB), 供外部融合使用. 无帧返回 None."""
        if self._latest_frame is None:
            return None
        arr = self._latest_frame
        if self.mirror_h_var.get():
            arr = arr[:, ::-1, :]
        if self.mirror_v_var.get():
            arr = arr[::-1, :, :]
        img = Image.fromarray(arr, mode="RGB")
        rot = self.rotation_var.get()
        if rot:
            mapping = {
                90: Image.Transpose.ROTATE_270,
                180: Image.Transpose.ROTATE_180,
                270: Image.Transpose.ROTATE_90,
            }
            img = img.transpose(mapping[rot])
        return img

    # ------------------------------------------------------------------
    # 内部
    # ------------------------------------------------------------------

    def _on_rotation_changed(self, _evt=None) -> None:
        sel = self.rotation_combo.get().rstrip("°")
        try:
            self.rotation_var.set(int(sel))
        except ValueError:
            pass
        self._redraw()

    def _schedule_redraw(self) -> None:
        """合并多次 update_frame, 避免主线程被 PIL 转换淹没."""
        if self._redraw_pending:
            return
        self._redraw_pending = True
        self.after(30, self._redraw)

    def _redraw(self) -> None:
        self._redraw_pending = False
        if self._latest_frame is None:
            return

        # 取出 RGB888, 应用镜像/旋转
        arr = self._latest_frame
        if self.mirror_h_var.get():
            arr = arr[:, ::-1, :]
        if self.mirror_v_var.get():
            arr = arr[::-1, :, :]

        img = Image.fromarray(arr, mode="RGB")
        rot = self.rotation_var.get()
        if rot:
            # PIL.Image.rotate 的角度方向为逆时针, 这里跟 GUI 标签一致用顺时针
            # 0/90/180/270 用 transpose 的常量更快且无插值
            mapping = {
                90: Image.Transpose.ROTATE_270,   # 视觉上顺时针 90
                180: Image.Transpose.ROTATE_180,
                270: Image.Transpose.ROTATE_90,   # 视觉上顺时针 270
            }
            img = img.transpose(mapping[rot])

        # 按 Label 当前尺寸等比缩放 (留 4px 边距防越界).
        # 首帧到达时 winfo 可能尚未布局完成, 回落到初始尺寸防止跳变
        w_now = self.image_label.winfo_width()
        h_now = self.image_label.winfo_height()
        if w_now <= 1 or h_now <= 1:
            w_now, h_now = self._init_w, self._init_h
        target_w = max(w_now - 4, 80)
        target_h = max(h_now - 4, 80)
        scale = min(target_w / img.width, target_h / img.height)
        if scale > 1:
            new_size = (int(img.width * scale), int(img.height * scale))
            img = img.resize(new_size, Image.Resampling.LANCZOS)

        self._photo = ImageTk.PhotoImage(img)
        self.image_label.configure(image=self._photo)


# ---------------------------------------------------------------------------
# 单文件 demo: 运行 ``python visible_view.py`` 可看到一帧测试图
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    root = tk.Tk()
    root.title("VisibleView demo")
    root.geometry("400x550")

    view = VisibleView(root, initial_size=(360, 480))
    view.pack(fill=tk.BOTH, expand=True)

    # 构造一帧渐变测试图 (120x160, 红→绿→蓝)
    h, w = 160, 120
    test = np.zeros((h, w), dtype="<u2")
    for y in range(h):
        for x in range(w):
            r = int(31 * x / (w - 1)) & 0x1F
            g = int(63 * y / (h - 1)) & 0x3F
            b = int(31 * (1 - x / (w - 1))) & 0x1F
            test[y, x] = (r << 11) | (g << 5) | b

    def push():
        view.update_frame(w, h, test)
        root.after(100, push)

    root.after(100, push)
    root.mainloop()
