"""
frame_parser.py
================

串口字节流 → 热成像帧 / 可见光帧 解析器 (零 GUI 依赖).

与设备固件 ``src/streaming.h`` 字节级一致, 同一根串口同时承载两种帧:

热帧 (固件 ``stream`` 命令)::

    "BEGIN" + T_max(4B float LE) + T_min(4B float LE) + T_avg(4B float LE)
           + 768 * float32 LE (3072 B, 24 行 32 列, 行优先)
           + "END"

可见光帧 (固件 ``vstream`` 命令)::

    "VBEG" + width(4B uint32 LE) + height(4B uint32 LE) + len(4B uint32 LE)
           + RGB565 LE * len/2  (行优先 row*width+col)
           + "VEND"

使用方式::

    parser = FrameParser(
        on_thermal=lambda tmax, tmin, tavg, arr: ...,
        on_visible=lambda w, h, arr: ...,
    )
    while True:
        chunk = ser.read(ser.in_waiting or 1)
        parser.feed(chunk)

设计要点
--------

* **无锁、单线程消费**: ``feed`` 每次追加字节后立即尝试解析所有完整帧.
  调用方负责保证 ``feed`` 不并发. 若需多线程, 在外层加锁即可.
* **容错**: 同时扫两个 magic, 找不到就丢弃 buffer 头部至下一个 magic 候选,
  防止单帧错位导致永久卡住.
* **零拷贝**: 帧体直接 ``np.frombuffer`` view 出来 (拷贝一次到 numpy 数组,
  避免持有原始 buffer 引用).
* **协议向前兼容**: 旧固件不会发 VBEG 帧, 解析器对此完全透明.

作者: 项目维护者 (适用于 Heimann + RP2040 双光红外测温仪)
许可: 与主项目一致
"""

from __future__ import annotations

import struct
from typing import Callable, Optional

import numpy as np

# ---------------------------------------------------------------------------
# 协议常量 (字节级与设备固件 streaming.h 一致, 修改前请确认双端同步)
# ---------------------------------------------------------------------------
THERMAL_MAGIC_BEGIN = b"BEGIN"
THERMAL_MAGIC_END = b"END"
THERMAL_HEADER_SIZE = 12          # T_max + T_min + T_avg, 各 4 字节 float LE
THERMAL_PIXEL_COUNT = 768         # 24 * 32
THERMAL_PIXEL_BYTES = THERMAL_PIXEL_COUNT * 4
THERMAL_FRAME_TOTAL = (
    len(THERMAL_MAGIC_BEGIN) + THERMAL_HEADER_SIZE + THERMAL_PIXEL_BYTES + len(THERMAL_MAGIC_END)
)  # 5 + 12 + 3072 + 3 = 3092

VISIBLE_MAGIC_BEGIN = b"VBEG"
VISIBLE_MAGIC_END = b"VEND"
VISIBLE_HEADER_SIZE = 12          # width + height + len, 各 4 字节 uint32 LE
VISIBLE_MAX_PAYLOAD = 120 * 160 * 2  # 38400 B, 上限保护

# 任一 magic 起始可能的最短长度 (用于扫描时的剪枝)
_ANY_MAGIC_MIN = min(len(THERMAL_MAGIC_BEGIN), len(VISIBLE_MAGIC_BEGIN))


# ---------------------------------------------------------------------------
# 回调类型别名 (帮助 IDE 自动提示, 不强制)
# ---------------------------------------------------------------------------
ThermalCallback = Callable[[float, float, float, np.ndarray], None]
"""(t_max, t_min, t_avg, frame[24,32] float32 °C) -> None"""

VisibleCallback = Callable[[int, int, np.ndarray], None]
"""(width, height, frame[height,width] uint16 RGB565) -> None"""


class FrameParser:
    """串口字节流的双类型帧解析器.

    Parameters
    ----------
    on_thermal : ThermalCallback, optional
        每解析出一帧热成像就调用. 不传则丢弃热帧.
    on_visible : VisibleCallback, optional
        每解析出一帧可见光就调用. 不传则丢弃可见光帧.
    max_buffer : int
        内部缓冲区上限. 超过此值会丢弃旧数据防止 OOM. 默认 512KB,
        足够承载多个完整可见光帧而不溢出.
    """

    def __init__(
        self,
        on_thermal: Optional[ThermalCallback] = None,
        on_visible: Optional[VisibleCallback] = None,
        max_buffer: int = 512 * 1024,
    ):
        self._on_thermal = on_thermal
        self._on_visible = on_visible
        self._max_buffer = max_buffer
        self._buf = bytearray()

        # 统计 (供调试 / UI 显示用)
        self.thermal_frames = 0
        self.visible_frames = 0
        self.dropped_bytes = 0  # 因找不到 magic 而被跳过的字节数

    # ------------------------------------------------------------------
    # 公共 API
    # ------------------------------------------------------------------

    def feed(self, data: bytes) -> None:
        """追加数据并尝试解析所有完整帧.

        每次调用都会尽可能多地从 buffer 中提取完整帧并触发回调,
        剩余不完整的部分留在 buffer 中等待下次 ``feed``.
        """
        if not data:
            return
        self._buf.extend(data)

        # 防爆保护: 缓冲过大时丢弃旧数据
        if len(self._buf) > self._max_buffer:
            drop = len(self._buf) - self._max_buffer
            self.dropped_bytes += drop
            del self._buf[:drop]

        # 循环解析直到 buffer 内不再能完整出帧
        while self._try_extract_one_frame():
            pass

    def reset(self) -> None:
        """清空内部 buffer 与计数器 (例如串口重连后调用)."""
        self._buf.clear()
        self.thermal_frames = 0
        self.visible_frames = 0
        self.dropped_bytes = 0

    # ------------------------------------------------------------------
    # 内部解析逻辑
    # ------------------------------------------------------------------

    def _try_extract_one_frame(self) -> bool:
        """从 buffer 头部尝试提取一帧 (热帧或可见光帧).

        Returns
        -------
        bool
            True  : 成功解析了一帧 (回调已触发, buffer 已前进).
            False : buffer 不足或当前位置没有可识别的 magic, 应等待更多数据.
        """
        # 找到下一个 magic 出现的最早位置, 中间的字节作为噪声丢弃
        idx_thermal = self._buf.find(THERMAL_MAGIC_BEGIN)
        idx_visible = self._buf.find(VISIBLE_MAGIC_BEGIN)

        # 都没找到: 保留 buffer 末尾的 (magic_len - 1) 字节, 防止跨 chunk 截断
        if idx_thermal < 0 and idx_visible < 0:
            keep = max(len(THERMAL_MAGIC_BEGIN), len(VISIBLE_MAGIC_BEGIN)) - 1
            if len(self._buf) > keep:
                drop = len(self._buf) - keep
                self.dropped_bytes += drop
                del self._buf[:drop]
            return False

        # 找到至少一个: 选最早出现的那个
        if idx_thermal < 0:
            idx, kind = idx_visible, "v"
        elif idx_visible < 0:
            idx, kind = idx_thermal, "t"
        else:
            if idx_thermal <= idx_visible:
                idx, kind = idx_thermal, "t"
            else:
                idx, kind = idx_visible, "v"

        # magic 之前的字节是噪声, 丢弃
        if idx > 0:
            self.dropped_bytes += idx
            del self._buf[:idx]

        # 现在 buffer 头部就是 magic 起始, 尝试按对应协议读出整帧
        if kind == "t":
            return self._try_extract_thermal()
        else:
            return self._try_extract_visible()

    # ------------------------------------------------------------------
    def _try_extract_thermal(self) -> bool:
        """假定 buffer 以 BEGIN 开头, 尝试读出完整热成像帧."""
        if len(self._buf) < THERMAL_FRAME_TOTAL:
            return False  # 数据还不够一帧, 等下次

        # 校验帧尾 magic, 不对就把 BEGIN 当噪声丢掉避免锁死
        end_off = len(THERMAL_MAGIC_BEGIN) + THERMAL_HEADER_SIZE + THERMAL_PIXEL_BYTES
        if bytes(self._buf[end_off:end_off + len(THERMAL_MAGIC_END)]) != THERMAL_MAGIC_END:
            self.dropped_bytes += 1
            del self._buf[:1]   # 仅丢首字节, 让外层再扫下一个 magic
            return True         # 仍算一次"前进", 让 while 继续

        # 解析三温
        t_max, t_min, t_avg = struct.unpack_from(
            "<fff", self._buf, len(THERMAL_MAGIC_BEGIN)
        )

        # 解析 768 个 float32 像素 (拷贝一次, 避免持有 buffer 引用)
        pixel_off = len(THERMAL_MAGIC_BEGIN) + THERMAL_HEADER_SIZE
        pixels = np.frombuffer(
            bytes(self._buf[pixel_off:pixel_off + THERMAL_PIXEL_BYTES]),
            dtype="<f4",
            count=THERMAL_PIXEL_COUNT,
        ).reshape(24, 32).copy()

        # 前进 buffer
        del self._buf[:THERMAL_FRAME_TOTAL]
        self.thermal_frames += 1

        if self._on_thermal is not None:
            self._on_thermal(t_max, t_min, t_avg, pixels)
        return True

    # ------------------------------------------------------------------
    def _try_extract_visible(self) -> bool:
        """假定 buffer 以 VBEG 开头, 尝试读出完整可见光帧."""
        magic_len = len(VISIBLE_MAGIC_BEGIN)
        # 至少要凑出 magic + 12B header 才能知道 payload 长度
        if len(self._buf) < magic_len + VISIBLE_HEADER_SIZE:
            return False

        width, height, payload_len = struct.unpack_from(
            "<III", self._buf, magic_len
        )

        # payload 长度合理性检查: 防恶意/错乱字节让我们读 GB 级数据
        if payload_len <= 0 or payload_len > VISIBLE_MAX_PAYLOAD or payload_len != width * height * 2:
            self.dropped_bytes += 1
            del self._buf[:1]
            return True

        total = magic_len + VISIBLE_HEADER_SIZE + payload_len + len(VISIBLE_MAGIC_END)
        if len(self._buf) < total:
            return False  # 等更多数据

        end_off = magic_len + VISIBLE_HEADER_SIZE + payload_len
        if bytes(self._buf[end_off:end_off + len(VISIBLE_MAGIC_END)]) != VISIBLE_MAGIC_END:
            self.dropped_bytes += 1
            del self._buf[:1]
            return True

        payload_off = magic_len + VISIBLE_HEADER_SIZE
        rgb565 = np.frombuffer(
            bytes(self._buf[payload_off:payload_off + payload_len]),
            dtype="<u2",
            count=width * height,
        ).reshape(height, width).copy()

        del self._buf[:total]
        self.visible_frames += 1

        if self._on_visible is not None:
            self._on_visible(width, height, rgb565)
        return True


# ---------------------------------------------------------------------------
# 辅助: RGB565 → RGB888 转换 (上位机显示前必做的颜色域转换)
# ---------------------------------------------------------------------------

def rgb565_to_rgb888(frame: np.ndarray) -> np.ndarray:
    """把 ``uint16 RGB565`` 帧转换为 ``uint8 RGB888`` 帧.

    Parameters
    ----------
    frame : np.ndarray, shape=(H, W), dtype=uint16
        每像素布局 (从 MSB): RRRRR GGGGGG BBBBB

    Returns
    -------
    np.ndarray, shape=(H, W, 3), dtype=uint8
        通道顺序 R, G, B.
    """
    r = ((frame >> 11) & 0x1F).astype(np.uint16)
    g = ((frame >> 5)  & 0x3F).astype(np.uint16)
    b = (frame         & 0x1F).astype(np.uint16)

    # 5 bit / 6 bit → 8 bit, 用左移 + 高位补低位的方式比简单乘法更准确
    r8 = ((r << 3) | (r >> 2)).astype(np.uint8)
    g8 = ((g << 2) | (g >> 4)).astype(np.uint8)
    b8 = ((b << 3) | (b >> 2)).astype(np.uint8)

    return np.stack([r8, g8, b8], axis=-1)


# ---------------------------------------------------------------------------
# 模块自测 (直接 ``python frame_parser.py`` 执行)
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    # 构造一帧合法热数据
    pixels = np.random.uniform(20, 40, size=(24, 32)).astype("<f4")
    thermal_bytes = (
        b"BEGIN"
        + struct.pack("<fff", 40.0, 20.0, 30.0)
        + pixels.tobytes()
        + b"END"
    )
    # 构造一帧合法可见光 (8x4 简化)
    w, h = 8, 4
    payload = np.arange(w * h, dtype="<u2").tobytes()
    visible_bytes = b"VBEG" + struct.pack("<III", w, h, len(payload)) + payload + b"VEND"

    received = {"t": 0, "v": 0}

    def on_t(tmax, tmin, tavg, arr):
        assert arr.shape == (24, 32)
        received["t"] += 1

    def on_v(ww, hh, arr):
        assert (ww, hh) == (w, h) and arr.shape == (h, w)
        received["v"] += 1

    p = FrameParser(on_thermal=on_t, on_visible=on_v)

    # 测试 1: 拼接送入, 应解出 2 + 2 帧
    p.feed(thermal_bytes + visible_bytes + thermal_bytes + visible_bytes)
    assert received == {"t": 2, "v": 2}, f"got {received}"

    # 测试 2: 头部噪声 + 跨边界送入
    p.reset()
    received = {"t": 0, "v": 0}
    big = b"\x00\x01\xff" + thermal_bytes + b"junk" + visible_bytes
    for i in range(0, len(big), 17):
        p.feed(big[i:i + 17])
    assert received == {"t": 1, "v": 1}, f"got {received}"

    # 测试 3: RGB565 → RGB888
    test_frame = np.array([[0xF800, 0x07E0, 0x001F]], dtype="<u2")  # 红、绿、蓝
    rgb = rgb565_to_rgb888(test_frame)
    assert rgb[0, 0, 0] > 240 and rgb[0, 1, 1] > 240 and rgb[0, 2, 2] > 240

    print("frame_parser self-test PASSED")
    print(f"  thermal frame total bytes = {THERMAL_FRAME_TOTAL}")
    print(f"  visible frame max bytes   = {len(VISIBLE_MAGIC_BEGIN) + VISIBLE_HEADER_SIZE + VISIBLE_MAX_PAYLOAD + len(VISIBLE_MAGIC_END)}")
