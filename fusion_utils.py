"""
fusion_utils.py
================

可见光 + 热成像 PC 端融合算法 (与 photo_download_tab 共享).

模式
----
* ``off``   - 仅热成像
* ``blend`` - alpha 加权混合 (热成像 * (1-α) + 可见光 * α)
* ``edge``  - 可见光 Sobel 边缘叠加到热成像 (设备端 ``fusion edge`` 一致)

注意
----
* 输入热像 PIL 必须是已染色后的 RGB Image
* 可见光 PIL 也是 RGB
* 输出大小 == 热像大小
"""
from __future__ import annotations

import numpy as np
from PIL import Image
import matplotlib.cm as mcm

try:
    import cv2  # type: ignore
    _HAS_CV2 = True
except Exception:  # pragma: no cover
    _HAS_CV2 = False


def hex_to_rgb(hex_color: str) -> tuple[int, int, int]:
    s = hex_color.lstrip('#')
    if len(s) == 3:
        s = ''.join(c * 2 for c in s)
    return int(s[0:2], 16), int(s[2:4], 16), int(s[4:6], 16)


def apply_nonlinear_mapping(normalized: np.ndarray, power: float = 2.5) -> np.ndarray:
    """S 曲线: 中间区域差异更大, 两端更平缓."""
    x = np.clip(normalized, 0.0, 1.0)
    return 1.0 / (1.0 + ((1.0 - x) / np.maximum(x, 1e-9)) ** power)


def colorize(normalized: np.ndarray,
             colormap_name: str = "jet",
             mapping_curve: str = "linear",
             use_custom_colors: bool = False,
             cold_color: str = "#0000ff",
             mid_color: str = "#00ff00",
             hot_color: str = "#ff0000") -> np.ndarray:
    """归一化矩阵 → uint8 RGB (H,W,3). 与 photo_download_tab 同款规则.

    * mapping_curve: linear / nonlinear (S 曲线)
    * use_custom_colors=True: 三色渐变 (cold→mid→hot)
    * use_custom_colors=False: matplotlib colormap (按名称, 内部裁剪到 0.05~0.95)
    """
    data = np.clip(normalized, 0.0, 1.0)
    if mapping_curve == "nonlinear":
        data = apply_nonlinear_mapping(data)

    if use_custom_colors:
        cold = np.array(hex_to_rgb(cold_color), dtype=np.float32)
        mid = np.array(hex_to_rgb(mid_color), dtype=np.float32)
        hot = np.array(hex_to_rgb(hot_color), dtype=np.float32)
        h, w = data.shape
        rgb = np.zeros((h, w, 3), dtype=np.uint8)
        mask_low = data <= 0.5
        mask_high = ~mask_low
        for i in range(3):
            tl = data[mask_low] * 2
            rgb[mask_low, i] = (cold[i] * (1 - tl) + mid[i] * tl).astype(np.uint8)
            th_ = (data[mask_high] - 0.5) * 2
            rgb[mask_high, i] = (mid[i] * (1 - th_) + hot[i] * th_).astype(np.uint8)
        return rgb

    try:
        cmap = mcm.get_cmap(colormap_name)
    except Exception:
        cmap = mcm.get_cmap("jet")
    # 裁剪到 0.05~0.95, 避免两端过黑/过白
    clipped = data * 0.9 + 0.05
    colored = cmap(clipped)
    return (colored[..., :3] * 255.0).astype(np.uint8)


def fuse(thermal_pil: Image.Image,
         visible_pil: Image.Image | None,
         mode: str = "off",
         gamma: float = 1.0,
         alpha: float = 0.5,
         edge_strength: float = 0.6,
         edge_thresh: float = 0.082,
         edge_width: int = 1,
         edge_color: str = "#333333") -> Image.Image:
    """对单帧执行融合, 返回新的 PIL.Image (尺寸 == thermal_pil)."""
    if visible_pil is None or mode == "off":
        return thermal_pil

    tw, th = thermal_pil.size
    therm_arr = np.array(thermal_pil, dtype=np.float32) / 255.0

    # 可见光缩到热像尺寸 (LANCZOS 给 blend; edge 模式实际用原图算 Sobel)
    vis_resized = visible_pil.resize((tw, th), Image.Resampling.LANCZOS)
    vis_arr = np.array(vis_resized, dtype=np.float32) / 255.0

    # 伽马
    if gamma <= 0:
        gamma = 1.0
    vis_arr = np.clip(vis_arr, 0.0, 1.0) ** (1.0 / gamma)

    if mode == "blend":
        a = max(0.0, min(1.0, float(alpha)))
        out = therm_arr * (1.0 - a) + vis_arr * a

    elif mode == "edge":
        src_arr = np.array(visible_pil)  # 原始分辨率
        if _HAS_CV2:
            gray0 = cv2.cvtColor(src_arr, cv2.COLOR_RGB2GRAY)
            gx0 = cv2.Sobel(gray0, cv2.CV_32F, 1, 0, ksize=3)
            gy0 = cv2.Sobel(gray0, cv2.CV_32F, 0, 1, ksize=3)
        else:
            gray0 = src_arr.astype(np.float32).mean(axis=2)
            gx0 = np.zeros_like(gray0)
            gy0 = np.zeros_like(gray0)
            gx0[1:-1, 1:-1] = (
                -gray0[:-2, :-2] + gray0[:-2, 2:]
                - 2 * gray0[1:-1, :-2] + 2 * gray0[1:-1, 2:]
                - gray0[2:, :-2] + gray0[2:, 2:]
            )
            gy0[1:-1, 1:-1] = (
                -gray0[:-2, :-2] - 2 * gray0[:-2, 1:-1] - gray0[:-2, 2:]
                + gray0[2:, :-2] + 2 * gray0[2:, 1:-1] + gray0[2:, 2:]
            )
        mag0 = np.abs(gx0) + np.abs(gy0)
        t = max(0.0, min(1.0, float(edge_thresh))) * 1020.0
        mask0 = (mag0 > t).astype(np.float32)

        if _HAS_CV2:
            mask = cv2.resize(mask0, (tw, th), interpolation=cv2.INTER_NEAREST)
        else:
            mi = Image.fromarray((mask0 * 255).astype(np.uint8))
            mi = mi.resize((tw, th), Image.Resampling.NEAREST)
            mask = np.array(mi).astype(np.float32) / 255.0

        w = max(0, min(6, int(edge_width)))
        if w > 0:
            if _HAS_CV2:
                k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * w + 1, 2 * w + 1))
                mask = cv2.dilate(mask, k, iterations=1)
            else:
                try:
                    from scipy.ndimage import maximum_filter
                    mask = maximum_filter(mask, size=2 * w + 1)
                except Exception:
                    pass

        s = max(0.0, min(1.0, float(edge_strength)))
        er, eg, eb = hex_to_rgb(edge_color)
        cr = np.full_like(mask, er / 255.0)
        cg = np.full_like(mask, eg / 255.0)
        cb = np.full_like(mask, eb / 255.0)
        weight = mask * s
        w3 = np.dstack([weight, weight, weight])
        color_layer = np.dstack([cr, cg, cb])
        out = therm_arr * (1.0 - w3) + color_layer * w3
        out = np.clip(out, 0.0, 1.0)
    else:
        return thermal_pil

    out_u8 = np.clip(out * 255.0, 0, 255).astype(np.uint8)
    return Image.fromarray(out_u8, mode='RGB')
