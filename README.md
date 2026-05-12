# 🍌 BananaThermal Studio

> **EN** — A modular PC host for the open-source **BananaNi Thermal Imager** (RP2040 + Heimann HTPA + OV2640). Live dual-light (thermal + visible) streaming, real-time RGB fusion, and on-device photo download — all in one Tk-based GUI.
>
> **中** — 适配 **香蕉泥热成像通讯协议** 的开源 PC 端上位机，提供热成像 + 可见光双光实时投屏、伪彩可调、实时融合、设备激活、片上图片下载渲染等功能。

<p align="center">
  <a href="https://github.com/applenana/BananaThermal-Studio/actions/workflows/build-exe.yml"><img alt="build" src="https://github.com/applenana/BananaThermal-Studio/actions/workflows/build-exe.yml/badge.svg"></a>
  <img alt="python" src="https://img.shields.io/badge/python-3.11%2B-blue">
  <img alt="platform" src="https://img.shields.io/badge/platform-Windows%20(x64)-lightgrey">
  <img alt="license" src="https://img.shields.io/badge/license-MIT-green">
</p>

---

## ✨ Features / 功能

**EN**
- 🔥 **Dual-light live view** — thermal `24×32 float` + visible `RGB565` muxed on a single 1 Mbps serial link
- 🎨 **Real-time fusion** — three modes (`thermal-only` / `blend` / `edge overlay`), live gamma / alpha / edge strength / edge color picker
- 🌈 **Colormap controls** — 11 built-in `matplotlib` colormaps + custom 3-color (cold / mid / hot) + linear / S-curve mapping
- 📥 **On-device photo download** — fetch JPEGs stored on RP2040 flash, batch re-render with overlay / annotations
- 🧠 **Stateful protocol parser** — pure-Python `FrameParser`, splits BEGIN/END (thermal) and VBEG/VEND (visible) markers, fully unit-testable
- 🔐 **Auto-detect + activation** — port scan, `GetSysInfo` probe, in-GUI `activate <key>` flow
- 🌡️ **Optional filtering** — 1-D Kalman on Tmax/Tmin/Tavg, 2-D per-pixel Kalman on the frame, OpenCV bilateral (toggle, off by default)
- 📈 **Rolling chart** — Tmax / Tmin / Tavg history embedded in the control panel, throttled 5 fps redraw
- 🖥️ **DPI-aware** — `SetProcessDpiAwareness(1)` + forced `tk scaling 1.50`, all geometry scaled at runtime
- 💬 **Built-in console** — raw commands (`stream` / `vstream` / `GetSysInfo` / `activate <key>` …) with color-coded log
- 📦 **One-click EXE** — GitHub Actions builds a single-file Windows EXE on every push, bundles fonts via PyInstaller `_MEIPASS`

**中**
- 🔥 **双光实时投屏** — 热成像 `24×32 float` + 可见光 `RGB565`，单根 1 Mbps 串口多路复用
- 🎨 **实时融合** — 纯热像 / 混合 / 边缘叠加 三种模式，伽马、可见光比例、边缘强度/阈值/粗细/颜色均可现场调
- 🌈 **伪彩调色** — 11 种 `matplotlib` 调色盘 + 自定义冷/中/热三色 + 线性/S曲线两种映射曲线
- 📥 **片上图片下载** — 把 RP2040 flash 里的 JPEG 批量拉到 PC，支持叠加批注重渲染
- 🧠 **状态机协议解析** — `frame_parser.py` 纯 Python，按 `BEGIN/END`（热像）与 `VBEG/VEND`（可见光）做分流，无 GUI 依赖，便于单元测试
- 🔐 **自动识别 + 激活** — 自动扫描串口，`GetSysInfo` 探测，GUI 内输入激活码即可
- 🌡️ **可选滤波** — 1D 卡尔曼对 Tmax/Tmin/Tavg、2D 像素级卡尔曼对整帧、OpenCV 双边滤波（默认关）
- 📈 **滚动温度曲线** — Tmax/Tmin/Tavg 100 点窗口，节流 5 帧重绘
- 🖥️ **高 DPI 自适应** — `SetProcessDpiAwareness(1)` + 强制 `tk scaling 1.50`，所有几何运行时缩放
- 💬 **内置命令行** — 直接发原始命令（`stream` / `vstream` / `GetSysInfo` / `activate <key>` ……），带分级彩色日志
- 📦 **一键打包** — GitHub Actions 在每次 push 时构建单文件 Windows EXE，PyInstaller `_MEIPASS` 自动内嵌字体

---

## 📡 Protocol / 协议

Both streams share one 1 Mbps serial link. The parser is a strict state machine.

### Thermal frame (3092 bytes)

```
BEGIN | T_max(4) | T_min(4) | T_avg(4) | float[768] (3072) | END
```

- All floats little-endian IEEE 754 / 全部小端 IEEE 754
- 768 = 24 × 32 pixels
- Trigger: `stream\n`. Keep-alive: 500 ms. Device timeout: 1 s (`streaming stoped`).

### Visible frame (variable)

```
VBEG | width(4) | height(4) | length(4) | RGB565[length] | VEND
```

- Width 120, height 160 (full-screen) or 120 (square) / 宽 120，高 160 或 120
- All `u32` little-endian
- Trigger: `vstream\n`. Same 500 ms keep-alive, 1 s timeout (`vstream stoped`).

---

## 🚀 Quick Start / 快速开始

### Option A — Download pre-built EXE / 直接下载预编译版

Go to [Releases](https://github.com/applenana/BananaThermal-Studio/releases) and grab `BananaThermal-Studio.exe`. Single file, no install, fonts bundled.

> 直接下载 → 双击运行，免安装，字体已内嵌。

### Option B — Run from source / 源码运行

```bash
git clone https://github.com/applenana/BananaThermal-Studio.git
cd BananaThermal-Studio
python -m venv .venv

# Windows
.venv\Scripts\activate
# Linux / macOS
source .venv/bin/activate

pip install -r requirements.txt
python thermal_dual_app.py
```

The app auto-scans serial ports and pops the activation panel on first connect.
启动后自动扫描串口，未激活设备会弹出激活面板。

---

## 🖥️ UI Overview / 界面说明

```
┌─ Top bar (port / status / device info) ──────────────────────────┐
├─ Tab 1: 实时投屏 ─────────────────────────────────────────────────┤
│  ┌───────────────────┐ ┌─ 实时温度 (Max/Min/Avg) ──────┐         │
│  │                   │ ├─ 温度曲线 (滚动 100 pts) ─────┤         │
│  │  融合主画面       │ ├─ 显示控制 (4 个滤波/投屏开关) ┤         │
│  │  (rendered RGB)   │ ├─ 颜色映射 (曲线/调色盘/三色) ─┤         │
│  │                   │ └─ 可见光融合 (模式/参数) ─────┘         │
│  └───────────────────┘                                           │
│  └─ 串口调试 (raw command + colored log) ────────────────────────┤
├─ Tab 2: 图片下载 (片上 JPEG 拉取与重渲染) ────────────────────────┤
└──────────────────────────────────────────────────────────────────┘
```

---

## 🗂️ Project Layout / 项目结构

```
BananaThermal-Studio/
├─ thermal_dual_app.py        # Main entry: Notebook + realtime fusion (主入口)
├─ visible_view.py            # Reusable visible-light widget (可见光控件)
├─ fusion_utils.py            # colorize() + fuse() pure functions
├─ photo_download_tab.py      # 图片下载 tab (片上 JPEG 拉取与重渲染)
├─ frame_parser.py            # Pure-Python protocol parser (协议解析)
├─ requirements.txt
├─ thermal_dual_app.spec      # PyInstaller spec (auto-bundles fonts)
├─ SmileySans-Oblique.ttf     # Bundled font for matplotlib zh rendering
└─ .github/workflows/
   └─ build-exe.yml           # CI: push → artifact, tag v* → release
```

---

## 🛠️ Tech Stack

| Layer | Choice |
|---|---|
| GUI | `tkinter` + `ttk` (Notebook) |
| Plotting | `matplotlib` (TkAgg) |
| Image | `Pillow`, `opencv-python` (bilateral / fusion) |
| Serial | `pyserial` 3.5+ |
| Build | `PyInstaller` 6.11+ (single-file, `_MEIPASS` font bundling) |
| CI | GitHub Actions (Windows runner) |

---

## 📦 Build EXE locally / 本地打包

```bash
pip install pyinstaller>=6.11
pyinstaller --noconfirm --clean thermal_dual_app.spec
# → dist/BananaThermal-Studio.exe
```

The spec auto-collects every `.ttf` / `.otf` / `.ttc` in repo root and `font/` `fonts/` subfolders. The runtime resolves them via `sys._MEIPASS`.
spec 会把根目录与 `font/` `fonts/` 下的字体都打包，运行时通过 `sys._MEIPASS` 解析。

### Release flow / 发版流程

```bash
git tag v0.1.0
git push origin v0.1.0
# → Actions builds + creates GitHub Release automatically
```

---

## 🤝 Contributing / 贡献

- New protocol fields → extend `FrameParser` + add a self-test case
- New display modes → extend `VisibleView` or `fusion_utils`, do **not** poke `thermal_dual_app.py` threads
- GUI tweaks → respect `_scaled()` / `_scaled_geom()` helpers for DPI scaling

新增协议字段请扩展 `FrameParser` 并补单测；新增显示模式优先扩展 `VisibleView` 或 `fusion_utils`，避免直接动主线程；GUI 改动遵守 `_scaled()` 缩放约定。

---

## 📜 License

MIT © 2025 BananaNi · applenana

## 🙏 Acknowledgements

- **Heimann Sensor** — HTPA32x32d datasheet
- **OmniVision** — OV2640 / OV3660 reference
- **Smiley Sans** font — bundled for matplotlib zh rendering

---

<p align="center">
  Made with 🍌 for fellow tinkerers / 给同好的香蕉味玩具
</p>
