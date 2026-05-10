# 🍌 BananaThermal Studio

> A modular PC host (上位机) for **BananaNi Thermal Imager** — the open-source RP2040 + Heimann HTPA dual-light (thermal + visible) imaging device.
>
> 适配 **香蕉泥热成像通讯协议** 的开源 PC 端上位机, 支持热成像帧 + 可见光帧实时双光显示、温度曲线、设备激活与串口调试.

<p align="center">
  <img alt="python" src="https://img.shields.io/badge/python-3.10%2B-blue">
  <img alt="platform" src="https://img.shields.io/badge/platform-Windows%20%7C%20Linux%20%7C%20macOS-lightgrey">
  <img alt="license" src="https://img.shields.io/badge/license-MIT-green">
  <img alt="status" src="https://img.shields.io/badge/status-active-brightgreen">
</p>

---

## ✨ Features

- 🔥 **Dual-Light Live Streaming** — Thermal (32×24 float) + Visible (RGB565 120×160 / 120×120) side-by-side, parsed from a single serial byte stream
- 🧠 **Stateful Frame Parser** — Zero-GUI [`frame_parser.py`](frame_parser.py) splits the multiplexed stream by `BEGIN/END` and `VBEG/VEND` magic markers, fully unit-testable
- 🎛️ **Hot-Plug Auto-Detect** — Scans serial ports, queries `GetSysInfo`, distinguishes activated vs. unactivated devices
- 🔐 **Activation Workflow** — Compatible with the legacy 激活上位机 protocol (`activate <key>`)
- 🌡️ **Filtering** — 1-D Kalman on T_max / T_min / T_avg, 2-D per-pixel Kalman on the 24×32 frame, optional bilateral filter (OpenCV)
- 📈 **Real-time Chart** — Rolling 100-point min/max/avg curves via Matplotlib
- 🔄 **Visible-Light Controls** — Per-display rotation (0/90/180/270, default 90°), horizontal/vertical mirror, FPS readout
- 🖥️ **High-DPI Aware** — `PROCESS_PER_MONITOR_DPI_AWARE` on Windows + adjustable global font scaling
- 💬 **Built-in Console** — Send raw commands (`stream`, `vstream`, `GetSysInfo`, `activate <key>`, …) directly from the GUI

## 📡 Protocol — BananaNi Thermal Wire Format

Both streams are multiplexed on a single 1 Mbps serial link. The parser is a strict state machine.

### Thermal frame (3092 bytes)

```
BEGIN | T_max(4) | T_min(4) | T_avg(4) | float[768] (3072) | END
```

- All floats are little-endian IEEE 754
- 768 = 24 × 32 pixels
- Triggered by `stream\n`, kept alive by 500 ms re-sends, device stops on 1 s silence (returns `streaming stoped`)

### Visible frame (variable)

```
VBEG | width(4) | height(4) | length(4) | RGB565[length] | VEND
```

- Width fixed = 120, height = 160 (full-screen mode) or 120 (square mode)
- All u32 are little-endian
- Triggered by `vstream\n`, same 500 ms keepalive, 1 s timeout (`vstream stoped`)

> See [3inch2-Heimann-dual-thermal/src/streaming.h](https://github.com/) for the firmware-side reference implementation.

## 🚀 Quick Start

### 1. Install

```bash
git clone https://github.com/<you>/banana-thermal-studio.git
cd banana-thermal-studio
python -m venv .venv
# Windows
.venv\Scripts\activate
# Linux / macOS
source .venv/bin/activate
pip install -r requirements.txt
```

### 2. Run

```bash
python thermal_dual_app.py
```

The app auto-scans serial ports and pops up the activation panel on first connect.

### 3. Adjust UI scale (optional)

```bash
# Windows PowerShell
$env:THERMAL_DUAL_FONT_SIZE = "14"     # default 12
$env:THERMAL_DUAL_SCALE     = "1.8"    # tk scaling, default 2.0
python thermal_dual_app.py
```

## 🗂️ Project Layout

```
banana-thermal-studio/
├─ thermal_dual_app.py     # Main GUI (Tkinter + Matplotlib)
├─ visible_view.py         # Reusable visible-light display widget
├─ frame_parser.py         # Pure-Python protocol parser (zero GUI deps)
├─ requirements.txt        # numpy / matplotlib / pillow / opencv-python / pyserial
└─ font/                   # Bundled HarmonyOS Sans SC for matplotlib zh rendering
```

## 🧪 Self-Test

The parser ships with a built-in round-trip test:

```bash
python frame_parser.py
# -> [self-test] thermal frames decoded: 1
#    [self-test] visible frames decoded: 1
#    [self-test] PASSED
```

## 🛠️ Tech Stack

| Layer | Choice |
|---|---|
| GUI | `tkinter` + `ttk` |
| Plotting | `matplotlib` (TkAgg backend) |
| Image | `Pillow`, optional `opencv-python` for bilateral filter |
| Serial | `pyserial` 3.5+ |
| Threading | 1 reader thread + 1 monitor thread + 2 keepalive threads + main-thread queue dispatch |

## 🤝 Contributing

Issues and PRs are welcome. Please follow the existing module split:

- New protocol fields → extend `FrameParser` and add a self-test case
- New display modes → subclass / extend `VisibleView`, do **not** poke at `thermal_dual_app.py`'s threads directly
- GUI tweaks → keep the layout responsive to `_win_ratio` font scaling

## 📜 License

MIT © 2025 BananaNi

## 🙏 Acknowledgements

- Heimann Sensor — HTPA32x32d datasheet
- OmniVision — OV2640 / OV3660 reference
- Earthquake Bone Cooker / 香蕉泥 — original 激活上位机 protocol baseline

---

<p align="center">
  Made with 🍌 for fellow tinkerers.
</p>
