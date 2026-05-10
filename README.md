# Heimann 双光红外测温仪 — 全能上位机

> 适配 RP2040 + Heimann HTPA32x32d 热成像 + OV2640/OV3660 可见光双光融合的开源测温仪
>
> 基于设备固件 `streaming.h` 模块实现的串口推流协议, 同时接收**热成像帧**与**可见光帧**, 实时显示并支持设备激活、命令调试。

---

## 功能概览

- **双光实时投屏**
  - 热成像 32×24 浮点温度帧 (固件 `stream` 命令)
  - 可见光 120×160 RGB565 帧 (固件 `vstream` 命令)
  - 两路心跳独立, 用户可单开/双开/全关
- **设备管理**: 自动搜索串口, 读取设备 SN/版本/激活状态
- **激活流程**: 校验激活码, 持久化激活时间与保修期
- **命令调试**: 收发任意串口命令, 实时显示设备返回
- **温度曲线**: 实时绘制 max/min/avg 温度历史折线图

---

## 目录结构

```
全能上位机/
├── README.md                   # 本文件
├── requirements.txt            # Python 依赖
├── .gitignore
├── .venv/                      # 虚拟环境 (本地, git 忽略)
├── frame_parser.py             # 串口帧解析模块 (BEGIN/END + VBEG/VEND)
├── visible_view.py             # 可见光显示组件 (Tkinter Canvas + PIL)
├── thermal_dual_app.py         # 主程序 (Tkinter GUI 入口)
└── font/                       # (可选) HarmonyOS 字体, 改善中文显示
```

---

## 协议规范

> 与设备固件 `src/streaming.h` 字节级一致。所有多字节字段为**小端**。

### A. 热成像帧 (向后兼容旧固件)

| 段 | 大小 | 内容 |
|---|---|---|
| Magic | 5 B | ASCII `"BEGIN"` |
| T_max | 4 B | float32, 单位 °C |
| T_min | 4 B | float32, 单位 °C |
| T_avg | 4 B | float32, 单位 °C |
| Pixels | 3072 B | float32 × 768 (24 行 × 32 列, 行优先) |
| Magic | 3 B | ASCII `"END"` |

### B. 可见光帧 (固件 v2 新增)

| 段 | 大小 | 内容 |
|---|---|---|
| Magic | 4 B | ASCII `"VBEG"` |
| width | 4 B | uint32, 通常 120 |
| height | 4 B | uint32, 全屏模式 160 / 方屏模式 120 |
| len | 4 B | uint32, payload 字节数 = width × height × 2 |
| payload | len B | RGB565 像素, 行优先, 每像素小端 uint16 |
| Magic | 4 B | ASCII `"VEND"` |

### C. 控制命令 (上位机 → 设备)

| 命令 | 作用 | 保活 |
|---|---|---|
| `stream\n` | 启动热成像推流 | 1000ms 内必须重发 |
| `vstream\n` | 启动可见光推流 | 1000ms 内必须重发 |
| 其他命令 | 见固件 `help` 命令 | — |

> ⚠️ 设备只在收到对应保活命令的 1s 时间窗内推流, 上位机需开独立心跳线程。

---

## 快速开始

### 1. 创建虚拟环境

```powershell
cd D:\Github_project\全能上位机
py -3 -m venv .venv
.\.venv\Scripts\Activate.ps1
```

### 2. 安装依赖

```powershell
pip install -r requirements.txt
```

### 3. 运行

```powershell
python thermal_dual_app.py
```

启动后界面会自动搜索 USB 串口。设备插入后约 2 秒内自动连接并显示 SN/激活状态。

---

## 二次开发提示

- `frame_parser.py` **零 GUI 依赖**, 可直接复用到任何后端项目 (服务器、SDK、自动化测试)。
- `visible_view.py` 单独的 Tkinter 组件, 可拆出嵌入其他 GUI。
- 主程序 `thermal_dual_app.py` 中的滤波/绘图函数都是独立方法, 关注点清晰。
- 想增加帧类型? 只需在 `frame_parser.FrameParser` 状态机里加一个 magic 分支, 主程序注册对应回调即可。

---

## 兼容性

- 设备固件: **必须 ≥ v2** (引入 `streaming.h` 模块的版本) 才支持 `vstream`
- 旧固件: 仅热成像功能可用, 可见光开关无效但不报错
- Python: 3.10+ (使用了 `match` 语句 / typing 改进)
- 操作系统: Windows / Linux / macOS (USB CDC 跨平台一致)
