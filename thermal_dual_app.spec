# -*- mode: python ; coding: utf-8 -*-
"""PyInstaller spec for BananaThermal Studio (thermal_dual_app)
打包成单文件 exe, 内嵌字体 (根目录 *.ttf/*.otf).
"""
from PyInstaller.utils.hooks import collect_submodules
import os

ROOT = os.path.abspath(os.path.dirname(SPEC) if 'SPEC' in dir() else '.')

# 把根目录里所有字体文件平铺到 _MEIPASS 根
datas = []
for fn in os.listdir(ROOT):
    if fn.lower().endswith(('.ttf', '.otf', '.ttc')):
        datas.append((os.path.join(ROOT, fn), '.'))

# 若存在 font/fonts 子目录也带上 (保持子目录结构, 与 _find_bundled_font 搜索一致)
for sub in ('font', 'fonts'):
    sub_dir = os.path.join(ROOT, sub)
    if os.path.isdir(sub_dir):
        for fn in os.listdir(sub_dir):
            if fn.lower().endswith(('.ttf', '.otf', '.ttc')):
                datas.append((os.path.join(sub_dir, fn), sub))

hiddenimports = []
hiddenimports += collect_submodules('matplotlib.backends')

a = Analysis(
    ['thermal_dual_app.py'],
    pathex=[ROOT],
    binaries=[],
    datas=datas,
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=['tkinter.test', 'unittest', 'pydoc_data'],
    noarchive=False,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name='BananaThermal-Studio',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=False,        # GUI 程序, 不显示控制台
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)
