"""
设备表：每台设备（手套 A、手套 B、机械手 H）用哪个串口、哪个模型、哪个标定文件。改串口只改 config/devices.json。
标定文件：A -> config/calibration.json，其他 -> config/calibration_<名字>.json（dxl_bus.calib_path 的约定）。
"""
import json
import os
import sys

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
FILE = os.path.join(ROOT, "config", "devices.json")


def load():
    return json.load(open(FILE, encoding="utf-8"))


def get(name):
    devs = load()
    if name not in devs:
        options = ", ".join(f"{k}（{v['desc']}）" for k, v in devs.items())
        sys.exit(f"不认识的设备 {name}，可选: {options}")
    return dict(devs[name], name=name)
