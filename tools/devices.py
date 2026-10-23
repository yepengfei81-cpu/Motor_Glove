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


def find_tty(product=None, driver=None):
    """按 USB 产品名（如 OpenRB-150）或驱动（ftdi_sio）找串口；找不到返回 None"""
    import glob
    for d in sorted(glob.glob("/sys/class/tty/tty*")):
        dev = os.path.join(d, "device")
        if not os.path.exists(dev):
            continue
        try:
            if product and open(os.path.join(dev, "..", "product")).read().strip() == product:
                return "/dev/" + os.path.basename(d)
            if driver and os.path.basename(os.path.realpath(os.path.join(dev, "driver"))) == driver:
                return "/dev/" + os.path.basename(d)
        except OSError:
            continue
    return None


ALIASES = {"openrb": ("OpenRB-150", None), "u2d2": (None, "ftdi_sio")}


def resolve_port(port):
    """
    devices.json 里可以写 /dev/openrb、/dev/u2d2（udev 规则 tools/99-dxl-names.rules 装了就直接存在）；
    没装规则时按 USB 产品名 / 驱动自动找。普通 /dev/ttyACM0 这类照旧，不存在时给出提示。
    """
    if os.path.exists(port):
        return port
    key = os.path.basename(port)
    if key in ALIASES:
        found = find_tty(*ALIASES[key])
        if found:
            return found
        sys.exit(f"没找到 {key}（{'OpenRB-150' if key == 'openrb' else 'U2D2'}）：线插了吗？")
    hint = find_tty("OpenRB-150")
    sys.exit(f"串口 {port} 不存在" + (f"；OpenRB 现在在 {hint}，改 config/devices.json 或者装 tools/99-dxl-names.rules 固定设备名" if hint else ""))


def get(name):
    devs = load()
    if name not in devs:
        options = ", ".join(f"{k}（{v['desc']}）" for k, v in devs.items())
        sys.exit(f"不认识的设备 {name}，可选: {options}")
    d = dict(devs[name], name=name)
    d["port"] = resolve_port(d["port"])
    return d
