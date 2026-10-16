#!/usr/bin/env python3
"""
改电机的输入电压保护范围（EEPROM 地址 32 最高、34 最低，单位 0.1 V）。
用途：XC330-T 系列是 12 V 电机，出厂最低电压保护 6.0 V，在 5–6 V 总线上会报"输入电压超限"拒绝开扭矩；把下限改到 3.5 V 就能用（力矩偏小）。

  python tools/dxl_voltage_limit.py 2 --port /dev/ttyACM0            # 只读 ID 2 的当前设置和电压
  python tools/dxl_voltage_limit.py 2 --port /dev/ttyACM0 --min 3.5  # 改最低电压保护为 3.5 V，然后 reboot 清错误
"""
import argparse
import sys
import time
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS  # noqa: E402
from dxl_bus import default_port, MODEL_NAME  # noqa: E402

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("id", type=int)
ap.add_argument("--port", default=None)
ap.add_argument("--min", type=float, default=None, help="新的最低电压保护 V")
ap.add_argument("--max", type=float, default=None, help="新的最高电压保护 V")
a = ap.parse_args()
port = PortHandler(a.port or default_port()); port.openPort(); port.setBaudRate(1000000)
pkt = PacketHandler(2.0)


def r(addr, n):
    v, res, err = {1: pkt.read1ByteTxRx, 2: pkt.read2ByteTxRx}[n](port, a.id, addr)
    if res != COMM_SUCCESS:
        sys.exit(f"读 {addr} 失败: {pkt.getTxRxResult(res)}")
    return v


def show():
    model = r(0, 2); vmax, vmin, v, hw, tq = r(32, 2), r(34, 2), r(144, 2), r(70, 1), r(64, 1)
    print(f"ID {a.id} {MODEL_NAME.get(model, model)}：保护范围 {vmin/10:.1f}–{vmax/10:.1f} V，当前电压 {v/10:.1f} V，扭矩 {'开' if tq else '关'}，"
          f"硬件错误 0x{hw:02X}" + ("（输入电压超限）" if hw & 1 else "") + ("（过载）" if hw >> 5 & 1 else ""))
    return vmin, vmax, hw


vmin, vmax, hw = show()
if a.min is not None or a.max is not None:
    if hw:                                                      # 错误标志挂着时 EEPROM 写会被拒绝：先 reboot 清掉，趁它还没重新检测出低压赶紧写
        pkt.reboot(port, a.id); time.sleep(0.25)
    pkt.write1ByteTxRx(port, a.id, 64, 0)                      # EEPROM 要在扭矩关时写
    if a.min is not None:
        res, err = pkt.write2ByteTxRx(port, a.id, 34, int(round(a.min * 10)))
        print(f"写最低电压 {a.min:.1f} V: {'OK' if res == COMM_SUCCESS and not err else pkt.getTxRxResult(res) + ' ' + pkt.getRxPacketError(err)}")
    if a.max is not None:
        res, err = pkt.write2ByteTxRx(port, a.id, 32, int(round(a.max * 10)))
        print(f"写最高电压 {a.max:.1f} V: {'OK' if res == COMM_SUCCESS and not err else pkt.getTxRxResult(res) + ' ' + pkt.getRxPacketError(err)}")
    pkt.reboot(port, a.id); time.sleep(0.6)                      # reboot 清掉错误标志
    show()
port.closePort()
