#!/usr/bin/env python3
"""
总线通信基准：一条总线上几个电机，量"一个控制周期到底花在哪"。不开扭矩，只读写寄存器。

  python tools/bench_bus.py --port /dev/ttyUSB0                     # 自动找电机（1 Mbps），跑全部测试
  python tools/bench_bus.py --port /dev/ttyACM0 --baud 2000000      # 电机已经改成 2 Mbps 的话
  python tools/bench_bus.py --port /dev/ttyUSB0 --set-baud 2000000  # 把总线上的电机改成 2 Mbps（EEPROM），然后用 --baud 2000000 再跑
  python tools/bench_bus.py --port /dev/ttyUSB0 --emulate 13        # 用大块读模拟 13 个电机的线上数据量
  python tools/bench_bus.py --port /dev/ttyUSB0 --hz 500            # 定节拍环的抖动

测试项：
  1. SDK GroupSyncRead（现在 dxl_bus 用的）一次往返
  2. 精简包（自己拼包、查表 CRC、pyserial 直读）的 Sync Read 一次往返  -> SDK 的 Python 开销
  3. Fast Sync Read (0x8A)：所有电机一个状态包              -> 包头和包间隔的开销（固件要支持）
  4. 大块读模拟 N 个电机的线上时间
  5. 读 + 写（Sync Write 目标电流 0）全速环：频率、p50/p99/max
  6. 定节拍环（--hz）：抖动直方图
"""
import argparse
import os
import struct
import sys
import time

import numpy as np
import serial

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

BAUD_CODE = {9600: 0, 57600: 1, 115200: 2, 1000000: 3, 2000000: 4, 3000000: 5, 4000000: 6}
ADDR_PRESENT_CURRENT = 126      # 电流(2) 速度(4) 位置(4) = 10 字节
ADDR_GOAL_CURRENT = 102

# ---- Protocol 2.0 CRC-16 ----
_CRC = []
for i in range(256):
    c = i << 8
    for _ in range(8):
        c = ((c << 1) ^ 0x8005) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    _CRC.append(c)


def crc16(data):
    c = 0
    for b in data:
        c = ((c << 8) ^ _CRC[((c >> 8) ^ b) & 0xFF]) & 0xFFFF
    return c


def packet(dxl_id, inst, params=b""):
    body = bytes([0xFF, 0xFF, 0xFD, 0x00, dxl_id]) + struct.pack("<H", len(params) + 3) + bytes([inst]) + params
    return body + struct.pack("<H", crc16(body))


def pct(xs):
    a = np.array(xs) * 1000
    return f"均值 {a.mean():6.3f}  p50 {np.percentile(a, 50):6.3f}  p99 {np.percentile(a, 99):6.3f}  最大 {a.max():6.3f} ms"


class Lean:
    """精简包层：预拼指令包，pyserial 直读，不建对象"""

    def __init__(self, port, baud, ids, timeout=0.05):
        self.ser = serial.Serial(port, baud, timeout=timeout, write_timeout=timeout)
        self.ids = list(ids)
        self.n = len(ids)

    def close(self):
        self.ser.close()

    def _xfer(self, pkt, expect):
        self.ser.reset_input_buffer()
        self.ser.write(pkt)
        buf = self.ser.read(expect)
        if len(buf) != expect:
            raise IOError(f"期望 {expect} 字节，收到 {len(buf)}")
        return buf

    def sync_read(self, addr, length):
        """普通 Sync Read：每个电机回一个状态包（11 + length 字节）"""
        pkt = packet(0xFE, 0x82, struct.pack("<HH", addr, length) + bytes(self.ids))
        buf = self._xfer(pkt, self.n * (11 + length))
        out, k = {}, 0
        for _ in range(self.n):
            seg = buf[k: k + 11 + length]
            if seg[:4] != b"\xff\xff\xfd\x00" or seg[7] != 0x55:
                raise IOError("状态包格式不对")
            out[seg[4]] = (seg[8], seg[9: 9 + length])
            k += 11 + length
        return out

    def fast_sync_read(self, addr, length):
        """Fast Sync Read (0x8A)：一个状态包：... 0x55 | ERR1 ID1 DATA1 CRC1 | ERR2 ID2 DATA2 CRC2 | ...（XL330 实测格式）"""
        pkt = packet(0xFE, 0x8A, struct.pack("<HH", addr, length) + bytes(self.ids))
        self.ser.reset_input_buffer()
        self.ser.write(pkt)
        head = self.ser.read(7)
        if len(head) != 7 or head[:4] != b"\xff\xff\xfd\x00":
            raise IOError(f"没有回应或格式不对（收到 {len(head)} 字节）")
        ln = struct.unpack("<H", head[5:7])[0]
        rest = self.ser.read(ln)
        if len(rest) != ln:
            raise IOError(f"状态包不完整：期望 {ln} 收到 {len(rest)}")
        if rest[0] != 0x55:
            raise IOError("不是状态包")
        if rest[1] & 0x7F == 0x02:
            raise IOError("电机回了 Instruction Error：固件不支持 Fast Sync Read")
        seg = length + 4                                   # 每个电机：ERR ID DATA CRC(2)，最后一个的 CRC 就是整包的 CRC（实测 1 电机 10 字节 = 15）
        if ln != 1 + self.n * seg:
            raise IOError(f"长度 {ln} 和预期 {1 + self.n * seg} 不符（固件格式不同？原始: {rest.hex()}）")
        out, k = {}, 1
        for _ in range(self.n):
            err, i = rest[k], rest[k + 1]
            out[i] = (err, rest[k + 2: k + 2 + length])
            k += seg
        return out

    def sync_write_current(self, ma):
        params = struct.pack("<HH", ADDR_GOAL_CURRENT, 2) + b"".join(bytes([i]) + struct.pack("<h", int(ma.get(i, 0))) for i in self.ids)
        self.ser.write(packet(0xFE, 0x83, params))


def find_ids(port, baud):
    from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
    ph = PortHandler(port); ph.openPort(); ph.setBaudRate(baud); pk = PacketHandler(2.0)
    ids = []
    for i in range(1, 30):
        _, res, _ = pk.ping(ph, i)
        if res == COMM_SUCCESS:
            ids.append(i)
    ph.closePort()
    return ids


def set_baud(port, baud, ids, new_baud):
    from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
    ph = PortHandler(port); ph.openPort(); ph.setBaudRate(baud); pk = PacketHandler(2.0)
    for i in ids:
        pk.write1ByteTxRx(ph, i, 64, 0)
        pk.write1ByteTxRx(ph, i, 9, 0)                                 # Return Delay 0
        res, err = pk.write1ByteTxRx(ph, i, 8, BAUD_CODE[new_baud])
        print(f"  ID {i} 波特率 -> {new_baud}: {'ok' if res == COMM_SUCCESS and not err else pk.getTxRxResult(res)}")
    ph.closePort()
    time.sleep(0.3)
    ph = PortHandler(port); ph.openPort(); ph.setBaudRate(new_baud); pk = PacketHandler(2.0)
    ok = [i for i in ids if pk.ping(ph, i)[1] == COMM_SUCCESS]
    ph.closePort()
    print(f"* 在 {new_baud} 下能 ping 到: {ok}")


def latency_timer(port):
    name = os.path.basename(port)
    p = f"/sys/bus/usb-serial/devices/{name}/latency_timer"
    return open(p).read().strip() + " ms" if os.path.exists(p) else "（不是 FTDI，没有这个设置）"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--ids", type=int, nargs="*", default=None)
    ap.add_argument("--n", type=int, default=1000, help="每项测试的次数")
    ap.add_argument("--emulate", type=int, default=0, help="用大块读模拟这么多个电机的线上数据量")
    ap.add_argument("--hz", type=float, default=0, help="定节拍环测试的目标频率（0 = 不做）")
    ap.add_argument("--set-baud", type=int, default=None, choices=sorted(BAUD_CODE), help="把电机改到这个波特率后退出")
    ap.add_argument("--set-id", type=int, default=None, help="总线上只有一个电机时，把它的 ID 改成这个后退出（两台电机要接同一条总线前用）")
    ap.add_argument("--as-device", default=None, metavar="NAME",
                    help="把这条总线上唯一的电机登记成一台『单关节设备』（写 config/devices.json 和 calibration_NAME.json；交互式标零点和方向），"
                         "之后可以 mg.py teleop T1 T2 thumb 用台架跑遥操作代码")
    ap.add_argument("--joint", default="thumb_cmc_abd_motor", help="--as-device 时这个电机扮演哪个关节（电机 ID 会改成该关节在 motors.json 里的 ID）")
    args = ap.parse_args()

    print(f"* 串口 {args.port} @ {args.baud}，USB latency timer: {latency_timer(args.port)}")
    ids = args.ids or find_ids(args.port, args.baud)
    if not ids:
        for b_ in (57600, 1000000, 2000000, 115200, 3000000, 4000000):
            if b_ == args.baud:
                continue
            found = find_ids(args.port, b_)
            if found:
                sys.exit(f"在 {args.baud} 下没找到电机，但在 {b_} 下找到了 {found}（出厂是 57600）。"
                         f"\n  改到想测的波特率：python tools/bench_bus.py --port {args.port} --baud {b_} --set-baud {args.baud}")
        sys.exit("没找到电机：检查供电（U2D2 自己不供电，要接电源板）和线")
    print(f"* 电机 ID: {ids}")
    # Return Delay 不是 0 的先改掉（EEPROM，扭矩关着）
    from dynamixel_sdk import PortHandler as _PH, PacketHandler as _PK
    _ph = _PH(args.port); _ph.openPort(); _ph.setBaudRate(args.baud); _pk = _PK(2.0)
    for i in ids:
        v, _, _ = _pk.read1ByteTxRx(_ph, i, 9)
        if v:
            _pk.write1ByteTxRx(_ph, i, 64, 0); _pk.write1ByteTxRx(_ph, i, 9, 0)
            print(f"* ID {i} Return Delay {v * 2} µs -> 0")
    _ph.closePort()
    if args.set_baud:
        set_baud(args.port, args.baud, ids, args.set_baud)
        return
    if args.as_device:
        import json
        from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import glove_model as gm
        from dxl_bus import load_registry, save_calibration, ROOT
        if len(ids) != 1:
            sys.exit(f"总线上有 {len(ids)} 个电机，登记设备时只能接一个")
        reg = load_registry(); want = reg[args.joint]
        ph = PortHandler(args.port); ph.openPort(); ph.setBaudRate(args.baud); pk = PacketHandler(2.0)
        pk.write1ByteTxRx(ph, ids[0], 64, 0)
        if ids[0] != want:
            res, err = pk.write1ByteTxRx(ph, ids[0], 7, want)
            print(f"* 电机 ID {ids[0]} -> {want}（{args.joint} 在 motors.json 里的 ID）: {'ok' if res == COMM_SUCCESS and not err else '失败'}")
        def readpos():
            v, res, _ = pk.read4ByteTxRx(ph, want, 132)
            if res != COMM_SUCCESS:
                sys.exit("读位置失败")
            return v - (1 << 32) if v >= (1 << 31) else v
        input(f"  [{args.as_device}] 把杆摆到零位（两台设备用同一个物理姿势，比如都竖直朝上），扶住按回车 ... ")
        pos = readpos()
        input(f"  [{args.as_device}] 把杆往『正方向』转 20° 以上（两台设备用同一个物理方向，比如从上往下看都顺时针），扶住按回车 ... ")
        d = (readpos() - pos + 2048) % 4096 - 2048
        if abs(d) < 100:
            sys.exit(f"只转了 {d * 360 / 4096:.1f}°，太小，重来")
        sign = 1 if d > 0 else -1
        print(f"  编码器{'增大' if d > 0 else '减小'} {abs(d) * 360 / 4096:.1f}° -> 方向 {sign:+d}")
        ph.closePort()
        axes = gm.joint_axes()
        save_calibration({"joints": {args.joint: {"id": want, "zero_raw": int(pos), "sign": sign, "axis_base": axes[args.joint]}}}, args.as_device)
        dpath = os.path.join(ROOT, "config", "devices.json")
        devs = json.load(open(dpath, encoding="utf-8"))
        alias = "/dev/openrb" if "ACM" in args.port else "/dev/u2d2" if "USB" in args.port else args.port
        devs[args.as_device] = {"port": alias, "model": "glove", "desc": f"台架电机 {args.as_device}（{args.joint}，{args.port}）"}
        json.dump(devs, open(dpath, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
        print(f"* 已登记设备 {args.as_device}：{alias}，关节 {args.joint}，零点 {pos}，方向 {sign:+d}，波特率 {args.baud}")
        return
    if args.set_id is not None:
        if len(ids) != 1:
            sys.exit(f"总线上有 {len(ids)} 个电机，改 ID 时只能接一个")
        from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
        ph = PortHandler(args.port); ph.openPort(); ph.setBaudRate(args.baud); pk = PacketHandler(2.0)
        pk.write1ByteTxRx(ph, ids[0], 64, 0)
        res, err = pk.write1ByteTxRx(ph, ids[0], 7, args.set_id)
        ok = pk.ping(ph, args.set_id)[1] == COMM_SUCCESS
        ph.closePort()
        print(f"* ID {ids[0]} -> {args.set_id}: {'ok' if res == COMM_SUCCESS and not err and ok else '失败'}")
        return
    n = args.n

    # 1. SDK GroupSyncRead
    from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncRead, COMM_SUCCESS
    ph = PortHandler(args.port); ph.openPort(); ph.setBaudRate(args.baud); pk = PacketHandler(2.0)
    gr = GroupSyncRead(ph, pk, ADDR_PRESENT_CURRENT, 10)
    for i in ids:
        gr.addParam(i)
    ts = []
    for _ in range(n):
        t = time.perf_counter(); r = gr.txRxPacket(); ts.append(time.perf_counter() - t)
        if r != COMM_SUCCESS:
            print("  SDK 读失败:", pk.getTxRxResult(r)); break
    print(f"1. SDK GroupSyncRead  {len(ids)} 电机 ×10 字节：{pct(ts)}")
    ph.closePort()

    L = Lean(args.port, args.baud, ids)
    # 2. 精简 Sync Read
    ts = []
    for _ in range(n):
        t = time.perf_counter(); L.sync_read(ADDR_PRESENT_CURRENT, 10); ts.append(time.perf_counter() - t)
    print(f"2. 精简 Sync Read     {len(ids)} 电机 ×10 字节：{pct(ts)}")
    # 3. Fast Sync Read
    try:
        ts = []
        for _ in range(n):
            t = time.perf_counter(); L.fast_sync_read(ADDR_PRESENT_CURRENT, 10); ts.append(time.perf_counter() - t)
        print(f"3. Fast Sync Read     {len(ids)} 电机 ×10 字节：{pct(ts)}")
        fast_ok = True
    except IOError as e:
        print(f"3. Fast Sync Read     不可用：{e}")
        fast_ok = False
    # 4. 模拟 N 个电机的数据量
    if args.emulate:
        total = args.emulate * (11 + 10)                       # N 个电机普通 Sync Read 的状态字节总数
        per = max(10, min(120, (total // len(ids)) - 11))      # 每个电机读这么多字节来凑
        ts = []
        for _ in range(n):
            t = time.perf_counter(); L.sync_read(20, per); ts.append(time.perf_counter() - t)
        print(f"4. 模拟 {args.emulate} 电机：{len(ids)} 电机各读 {per} 字节（≈ {len(ids) * (11 + per)} 字节 vs 真实 {total}）：{pct(ts)}")
        wire = total * 10 / args.baud * 1000
        print(f"   纯线上时间理论值 {wire:.2f} ms（{args.baud} bps）；Fast Sync Read 时约 {(8 + args.emulate * 14) * 10 / args.baud * 1000:.2f} ms")
    # 5. 全速读写环
    rd = L.fast_sync_read if fast_ok else L.sync_read
    ts = []
    for _ in range(n):
        t = time.perf_counter()
        rd(ADDR_PRESENT_CURRENT, 10); L.sync_write_current({})
        ts.append(time.perf_counter() - t)
    a = np.array(ts)
    print(f"5. 读({'fast' if fast_ok else 'sync'})+写 全速环：{1 / a.mean():5.0f} Hz   {pct(ts)}")
    # 6. 定节拍
    if args.hz > 0:
        period = 1 / args.hz; lat = []; t_next = time.perf_counter()
        for _ in range(n):
            t_next += period
            rd(ADDR_PRESENT_CURRENT, 10); L.sync_write_current({})
            now = time.perf_counter()
            lat.append(now - t_next)                     # 正 = 迟到
            while time.perf_counter() < t_next:
                pass
        a = np.array(lat) * 1000
        late = (a > 0.2).mean() * 100
        print(f"6. 定节拍 {args.hz:.0f} Hz：迟到 >0.2 ms 的周期占 {late:.1f}%，最大迟到 {a.max():.2f} ms"
              + ("   <- 跟不上，降频率或者减负载" if late > 5 else "   ok"))
    L.close()


if __name__ == "__main__":
    main()
