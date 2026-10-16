#!/usr/bin/env python3
"""
体检脚本（只读，不写任何寄存器）：把串口上所有 DYNAMIXEL 找出来并打印状态，用于检查拆来的二手电机。

  python tools/dxl_probe.py                       # 自动选串口（优先 U2D2 /dev/ttyUSB0，其次 OpenRB /dev/ttyACM0）
  python tools/dxl_probe.py --port /dev/ttyACM0
  python tools/dxl_probe.py --expect 4            # 告诉它你串了几个，少了会提示可能是重复 ID
  python tools/dxl_probe.py --quick               # 只用广播 ping，不逐个 ID 扫（快，但重复 ID 时可能漏）

每个电机会打印：型号（是不是 XL330-M288）、固件、ID、波特率、返回延迟、工作模式、驱动模式（反转位）、
零点偏移、电流上限、位置上下限、保护配置、扭矩开关、硬件错误、电压、温度、当前位置/电流，
再给一行结论：能不能直接用在手套上、需要改什么（ID 冲突、波特率、延迟、电压规格等）。
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS  # noqa: E402
from dynamixel_sdk import port_handler as _ph  # noqa: E402
_ph.LATENCY_TIMER = 3          # 失败的 ping 只等 ~8 ms（默认 16 ms 等一次要 35 ms，逐个 ID 扫太慢）
from dxl_bus import default_port, latency_warning, load_registry  # noqa: E402

BAUDS = [57600, 1000000, 115200, 9600, 2000000, 3000000, 4000000]
BAUD_NAME = {0: "9600", 1: "57600", 2: "115200", 3: "1M", 4: "2M", 5: "3M", 6: "4M", 7: "4.5M"}
MODELS = {1190: "XL330-M077", 1200: "XL330-M288", 1210: "XC330-T181", 1220: "XC330-T288", 1230: "XC330-M181", 1240: "XC330-M288",
          1060: "XL430-W250", 1090: "XL430-W250-2", 1020: "XM430-W350", 1030: "XM430-W210", 1120: "XM540-W150", 1130: "XM540-W270",
          350: "XL-320", 12: "AX-12A", 300: "AX-12W", 18: "AX-18A", 29: "MX-28", 310: "MX-64", 320: "MX-106"}
V_RANGE = {1190: (3.7, 6.0), 1200: (3.7, 6.0), 1210: (6.5, 12.0), 1220: (6.5, 12.0), 1230: (3.7, 6.0), 1240: (3.7, 6.0),
           350: (6.0, 8.4), 1060: (6.5, 12.0), 1090: (6.5, 12.0), 1020: (10.0, 14.8), 1030: (10.0, 14.8), 1120: (10.0, 14.8), 1130: (10.0, 14.8)}
X330 = {1190, 1200, 1210, 1220, 1230, 1240}       # 控制表和手套上的电机相同
X430 = {1060, 1090, 1020, 1030, 1120, 1130}       # 控制表地址相同，但 12 V 电机，不能用在 6 V 总线上
MODES = {0: "电流", 1: "速度", 3: "位置", 4: "多圈位置", 5: "电流限制的位置", 16: "PWM"}
HW_ERRORS = {0: "输入电压超限", 2: "过热", 3: "编码器", 4: "电气冲击", 5: "过载"}
SHUTDOWN_BITS = {0: "电压", 2: "过热", 3: "编码器", 4: "电气冲击", 5: "过载"}
GLOVE_IDS = set(range(1, 14))

# X 系列（330 / 430 / 540）控制表：名字 -> (地址, 字节数)
X_TABLE = {
    "firmware": (6, 1), "id": (7, 1), "baud": (8, 1), "return_delay": (9, 1), "drive_mode": (10, 1), "mode": (11, 1), "secondary_id": (12, 1),
    "protocol": (13, 1), "homing_offset": (20, 4), "temp_limit": (31, 1), "max_voltage": (32, 2), "min_voltage": (34, 2), "pwm_limit": (36, 2),
    "current_limit": (38, 2), "velocity_limit": (44, 4), "max_position": (48, 4), "min_position": (52, 4), "shutdown": (63, 1),
    "torque": (64, 1), "led": (65, 1), "hw_error": (70, 1), "pos_p": (84, 2), "goal_position": (116, 4),
    "present_current": (126, 2), "present_velocity": (128, 4), "present_position": (132, 4), "voltage": (144, 2), "temperature": (146, 1),
}


def signed(v, nbytes):
    bits = 8 * nbytes
    return v - (1 << bits) if v >= 1 << (bits - 1) else v


class Probe:
    def __init__(self, portname):
        self.portname = portname
        self.port = PortHandler(portname)
        if not self.port.openPort():
            sys.exit(f"打不开 {portname}（板子插上了吗？电机供电开了吗？）")
        self.p2 = PacketHandler(2.0)
        self.p1 = PacketHandler(1.0)

    def set_baud(self, baud):
        """换波特率（SDK 会重新打开串口），并给写操作加超时：板子的 USB 卡住时立刻报错，而不是永远等"""
        self.port.setBaudRate(baud)
        self.port.ser.write_timeout = 1.0
        self.port.ser.flush = lambda: None      # SDK 每次发包前 flush()（tcdrain），板子卡住时会永远等；去掉它，让 write 自己超时

    def close(self):
        try:
            self.port.ser.close()
        except Exception:
            pass

    def read(self, dxl_id, addr, n):
        fn = {1: self.p2.read1ByteTxRx, 2: self.p2.read2ByteTxRx, 4: self.p2.read4ByteTxRx}[n]
        v, res, err = fn(self.port, dxl_id, addr)
        return v if res == COMM_SUCCESS else None

    def scan_baud(self, baud, quick):
        """返回 {id: model}（Protocol 2.0）。广播 ping 之后，在常用波特率（57600 / 1M）和有发现的波特率上再逐个 ID 扫一遍"""
        self.set_baud(baud)
        found = {}
        res = self.p2.broadcastPing(self.port)[0]
        for i, v in res.items():
            found[i] = v[0]
        if not quick and (found or baud in (57600, 1000000)):
            for i in range(0, 253):
                if i in found:
                    continue
                v, res_i, _ = self.p2.ping(self.port, i)
                if res_i == COMM_SUCCESS:
                    found[i] = v
        return found

    def scan_p1(self, baud, ids=range(0, 21)):
        """Protocol 1.0（AX / 老 MX），只试常见 ID"""
        self.set_baud(baud)
        found = {}
        for i in ids:
            v, res, _ = self.p1.ping(self.port, i)
            if res == COMM_SUCCESS:
                found[i] = v
        return found

    def stability(self, dxl_id, n=8):
        """连续读 n 次型号，失败次数 > 0 多半是两个电机同 ID 在抢总线"""
        bad = 0
        for _ in range(n):
            if self.read(dxl_id, 0, 2) is None:
                bad += 1
        return bad

    def dump_x(self, dxl_id):
        d = {}
        for k, (addr, n) in X_TABLE.items():
            v = self.read(dxl_id, addr, n)
            if v is not None and k in ("homing_offset", "present_velocity", "present_position", "goal_position", "present_current"):
                v = signed(v, n)
            d[k] = v
        return d


def report(model, baud, dxl_id, d, reg_ids, bad_reads):
    name = MODELS.get(model, f"未知型号 {model}")
    print(f"\n==== ID {dxl_id} @ {baud} bps  {name}  (固件 v{d.get('firmware')}) ====")
    if bad_reads:
        print(f"  !! 连续读 8 次有 {bad_reads} 次失败：很可能有另一个电机也是 ID {dxl_id}，请拆开单独接")
    g = lambda k: d.get(k)
    mode = g("mode"); dm = g("drive_mode") or 0
    print(f"  通信   : ID {g('id')}，波特率 {BAUD_NAME.get(g('baud'), g('baud'))}，返回延迟 {g('return_delay')} ×2 µs，"
          f"Secondary ID {g('secondary_id')}，协议 {g('protocol')}")
    print(f"  模式   : {MODES.get(mode, mode)}（{mode}），驱动模式 0x{dm:02X}"
          + ("，方向反转" if dm & 1 else "") + ("，时间轨迹" if dm & 4 else "") + ("，扭矩开即使能 LED" if dm & 8 else ""))
    print(f"  零点   : Homing Offset {g('homing_offset')} ticks；位置范围 {g('min_position')} … {g('max_position')}")
    print(f"  上限   : 电流 {g('current_limit')} mA，PWM {g('pwm_limit')}，速度 {g('velocity_limit')}，温度 {g('temp_limit')} °C，"
          f"电压 {(g('min_voltage') or 0)/10:.1f}–{(g('max_voltage') or 0)/10:.1f} V")
    sd = g("shutdown") or 0
    print(f"  保护   : Shutdown 0x{sd:02X} = " + "、".join(n for b, n in SHUTDOWN_BITS.items() if sd >> b & 1))
    hw = g("hw_error") or 0
    errs = [n for b, n in HW_ERRORS.items() if hw >> b & 1]
    v = (g("voltage") or 0) / 10
    print(f"  状态   : 扭矩 {'开' if g('torque') else '关'}，LED {'亮' if g('led') else '灭'}，电压 {v:.1f} V，温度 {g('temperature')} °C，"
          f"位置 {g('present_position')} ticks（{(g('present_position') or 0) * 360 / 4096:.1f}°），电流 {g('present_current')} mA，P 增益 {g('pos_p')}")
    print(f"  硬件错误: " + ("、".join(errs) + "（LED 每秒闪，要断电重启或 reboot 清除）" if errs else "无"))

    # 结论
    verdict = []
    if model == 1200:
        verdict.append("型号对，XL330-M288，可以直接当手套电机用")
    elif model == 1190:
        verdict.append("是 XL330-M077（77.5:1），只能替换 PIP / IP 那四个远端关节（ID 4、7、10、13），不能替换 288:1 的位置")
    elif model in (1220, 1210):
        verdict.append("是 XC330-T 系列 12 V 电机：6 V 总线上会欠压、力矩偏小（和现在的 ID 3 一样），能用但不理想")
    elif model == 1240:
        verdict.append("是 XC330-M288：电压和减速比都和 M288 一样，金属齿轮版，可以直接用")
    elif model in X430:
        verdict.append("是 430 / 540 系列 12 V 电机，接口和电压都不一样，不能用在这条 6 V 总线上")
    else:
        verdict.append("不是 330 系列，控制表不同，这套脚本不支持")
    vmin, vmax = V_RANGE.get(model, (3.7, 6.0))
    if v and v < vmin:
        verdict.append(f"当前电压 {v:.1f} V 低于规格下限 {vmin} V")
    if v and v > vmax:
        verdict.append(f"当前电压 {v:.1f} V 高于规格上限 {vmax} V，不要这样长期跑")
    if dxl_id in reg_ids:
        verdict.append(f"ID {dxl_id} 和手套上已有的电机冲突，接到手套总线前要先改 ID（dxl_setup.py set / reassign）")
    if baud != 1000000:
        verdict.append("波特率不是 1 M，接进手套前要改（dxl_setup.py set 会一起改）")
    if g("return_delay") not in (0, None):
        verdict.append(f"返回延迟 {g('return_delay')} 不是 0，会拖慢总线（dxl_setup.py set 会改成 0）")
    if g("homing_offset"):
        verdict.append(f"Homing Offset {g('homing_offset')} 不是 0：上一个主人改过零点，标定前建议清零")
    if dm & 1:
        verdict.append("驱动模式里方向是反转的：手套上其他电机都是正常方向，建议改回 0，否则标定的符号会混乱")
    if (g("min_position"), g("max_position")) not in ((0, 4095), (None, None)):
        verdict.append(f"位置范围被改成 {g('min_position')}…{g('max_position')}：位置模式下会卡在边界，建议恢复 0…4095")
    if g("secondary_id") not in (255, None):
        verdict.append(f"Secondary ID {g('secondary_id')} 不是 255：它会同时响应这个 ID，改成 255")
    if g("protocol") not in (2, None):
        verdict.append(f"Protocol Type {g('protocol')} 不是 2")
    if errs:
        verdict.append("有硬件错误标志，先 reboot 清掉再看是否复发（反复过载 = 齿轮或电机坏）")
    if g("torque"):
        verdict.append("扭矩是开的（上电默认应为关）：说明有程序在控制它，或 Torque Enable 被写进了 EEPROM 之外的某处")
    print("  结论   : " + "；".join(verdict))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=None)
    ap.add_argument("--expect", type=int, default=None, help="串在一起的电机数量，用来判断有没有漏")
    ap.add_argument("--quick", action="store_true", help="只用广播 ping，不逐个 ID 扫")
    ap.add_argument("--no-p1", action="store_true", help="不扫 Protocol 1.0")
    args = ap.parse_args()
    portname = args.port or default_port()
    print(f"* 串口 {portname}", flush=True)
    w = latency_warning(portname)
    if w:
        print("* " + w)
    try:
        reg_ids = set(load_registry().values())
    except Exception:
        reg_ids = GLOVE_IDS
    pr = Probe(portname)
    t0 = time.time()
    found = []   # (baud, id, model)
    try:
        for baud in BAUDS:
            ids = pr.scan_baud(baud, args.quick)
            print(f"  {baud:>8} bps (2.0): " + (", ".join(f"ID {i} {MODELS.get(m, m)}" for i, m in sorted(ids.items())) or "无"), flush=True)
            found += [(baud, i, m) for i, m in sorted(ids.items())]
        if not args.no_p1:
            for baud in (57600, 1000000):
                ids = pr.scan_p1(baud)
                if ids:
                    print(f"  {baud:>8} bps (1.0): " + ", ".join(f"ID {i} {MODELS.get(m, m)}" for i, m in sorted(ids.items())))
                    found += [(baud, i, ("p1", m)) for i, m in sorted(ids.items())]
        print(f"* 扫描用时 {time.time() - t0:.0f} s，共 {len(found)} 个")
        for baud, i, m in found:
            if isinstance(m, tuple):
                print(f"\n==== ID {i} @ {baud} bps  Protocol 1.0 {MODELS.get(m[1], m[1])} ====\n  结论   : 老协议电机（AX / MX 系列），不能和 XL330 混在一条 Protocol 2.0 总线上")
                continue
            pr.set_baud(baud)
            bad = pr.stability(i)
            if m in X330 or m in X430:
                d = pr.dump_x(i)
                report(m, baud, i, d, reg_ids, bad)
            else:
                print(f"\n==== ID {i} @ {baud} bps  {MODELS.get(m, m)} ====\n  结论   : 不是 X 系列，控制表不同，这套脚本不支持")
        # 汇总
        print("\n==== 汇总 ====")
        n_m288 = sum(1 for _, _, m in found if m in (1200, 1240))
        print(f"  找到 {len(found)} 个电机，其中可直接用的 M288 级 {n_m288} 个")
        ids_seen = [i for _, i, _ in found]
        dup = sorted({i for i in ids_seen if ids_seen.count(i) > 1})
        if dup:
            print(f"  同一个 ID 在多个波特率下都出现: {dup}（同一个电机不会这样，说明有两个电机同 ID）")
        if args.expect is not None and len(found) < args.expect:
            print(f"  !! 只找到 {len(found)} 个，你说串了 {args.expect} 个：少的那些多半和已找到的同 ID（广播 ping 时互相撞包）、"
                  f"或者波特率不在列表里、或者线没通。办法：每次只接一个电机扫，找到就用 dxl_setup.py set 改成不冲突的 ID")
        conflict = sorted({i for _, i, _ in found if i in reg_ids})
        if conflict:
            print(f"  ID {conflict} 和手套已有电机冲突，接到手套总线前先改 ID（这几个电机单独接在 U2D2 上改最安全）")
    except Exception as e:
        if "timeout" in str(e).lower() or "Timeout" in type(e).__name__:
            print(f"\n!! 往 {portname} 写数据超时：板子的 USB 卡住了（不是电机的问题）。把 OpenRB 的 USB 线拔掉再插上（或按一下 RESET，"
                  f"检查有没有胶带压着按钮），确认电机电源已开，再跑一次；或者把电机接到 U2D2 上用 --port /dev/ttyUSB0", flush=True)
            os._exit(1)          # 卡住的串口 close() 也会永远等，直接退出
        raise
    finally:
        pr.close()


if __name__ == "__main__":
    main()
