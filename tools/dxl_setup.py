#!/usr/bin/env python3
"""
电机 ID / 波特率设置（通过 OpenRB-150 的 usb_to_dynamixel 桥接）。

  python tools/dxl_setup.py scan                      # 在所有常用波特率上扫描总线
  python tools/dxl_setup.py set ring_mcp_abd_motor    # 把"新接上的那个电机"设成这个关节的 ID（按约定表），1 Mbps
  python tools/dxl_setup.py set ring_pip_motor --new-id 20   # 不按约定表，手动指定 ID
  python tools/dxl_setup.py show                      # 1 Mbps 下列出所有电机：ID / 关节 / 电压 / 温度 / 模式
  python tools/dxl_setup.py blink 12                  # 让 ID 12 的电机 LED 闪几下，确认是哪一个
  python tools/dxl_setup.py status                    # 13 个关节的进度总览：ID 设了没、在不在总线上、标定了没
  python tools/dxl_setup.py reassign 2 thumb_cmc_abd_motor   # 设错了：把现在 ID 2 的电机改成这个关节（按约定改 ID）
  python tools/dxl_setup.py reassign 12 thumb_cmc_flex_motor --duplicate
                                                      # 两个电机撞了同一个 ID：总线上只接设错的那个，把它改过来，
                                                      # 原来 ID 12 的登记和标定保持不动
  python tools/dxl_setup.py forget 2                  # 只删除登记（电机里的 ID 不变）
  python tools/dxl_setup.py reboot                    # 重启有硬件错误（过载、过热等保护）的电机；reboot 2 只重启 ID 2
  python tools/dxl_setup.py verify                    # 全面检查：13 个 ID 都在、没有撞 ID、没有遗漏的出厂电机
  python tools/dxl_setup.py verify --move             # 再逐个关节动一下，核对每个电机装的位置和登记的关节一致

"新接上的电机"= 总线上唯一一个还没登记（不在 config/motors.json 里、或不在 1 Mbps）的电机。
第二只手套（从手）用同一张 ID 表，接在另一条总线上：python tools/dxl_setup.py --port /dev/ttyUSB0 set ring_mcp_flex_motor
所以可以一个一个往链上加电机，不用每次把前面的拔掉：断电 → 接上下一个 → 上电 → set。
EEPROM 项（ID、波特率、Return Delay）只在扭矩关闭时能写，电机刚上电默认就是关闭的。
"""
import argparse
import json
import os
import sys
import time

from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncRead, COMM_SUCCESS


def default_port():
    """DXL_PORT 环境变量优先；否则有 U2D2(/dev/ttyUSB*) 就用它，再退回 OpenRB(/dev/ttyACM0)"""
    import glob
    if os.environ.get("DXL_PORT"):
        return os.environ["DXL_PORT"]
    usb = sorted(glob.glob("/dev/ttyUSB*"))
    return usb[0] if usb else "/dev/ttyACM0"


def latency_warning(port):
    """U2D2 是 FTDI 芯片，latency timer 默认 16 ms，读写会很慢"""
    name = os.path.basename(os.path.realpath(port))
    path = f"/sys/bus/usb-serial/devices/{name}/latency_timer"
    if os.path.exists(path):
        v = open(path).read().strip()
        if v != "1":
            return f"提示：{port} 的 USB latency timer 是 {v} ms（建议 1 ms，见 tools/99-dxl-latency.rules）"
    return None


PORT = default_port()

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from glove_model import DEFAULT_IDS  # noqa: E402

BAUDS = [57600, 1000000, 115200, 9600, 2000000, 3000000, 4000000]
BAUD_CODE = {9600: 0, 57600: 1, 115200: 2, 1000000: 3, 2000000: 4, 3000000: 5, 4000000: 6}
TARGET_BAUD = 1000000
ADDR_ID, ADDR_BAUD, ADDR_RETURN_DELAY, ADDR_MODE, ADDR_TORQUE, ADDR_LED = 7, 8, 9, 11, 64, 65
ADDR_VOLTAGE, ADDR_TEMP, ADDR_HW_ERROR = 144, 146, 70
HW_ERRORS = {0: "输入电压超限", 2: "过热", 3: "编码器", 4: "电气冲击", 5: "过载"}
V_SPEC_MAX = 6.0   # XL330 规格上限 6.0 V；电机内部报警阈值默认 7.0 V (Max Voltage Limit)


def health(port, dxl_id):
    """读电压和硬件错误，返回一行说明（电压按这个电机型号的规格范围判断）"""
    v, r1, _ = PKT.read2ByteTxRx(port, dxl_id, ADDR_VOLTAGE)
    e, r2, _ = PKT.read1ByteTxRx(port, dxl_id, ADDR_HW_ERROR)
    mdl, r3, _ = PKT.read2ByteTxRx(port, dxl_id, 0)
    if r1 != COMM_SUCCESS or r2 != COMM_SUCCESS:
        return "读状态失败"
    vmin, vmax = V_RANGE.get(mdl, (3.7, V_SPEC_MAX)) if r3 == COMM_SUCCESS else (3.7, V_SPEC_MAX)
    out = f"{v/10:.1f} V"
    if v / 10 > vmax:
        out += f"（超过规格上限 {vmax} V）"
    elif v / 10 < vmin:
        out += f"（低于这个型号的规格下限 {vmin} V，力矩会偏小）"
    errs = [name for bit, name in HW_ERRORS.items() if e >> bit & 1]
    out += "，硬件错误: " + "、".join(errs) + "（LED 每秒闪一次，需要断电重启）" if errs else "，无硬件错误"
    return out
REG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", "motors.json")
PKT = PacketHandler(2.0)
# 型号代码 -> 名称、规格电压范围（来自 ROBOTIS e-Manual；T 系列是 12 V 电机）
MODELS = {1190: "XL330-M077", 1200: "XL330-M288", 1210: "XC330-T181", 1220: "XC330-T288", 1230: "XC330-M181", 1240: "XC330-M288"}
V_RANGE = {1190: (3.7, 6.0), 1200: (3.7, 6.0), 1210: (6.5, 12.0), 1220: (6.5, 12.0), 1230: (3.7, 6.0), 1240: (3.7, 6.0)}


def load_reg():
    return json.load(open(REG_FILE)) if os.path.exists(REG_FILE) else {}


def save_reg(reg):
    os.makedirs(os.path.dirname(REG_FILE), exist_ok=True)
    json.dump(dict(sorted(reg.items(), key=lambda kv: int(kv[0]))), open(REG_FILE, "w"), indent=1, ensure_ascii=False)


def open_port(baud):
    port = PortHandler(PORT)
    try:
        ok = port.openPort()
    except Exception:
        ok = False
    if not ok:
        sys.exit(f"打不开 {PORT}（U2D2 / OpenRB 接上电脑了吗？）")
    port.setBaudRate(baud)
    port.setPacketTimeout(200)
    return port


def scan(verbose=True):
    """返回 [(baud, id, model)]"""
    if verbose:
        print(f"* 端口 {PORT}")
        w = latency_warning(PORT)
        if w:
            print("* " + w)
    found = []
    for baud in BAUDS:
        port = open_port(baud)
        res = PKT.broadcastPing(port)[0]
        if verbose:
            print(f"  {baud:>8} bps: " + ("无" if not res else ""))
            for i, v in sorted(res.items()):
                print(f"      ID {i} ({MODELS.get(v[0], 'model ' + str(v[0]))})  {health(port, i)}")
        port.closePort()
        for i, v in sorted(res.items()):
            found.append((baud, i, v[0]))
    return found


def check(res, err, what):
    if res != COMM_SUCCESS:
        sys.exit(f"{what} 失败: {PKT.getTxRxResult(res)}")
    if err:
        sys.exit(f"{what} 电机报错: {PKT.getRxPacketError(err)}")


def blink(port, dxl_id, times=3):
    for _ in range(times):
        PKT.write1ByteTxRx(port, dxl_id, ADDR_LED, 1); time.sleep(0.25)
        PKT.write1ByteTxRx(port, dxl_id, ADDR_LED, 0); time.sleep(0.25)


def cmd_set(args):
    joint = args.joint
    if joint not in DEFAULT_IDS:
        sys.exit(f"不认识的关节名 {joint}。可选: {', '.join(DEFAULT_IDS)}")
    new_id = args.new_id or DEFAULT_IDS[joint]
    reg = load_reg()
    print("* 扫描总线 ...")
    found = scan()
    known = [(b, i) for b, i, _ in found if b == TARGET_BAUD and str(i) in reg]
    new = [(b, i) for b, i, _ in found if (b, i) not in known]
    if args.from_id is not None:
        # 总线上同时接着好几个已经编了临时 ID（dxl_exercise.py --assign-ids）的电机：指定要改哪一个
        new = [(b, i) for b, i in new if i == args.from_id]
        if len(new) == 0:
            sys.exit(f"!! 总线上没有 ID {args.from_id} 的未登记电机。找到的: {[(b, i) for b, i, _ in found]}")
        if len(new) > 1:
            sys.exit(f"!! ID {args.from_id} 在多个波特率下都出现 {new}，有两个电机同 ID，拆开再来")
    if len(new) == 0:
        sys.exit("!! 没找到新电机（出厂电机是 ID 1 @ 57600）。确认新电机的线接好了、电池接着。\n"
                 "   现在总线上已登记的: " + ", ".join(f"ID {i}={reg[str(i)]}" for _, i in known) +
                 "\n   如果你要设的电机其实已经设过（ID 设错了），用  python tools/dxl_setup.py reassign <它现在的ID> <正确关节名>")
    if len(new) > 1:
        sys.exit(f"!! 总线上有 {len(new)} 个未登记的电机 {new}，一次只能接一个新的（出厂 ID 都是 1，会冲突）；"
                 f"如果它们已经有各不相同的临时 ID，用 --from <临时ID> 指定要改哪一个")
    if any(i == new_id for _, i in known):
        sys.exit(f"!! ID {new_id} 已经被 {reg[str(new_id)]} 用了")
    baud, old_id = new[0]
    port = open_port(baud)
    print(f"* 新电机: ID {old_id} @ {baud} bps  ->  ID {new_id} @ {TARGET_BAUD} bps（{joint}）")
    check(*PKT.write1ByteTxRx(port, old_id, ADDR_TORQUE, 0), "关扭矩")
    if old_id != new_id:
        check(*PKT.write1ByteTxRx(port, old_id, ADDR_ID, new_id), "写 ID")
    check(*PKT.write1ByteTxRx(port, new_id, ADDR_RETURN_DELAY, 0), "写 Return Delay")
    if baud != TARGET_BAUD:
        check(*PKT.write1ByteTxRx(port, new_id, ADDR_BAUD, BAUD_CODE[TARGET_BAUD]), "写波特率")
    port.closePort()
    time.sleep(0.3)
    port = open_port(TARGET_BAUD)
    model, res, err = PKT.ping(port, new_id)
    check(res, err, "改完后 ping")
    print(f"* 完成：ID {new_id} 应答正常，{health(port, new_id)}。LED 闪 3 下，确认是你刚接的那个电机")
    blink(port, new_id)
    port.closePort()
    reg[str(new_id)] = joint
    save_reg(reg)
    print(f"* 已登记 ID {new_id} -> {joint}（{os.path.relpath(REG_FILE)}）")


def cmd_show(args):
    port = open_port(TARGET_BAUD)
    found, _ = PKT.broadcastPing(port)
    reg = load_reg()
    print(f"{TARGET_BAUD} bps 下找到 {len(found)} 个电机")
    for i in sorted(found):
        t, _, _ = PKT.read1ByteTxRx(port, i, ADDR_TEMP)
        m, _, _ = PKT.read1ByteTxRx(port, i, ADDR_MODE)
        name = reg.get(str(i), "(未登记)")
        warn = "" if name == "(未登记)" or DEFAULT_IDS.get(name) == i else "  <- 和约定 ID 不一致"
        model = MODELS.get(found[i][0], f"model {found[i][0]}")
        print(f"  ID {i:2d}  {name:22s} {model:11s} {t} °C  模式 {m}  {health(port, i)}{warn}")
    missing = [f"{j}(ID {i})" for j, i in DEFAULT_IDS.items() if str(i) in reg and i not in found]
    if missing:
        print("  已登记但没应答:", ", ".join(missing))
    port.closePort()


def cmd_status(args):
    reg = {name: int(i) for i, name in load_reg().items()}
    cal_file = os.path.join(os.path.dirname(REG_FILE), "calibration.json")
    cal = json.load(open(cal_file))["joints"] if os.path.exists(cal_file) else {}
    on_bus = {}
    try:
        port = open_port(TARGET_BAUD)
        on_bus = {i: v[0] for i, v in PKT.broadcastPing(port)[0].items()}
        port.closePort()
    except SystemExit:
        print("（板子没接上，只显示登记和标定情况）")
    from glove_model import FINGERS
    print(f"{'手指':6s} {'关节':22s} {'约定ID':>6s}  {'已设ID':6s} {'在总线上':8s} {'型号':11s} 已标定")
    for f, joints in FINGERS.items():
        for j in joints:
            i = reg.get(j)
            print(f"{f:6s} {j:22s} {DEFAULT_IDS[j]:6d}  {('ID %d' % i) if i else '-':6s} "
                  f"{('是' if i in on_bus else '否') if i else '-':8s} {MODELS.get(on_bus.get(i), '') if i in on_bus else '':11s} "
                  f"{'是' if j in cal else '-'}")
    done = sum(j in cal for js in FINGERS.values() for j in js)
    nxt = (next((f"python tools/dxl_setup.py set {j}" for js in FINGERS.values() for j in js if j not in reg), "") or
           next((f"python tools/calibrate.py {f}" for f, js in FINGERS.items() if any(j not in cal for j in js)), "") or
           "全部完成，可以运行 python tools/teleop_view.py 或 python tools/force_feedback.py")
    print(f"\n已标定 {done}/13；下一步：{nxt}")


GLOVE = "A"


def drop_calibration(*joints):
    cal_file = os.path.join(os.path.dirname(REG_FILE), "calibration.json" if GLOVE == "A" else f"calibration_{GLOVE}.json")
    if not os.path.exists(cal_file):
        return []
    cal = json.load(open(cal_file))
    dropped = [j for j in joints if cal["joints"].pop(j, None) is not None]
    json.dump(cal, open(cal_file, "w"), indent=1, ensure_ascii=False)
    return dropped


def cmd_reassign(args):
    joint = args.joint
    if joint not in DEFAULT_IDS:
        sys.exit(f"不认识的关节名 {joint}")
    reg = load_reg()
    new_id = args.new_id or DEFAULT_IDS[joint]
    port = open_port(TARGET_BAUD)
    on_bus = PKT.broadcastPing(port)[0]
    if args.id not in on_bus:
        sys.exit(f"!! 1 Mbps 下没有 ID {args.id} 的电机（接上了吗？电池接了吗？）")
    if args.duplicate and len(on_bus) != 1:
        sys.exit(f"!! --duplicate 时总线上只能接设错的那一个电机，现在找到 {sorted(on_bus)}。"
                 f"先拔电池，把其他电机（特别是另一个 ID {args.id}）的线拔掉")
    if new_id != args.id and new_id in on_bus:
        sys.exit(f"!! 总线上已经有一个 ID {new_id}（登记为 {reg.get(str(new_id), '未登记')}），先把它拔掉或也改掉")
    old_name = "(撞 ID 的电机)" if args.duplicate else reg.get(str(args.id), "(未登记)")
    print(f"* ID {args.id}（原登记 {old_name}） -> ID {new_id}（{joint}）")
    check(*PKT.write1ByteTxRx(port, args.id, ADDR_TORQUE, 0), "关扭矩")
    if new_id != args.id:
        check(*PKT.write1ByteTxRx(port, args.id, ADDR_ID, new_id), "写 ID")
    time.sleep(0.1)
    _, res, err = PKT.ping(port, new_id)
    check(res, err, "改完后 ping")
    print("* 改好了，LED 闪 3 下，确认是这个电机")
    blink(port, new_id)
    port.closePort()
    if not args.duplicate:                                  # 重复 ID 的情况下，原来那个 ID 属于另一个电机，登记保留
        reg.pop(str(args.id), None)
    for k in [k for k, v in reg.items() if v == joint and k != str(args.id)]:   # 这个关节以前登记的别的 ID 作废
        reg.pop(k)
    reg[str(new_id)] = joint
    save_reg(reg)
    dropped = drop_calibration(joint) if args.duplicate else drop_calibration(joint, old_name)
    print(f"* 登记已更新" + (f"；{dropped} 的标定已作废，需要重新标定" if dropped else ""))


def cmd_forget(args):
    reg = load_reg()
    name = reg.pop(str(args.id), None)
    if name is None:
        sys.exit(f"ID {args.id} 没有登记")
    save_reg(reg)
    dropped = drop_calibration(name)
    print(f"* 已删除登记 ID {args.id} -> {name}" + (f"（标定也删了）" if dropped else "") + "。电机里的 ID 没变")


def cmd_verify(args):
    from glove_model import FINGERS, physical_positive
    reg = load_reg()
    expect = {j: DEFAULT_IDS[j] for js in FINGERS.values() for j in js}
    problems = []
    print("[1] 扫描所有波特率，找没设好的电机 ...")
    found = scan(verbose=False)
    stray = [(b, i) for b, i, _ in found if b != TARGET_BAUD]
    for b, i in stray:
        problems.append(f"{b} bps 上还有一个 ID {i} 的电机（{'出厂设置，还没设 ID' if (b, i) == (57600, 1) else '没改成 1 Mbps'}）")
    on_bus = {i: mdl for b, i, mdl in found if b == TARGET_BAUD}

    print(f"[2] 1 Mbps 下逐个检查 13 个 ID（每个 ping 30 次，撞 ID 会出通信错误）...")
    port = open_port(TARGET_BAUD)
    present = {}                                   # 有应答的关节：只核对这些（手套没装满时其余的跳过）
    for j, i in expect.items():
        name = reg.get(str(i))
        if name != j:
            problems.append(f"登记表: ID {i} 应该是 {j}，现在登记为 {name or '无'}")
        bad = 0
        for _ in range(30):
            _, res, err = PKT.ping(port, i)
            bad += res != COMM_SUCCESS
        model = MODELS.get(on_bus.get(i), "-")
        state = "无应答" if bad == 30 else (f"{bad}/30 次通信错误，可能有两个电机都是 ID {i}" if bad else "正常")
        print(f"    ID {i:2d} {j:22s} {model:11s} {state}" + (f"  {health(port, i)}" if bad < 30 else ""))
        if bad:
            problems.append(f"ID {i}（{j}）{state}")
        if bad < 30:
            present[j] = i
    extra = sorted(set(on_bus) - set(expect.values()))
    for i in extra:
        problems.append(f"总线上多了一个 ID {i}（约定表里没有这个 ID），用 blink {i} 找到它再 reassign")

    if args.move and present:
        skipped = [j for j in expect if j not in present]
        print(f"\n[3] 逐个关节核对位置（{len(present)} 个有应答的关节" + (f"，跳过没装的 {len(skipped)} 个" if skipped else "") + "）：扭矩已关，按提示只动一个关节，动着或停在新位置时按回车")
        ids = list(present.values())
        for i in ids:
            PKT.write1ByteTxRx(port, i, ADDR_TORQUE, 0)
        reader = GroupSyncRead(port, PKT, 132, 4)
        for i in ids:
            reader.addParam(i)

        def read_all():
            if reader.txRxPacket() != COMM_SUCCESS:
                return None
            return {i: reader.getData(i, 132, 4) for i in ids}
        for j, i in present.items():
            base = read_all()
            input(f"    只动 {j}（{physical_positive(j)}）约 10°，按回车 ...")
            now = read_all()
            if base is None or now is None:
                problems.append(f"{j}: 读取失败")
                continue
            moved = {k: abs(((now[k] - base[k] + 2048) % 4096) - 2048) for k in ids}
            top = max(moved, key=moved.get)
            if moved[top] < 35:                      # 10° ≈ 114 tick，门槛 35 tick ≈ 3°
                print(f"      没检测到明显转动（最大 {moved[top] * 0.088:.1f}°），跳过")
                problems.append(f"{j}: 没检测到转动，没核对上")
            elif top == i:
                print(f"      ✓ ID {i} 在动（{moved[i] * 0.088:.1f}°）")
            else:
                print(f"      ✗ 动的是 ID {top}（{reg.get(str(top), '未登记')}），不是 ID {i}")
                problems.append(f"{j} 实际是 ID {top} 的电机在动，登记的是 ID {i}")
    port.closePort()
    print("\n" + ("========== 全部正确 ==========" if not problems else "========== 发现问题 ==========\n  - " + "\n  - ".join(problems)))


def cmd_reboot(args):
    port = open_port(TARGET_BAUD)
    targets = [args.id] if args.id else [i for i in range(1, 254) if str(i) in load_reg()]
    done = []
    for i in targets:
        e, res, _ = PKT.read1ByteTxRx(port, i, ADDR_HW_ERROR)
        if res != COMM_SUCCESS:
            continue
        if e or args.id:
            PKT.reboot(port, i)
            done.append((i, [n for b, n in HW_ERRORS.items() if e >> b & 1]))
    time.sleep(0.6)
    for i, errs in done:
        print(f"  ID {i}: 重启前 {('、'.join(errs) or '无错误')}，重启后 {health(port, i)}")
    if not done:
        print("没有电机有硬件错误")
    port.closePort()


def cmd_blink(args):
    port = open_port(TARGET_BAUD)
    _, res, _ = PKT.ping(port, args.id)
    if res != COMM_SUCCESS:
        sys.exit(f"ID {args.id} 没应答")
    blink(port, args.id, 5)
    port.closePort()


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=None, help="串口（默认自动：有 U2D2 用 ttyUSB0，否则 ttyACM0）；也可用环境变量 DXL_PORT")
    ap.add_argument("--glove", default="A", help="在哪只手套的总线上操作（A / B）：决定 reassign 作废的是哪个标定文件")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("scan")
    s = sub.add_parser("set"); s.add_argument("joint"); s.add_argument("--new-id", type=int, default=None)
    s.add_argument("--from", dest="from_id", type=int, default=None, help="总线上有多个新电机时，指定要改的那个的当前（临时）ID")
    sub.add_parser("show")
    sub.add_parser("status")
    r = sub.add_parser("reassign"); r.add_argument("id", type=int); r.add_argument("joint"); r.add_argument("--new-id", type=int, default=None)
    r.add_argument("--duplicate", action="store_true", help="这个电机和另一个已登记的电机撞了 ID：不动原来的登记和标定")
    fg = sub.add_parser("forget"); fg.add_argument("id", type=int)
    v = sub.add_parser("verify"); v.add_argument("--move", action="store_true", help="逐个关节动一下核对位置")
    rb = sub.add_parser("reboot"); rb.add_argument("id", type=int, nargs="?", default=None)
    b = sub.add_parser("blink"); b.add_argument("id", type=int)
    a = ap.parse_args()
    if a.port:
        globals()["PORT"] = a.port
    globals()["GLOVE"] = a.glove.upper()
    {"scan": lambda a: scan(), "set": cmd_set, "show": cmd_show, "status": cmd_status, "reassign": cmd_reassign, "forget": cmd_forget, "verify": cmd_verify, "reboot": cmd_reboot, "blink": cmd_blink}[a.cmd](a)
