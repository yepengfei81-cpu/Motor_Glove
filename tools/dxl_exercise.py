#!/usr/bin/env python3
"""
动态测试（会开扭矩、会转动）：逐个电机跑一遍手套要用的两种模式，判断它是不是好的。

  python tools/dxl_exercise.py --port /dev/ttyUSB0 --baud 57600          # 测这个波特率下找到的全部电机，逐个来
  python tools/dxl_exercise.py --port /dev/ttyUSB0 --baud 57600 --ids 1 2 3
  python tools/dxl_exercise.py ... --manual                               # 每个电机多一步：扭矩关掉，让你用手转 5 s，检查编码器
  python tools/dxl_exercise.py ... --yes                                  # 不等回车直接开始
  分批测（比如 8 个电机一次插 4 个）：
  python tools/dxl_exercise.py --port /dev/ttyUSB0 --baud 57600 --expect 4 --label batch1 --assign-ids 21
  python tools/dxl_exercise.py --port /dev/ttyUSB0 --baud 57600 --expect 4 --label batch2 --assign-ids 21
    --expect 4      插了 4 个却找不到 4 个，或者某个 ID 读起来时好时坏 = 有两个电机同 ID，会停下来让你拆开
    --label         这一批的名字，结果追加到 data/motor_tests.json，最后打印所有批次的汇总
    --assign-ids 21 通过测试的电机当场改成 21、22、23…（接着上一批编号，避开总线上已有的 ID），并闪灯让你贴标签；
                    这样两批 8 个以后可以串在一起不撞 ID，再用 dxl_setup.py set 改成手套的正式 ID
  测试中的那个电机 LED 会常亮，方便你知道在测哪个。

前提：输出轴上不要装任何东西（会转 ±45°），电机供电 5–6 V。

每个电机做的事：
  1. 电流模式（模式 0）：+60 mA 转 0.5 s，再 −60 mA 转 0.5 s。看两个方向都动了多少、测到的电流和指令差多少。
     这就是主手套渲染反馈力的模式。
  2. 模式 5（电流限制的位置控制）：电流上限 300 mA，走到 +45°（或 −45°），再回来。看多久到位、最终差几个 tick、
     到位后保持电流多大。这就是从手套 / 机械手跟随的模式。
  3. 前后温度、硬件错误标志。
  4. --manual：扭矩关掉，用手来回转输出轴 5 s，看编码器读数是否连续（有没有跳变）。
做完把模式改回出厂的位置模式（3）、扭矩关掉。不改 ID 和波特率（那是 dxl_setup.py set 的事）。
"""
import argparse
import os
import signal
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS  # noqa: E402
from dxl_bus import default_port  # noqa: E402

A = dict(model=0, id=7, mode=11, current_limit=38, torque=64, led=65, hw_error=70, goal_current=102, goal_position=116,
         present_current=126, present_position=132, voltage=144, temperature=146)
import json  # noqa: E402
RESULTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data", "motor_tests.json")
MODELS = {1190: "XL330-M077", 1200: "XL330-M288", 1220: "XC330-T288", 1240: "XC330-M288", 1230: "XC330-M181", 1210: "XC330-T181"}
TEST_MA, LIMIT_MA, STEP_TICKS = 60, 300, 512      # 60 mA 探方向；模式 5 电流上限 300 mA；±45°
REACH_TICKS, ERR_TICKS = 15, 20                   # 到位判定 ±15 tick（1.3°），最终误差上限 20 tick（1.8°）：减速箱摩擦下 ±10 tick 的稳态误差是正常的


def s16(v):
    return v - 65536 if v >= 32768 else v


class M:
    def __init__(self, port, pkt, i):
        self.port, self.pkt, self.i = port, pkt, i

    def r(self, name, n):
        fn = {1: self.pkt.read1ByteTxRx, 2: self.pkt.read2ByteTxRx, 4: self.pkt.read4ByteTxRx}[n]
        v, res, err = fn(self.port, self.i, A[name])
        if res != COMM_SUCCESS:
            raise IOError(f"ID {self.i} 读 {name} 失败: {self.pkt.getTxRxResult(res)}")
        return v

    def w(self, name, n, v):
        fn = {1: self.pkt.write1ByteTxRx, 2: self.pkt.write2ByteTxRx, 4: self.pkt.write4ByteTxRx}[n]
        res, err = fn(self.port, self.i, A[name], int(v) & ((1 << (8 * n)) - 1))
        if res != COMM_SUCCESS or err:
            raise IOError(f"ID {self.i} 写 {name}={v} 失败: {self.pkt.getTxRxResult(res)} {self.pkt.getRxPacketError(err) if err else ''}")

    def pos(self):
        return self.r("present_position", 4)

    def cur(self):
        return s16(self.r("present_current", 2))

    def off(self):
        try:
            self.w("goal_current", 2, 0)
        except Exception:
            pass
        self.w("torque", 1, 0)


def stability(m, n=10):
    """连续读 n 次型号，失败次数；> 0 基本就是两个电机同 ID 在抢总线"""
    bad = 0
    for _ in range(n):
        v, res, _ = m.pkt.read2ByteTxRx(m.port, m.i, A["model"])
        bad += res != COMM_SUCCESS
        time.sleep(0.01)
    return bad


def test_motor(m, manual):
    out = {"id": m.i, "model": MODELS.get(m.r("model", 2), "?")}
    m.off()
    m.w("led", 1, 1)
    out["temp0"] = m.r("temperature", 1); out["volt"] = m.r("voltage", 2) / 10
    if m.r("hw_error", 1):
        out["verdict"] = "上电就有硬件错误标志，先 reboot 再测"; return out

    # 1) 电流模式：两个方向各 0.5 s
    m.w("mode", 1, 0); m.w("current_limit", 2, LIMIT_MA)
    m.w("torque", 1, 1); m.w("goal_current", 2, 0)
    for sign, key in ((1, "cw"), (-1, "ccw")):
        p0 = m.pos(); m.w("goal_current", 2, sign * TEST_MA)
        samples = []
        t0 = time.time()
        while time.time() - t0 < 0.5:
            samples.append(m.cur()); time.sleep(0.03)
        p1 = m.pos(); m.w("goal_current", 2, 0); time.sleep(0.3)
        out[f"move_{key}"] = p1 - p0
        out[f"imeas_{key}"] = statistics.median(samples[len(samples) // 2:]) if samples else 0
    m.off()

    # 2) 模式 5：±45° 一来一回
    m.w("mode", 1, 5); m.w("torque", 1, 1); m.w("goal_current", 2, LIMIT_MA)
    start = m.pos() % 4096
    target = start + STEP_TICKS if start + STEP_TICKS <= 4000 else start - STEP_TICKS
    legs = []
    for goal in (target, start):
        m.w("goal_position", 4, goal)
        t0 = time.time(); t_reach = None; peak = 0
        while time.time() - t0 < 2.0:
            p = m.pos(); c = abs(m.cur()); peak = max(peak, c)
            if t_reach is None and abs(p - goal) <= REACH_TICKS:
                t_reach = time.time() - t0
            if t_reach is not None and time.time() - t0 > t_reach + 0.4:
                break
            time.sleep(0.02)
        time.sleep(0.2)
        legs.append(dict(t=t_reach, err=m.pos() - goal, hold=abs(m.cur()), peak=peak))
    m.off()
    out["legs"] = legs

    # 3) 收尾：改回位置模式、温度、错误
    m.w("mode", 1, 3); m.w("led", 1, 0)
    out["temp1"] = m.r("temperature", 1); out["hw"] = m.r("hw_error", 1)

    # 4) 手转编码器检查
    if manual:
        input(f"  ID {m.i}: 扭矩已关，请用手来回转输出轴，按回车开始记录 5 s ...")
        ps, t0 = [], time.time()
        while time.time() - t0 < 5.0:
            ps.append(m.pos() % 4096); time.sleep(0.02)
        jumps = sum(1 for a, b in zip(ps, ps[1:]) if 300 < abs(a - b) < 3796)
        out["manual"] = dict(range=max(ps) - min(ps), jumps=jumps)

    # 结论
    bad = []
    if abs(out["move_cw"]) < 30 or abs(out["move_ccw"]) < 30:
        bad.append(f"电流模式 {TEST_MA} mA 下转得太少（{out['move_cw']:+d} / {out['move_ccw']:+d} ticks）：卡滞或齿轮坏")
    if out["move_cw"] * out["move_ccw"] > 0:
        bad.append("两个方向的电流让它往同一边转：异常")
    for key in ("cw", "ccw"):
        if abs(abs(out[f"imeas_{key}"]) - TEST_MA) > 30:
            bad.append(f"电流读数 {out[f'imeas_{key}']:+.0f} mA 和指令 ±{TEST_MA} mA 差太多：电流检测不准")
    for k, leg in enumerate(legs):
        if leg["t"] is None and abs(leg["err"]) > ERR_TICKS:
            bad.append(f"模式 5 第 {k + 1} 段 2 s 内没到位（差 {leg['err']:+d} ticks）")
        elif leg["t"] is not None and leg["t"] > 1.0:
            bad.append(f"模式 5 第 {k + 1} 段到位用了 {leg['t']:.2f} s，偏慢")
        elif abs(leg["err"]) > ERR_TICKS:
            bad.append(f"模式 5 第 {k + 1} 段最终误差 {leg['err']:+d} ticks（> {ERR_TICKS * 360 / 4096:.1f}°）")
        if leg["hold"] > 120:
            bad.append(f"模式 5 第 {k + 1} 段到位后还在出 {leg['hold']} mA：卡滞")
    if out["hw"]:
        bad.append(f"测试后出现硬件错误 0x{out['hw']:02X}")
    if out["temp1"] - out["temp0"] > 8:
        bad.append(f"温升 {out['temp1'] - out['temp0']} °C 偏大")
    if manual and out["manual"]["jumps"]:
        bad.append(f"手转时编码器跳变 {out['manual']['jumps']} 次")
    out["verdict"] = "通过" if not bad else "；".join(bad)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--ids", type=int, nargs="*", default=None)
    ap.add_argument("--manual", action="store_true")
    ap.add_argument("--yes", action="store_true")
    ap.add_argument("--expect", type=int, default=None, help="这次插了几个电机；找不到这么多就停下来查重复 ID")
    ap.add_argument("--label", default=None, help="这一批的名字，结果追加到 data/motor_tests.json")
    ap.add_argument("--assign-ids", type=int, default=None, metavar="START", help="通过的电机当场改成从 START 起的连续新 ID（接着上一批），并闪灯")
    ap.add_argument("--force", action="store_true", help="就算怀疑有重复 ID 也继续测")
    args = ap.parse_args()
    label = args.label or time.strftime("%Y%m%d-%H%M")
    portname = args.port or default_port()
    port = PortHandler(portname)
    if not port.openPort():
        sys.exit(f"打不开 {portname}")
    port.setBaudRate(args.baud); port.ser.write_timeout = 1.0
    pkt = PacketHandler(2.0)
    ids = args.ids or sorted(pkt.broadcastPing(port)[0].keys())
    if not ids:
        sys.exit(f"{args.baud} bps 下没找到电机")
    # 广播 ping 可能漏掉同 ID 的电机：再逐个 ping 0–252 补一遍
    for i in range(253):
        if i not in ids and pkt.ping(port, i)[1] == COMM_SUCCESS:
            ids.append(i)
    ids = sorted(ids)
    print(f"* {portname} @ {args.baud} bps，找到的电机: {ids}")
    unstable = [i for i in ids if stability(M(port, pkt, i)) > 0]
    if unstable:
        print(f"!! 这些 ID 读起来时好时坏: {unstable} —— 几乎可以肯定有两个电机用同一个 ID，它们会一起响应、互相撞包")
    if args.expect is not None and len(ids) < args.expect:
        print(f"!! 你说插了 {args.expect} 个，只找到 {len(ids)} 个：少的那 {args.expect - len(ids)} 个和已找到的某个同 ID（或者线没通）")
    if (unstable or (args.expect is not None and len(ids) < args.expect)) and not args.force:
        try:
            port.ser.close()
        except Exception:
            pass
        sys.exit("* 停下：把这批电机拆成更小的组（每组里 ID 不重复）再测；或者加 --force 硬测（同 ID 的那两个结果不可信）")
    print("* 会开扭矩并转动 ±45°：确认输出轴上没装东西、线没缠住", flush=True)
    if not args.yes:
        input("  按回车开始 ...")
    motors = [M(port, pkt, i) for i in ids]
    results = []
    history = []
    if os.path.exists(RESULTS):
        try:
            history = json.load(open(RESULTS, encoding="utf-8"))
        except Exception:
            history = []
    used_ids = set(ids) | {h.get("new_id") for h in history if h.get("new_id")}
    next_id = args.assign_ids

    def stop_all(*_):
        for m in motors:
            try:
                m.off(); m.w("mode", 1, 3)
            except Exception:
                pass
    signal.signal(signal.SIGINT, lambda *_: (stop_all(), sys.exit("\n* 已中断，扭矩全关")))
    try:
        for m in motors:
            print(f"\n== ID {m.i} ==", flush=True)
            try:
                r = test_motor(m, args.manual)
            except IOError as e:
                r = {"id": m.i, "model": "?", "verdict": f"通信出错: {e}"}
                m.off()
            results.append(r)
            if "legs" in r:
                print(f"  电流模式: +{TEST_MA} mA 转 {r['move_cw']:+d} ticks（读 {r['imeas_cw']:+.0f} mA），"
                      f"−{TEST_MA} mA 转 {r['move_ccw']:+d} ticks（读 {r['imeas_ccw']:+.0f} mA）")
                for k, leg in enumerate(r["legs"]):
                    print(f"  模式 5 第 {k + 1} 段: 到位 {('%.2f s' % leg['t']) if leg['t'] else '未到'}，误差 {leg['err']:+d} ticks，"
                          f"峰值 {leg['peak']} mA，保持 {leg['hold']} mA")
                print(f"  温度 {r['temp0']} → {r['temp1']} °C，电压 {r['volt']:.1f} V" + (f"，手转范围 {r['manual']['range']} ticks，跳变 {r['manual']['jumps']} 次" if "manual" in r else ""))
            print(f"  结论: {r['verdict']}", flush=True)
            r["label"] = label; r["baud"] = args.baud; r["time"] = time.strftime("%Y-%m-%d %H:%M")
            if next_id is not None and r["verdict"] == "通过" and m.i >= args.assign_ids and m.i not in range(1, 14) and sum(1 for x in ids if x == m.i) == 1:
                r["new_id"] = m.i                                            # 已经是独一无二、不在 1–13 的 ID，不用改
                print(f"  ID {m.i} 保持不变（已经合适）")
            elif next_id is not None and r["verdict"] == "通过":
                while next_id in used_ids or next_id in range(1, 14):       # 避开总线上已有的和手套正式用的 1–13
                    next_id += 1
                try:
                    m.w("torque", 1, 0); m.w("id", 1, next_id); time.sleep(0.1)
                    m2 = M(port, pkt, next_id); m2.r("model", 2)
                    for _ in range(3):
                        m2.w("led", 1, 1); time.sleep(0.25); m2.w("led", 1, 0); time.sleep(0.25)
                    print(f"  ID {m.i} → {next_id}（已闪灯 3 次，给它贴上 {next_id} 的标签）", flush=True)
                    r["new_id"] = next_id; used_ids.add(next_id); next_id += 1
                except IOError as e:
                    print(f"  !! 改 ID 失败: {e}")
    finally:
        stop_all()
        try:
            port.ser.close()
        except Exception:
            pass
    print(f"\n==== 这一批（{label}）====")
    for r in results:
        print(f"  ID {r['id']:2d} {r['model']:12s} {r['verdict']}" + (f"  → 新 ID {r['new_id']}" if r.get("new_id") else ""))
    ok = [r["id"] for r in results if r["verdict"] == "通过"]
    print(f"  通过 {len(ok)}/{len(results)}: {ok}")
    history = [h for h in history if h.get("label") != label] + results
    os.makedirs(os.path.dirname(RESULTS), exist_ok=True)
    json.dump(history, open(RESULTS, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    if len({h.get("label") for h in history}) > 1:
        print("\n==== 所有批次 ====")
        for h in history:
            print(f"  {h.get('label', ''):12s} ID {h['id']:2d}" + (f" → {h['new_id']:3d}" if h.get("new_id") else "      ") + f"  {h.get('model', '?'):12s} {h['verdict']}")
        passed = [h for h in history if h["verdict"] == "通过"]
        print(f"  累计通过 {len(passed)}/{len(history)}")
    print(f"* 结果已存到 {os.path.relpath(RESULTS)}")
    if ok:
        print("  下一步：用 dxl_setup.py set <关节名> --port ... 一个一个改成手套需要的 ID 和 1 M 波特率（返回延迟也会改成 0）")


if __name__ == "__main__":
    main()
