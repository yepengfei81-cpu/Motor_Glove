#!/usr/bin/env python3
"""
零点检查：一台设备（手套 A / B 或机械手 H）用位置模式回到标定零点并保持，看大电流下零点会不会漂。

  python tools/zero_check.py A                       # 回零并保持（电流上限 1750 mA，P 800），实时显示每个关节的偏差和电流
  python tools/zero_check.py A thumb index           # 只看几根手指
  python tools/zero_check.py A --limit 600 --kp 400  # 换电流上限 / 增益
  python tools/zero_check.py A --log data/zero_A.csv # 记录每个周期的偏差和电流

  python tools/zero_check.py A --stops save          # 扭矩关：每个关节推到机械限位，记下编码器读数（存 config/stops_A.json）
  python tools/zero_check.py A --stops compare       # 再推一遍，和存的比

两件事要分开看：
  · 保持模式里的"偏差"是编码器读数离零点多远。你用手掰、松开后它回到 < 1° 就说明位置环没问题。
    但编码器装在电机输出轴上，舵盘在花键上打滑、连杆或打印件变形，编码器是看不出来的——它照样显示 0°。
  · 所以真正的"零点漂移"要用机械限位查：大电流测试之前 --stops save，测试之后 --stops compare。
    限位是零件上固定的，同一个限位读出来的角度变了（> 1°），就是电机和关节之间真的错位了。
"""
import argparse
import csv
import json
import math
import os
import signal
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import devices  # noqa: E402
import glove_model as gm  # noqa: E402
from dxl_bus import (Bus, TICKS_PER_REV, effective_sign, load_calibration, load_registry,  # noqa: E402
                     ticks_to_rad, wrap_ticks)


def setup(dev, fingers):
    reg, cal = load_registry(), load_calibration(dev["name"])
    axes = gm.joint_axes(gm.scene_of(dev["model"]))
    joints = [j for f in fingers for j in gm.FINGERS[f] if j in cal["joints"]]
    missing = [j for f in fingers for j in gm.FINGERS[f] if j not in cal["joints"]]
    if missing:
        print(f"* 没标定、跳过: {missing}")
    if not joints:
        sys.exit(f"设备 {dev['name']} 这些手指还没有标定")
    ids = {j: reg[j] for j in joints}
    sign = {j: effective_sign(cal["joints"][j], j, axes) for j in joints}
    return reg, cal, joints, ids, sign


def hold(args, dev, fingers):
    reg, cal, joints, ids, sign = setup(dev, fingers)
    bus = Bus(ids.values(), port=dev["port"])
    bus.torque_off()
    _, still = bus.clear_hardware_errors()
    if still:
        print(f"!! 有硬件错误没清掉: {still}")
    raw = bus.read_raw()
    zero = {}
    for j in joints:                                     # 零点换到电机当前所在的那一圈
        z = cal["joints"][j]["zero_raw"]
        zero[j] = z + round((raw[ids[j]] - z) / TICKS_PER_REV) * TICKS_PER_REV
    k = TICKS_PER_REV / (2 * math.pi)
    ang = lambda st: {j: sign[j] * ticks_to_rad(st[ids[j]][2] - zero[j]) for j in joints}
    goal = lambda q: {ids[j]: zero[j] + sign[j] * q[j] * k for j in joints}

    bus.enter_position_mode(args.limit, watchdog_ms=0)
    bus.set_position_gains(args.kp, args.kd)
    bus.check_torque(f"{dev['name']} ")
    q0 = ang(bus.read_state())
    print(f"* {dev['desc']}：位置模式，电流上限 {args.limit} mA，P {args.kp}，D {args.kd}；2 s 回零 ...")
    t0 = time.time()
    while True:
        a = min(1.0, (time.time() - t0) / 2.0)
        bus.write_positions(goal({j: q0[j] * (1 - a) for j in joints}))
        if a >= 1.0:
            break
        time.sleep(0.02)
    time.sleep(0.5)
    print("* 已回零并保持。可以用手掰手指再松开，看它回不回得到 0°。Ctrl+C 结束\n")

    stop = {"flag": False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(flag=True))
    peak_e = {j: 0.0 for j in joints}; peak_i = {j: 0 for j in joints}
    rows, t_rep, t_v, volt, hw = [], 0.0, 0.0, float("nan"), {}
    n_lines = len(joints) + 4
    print("\n" * n_lines, end="")
    t_start = time.time()
    try:
        while not stop["flag"]:
            bus.write_positions(goal({j: 0.0 for j in joints}))
            st = bus.read_state()
            q = ang(st)
            now = time.time()
            for j in joints:
                peak_e[j] = max(peak_e[j], abs(math.degrees(q[j])))
                peak_i[j] = max(peak_i[j], abs(st[ids[j]][0]))
            if args.log:
                rows.append([now - t_start] + [v for j in joints for v in (math.degrees(q[j]), st[ids[j]][0])])
            if now - t_v > 1.0:
                volt, hw, t_v = bus.voltage(), bus.hardware_errors(), now
            if now - t_rep > 0.25:
                lines = [f"{dev['name']}  电压 {volt:.1f} V   {'硬件保护 ' + str(hw) if hw else ''}",
                         f"{'ID':>3} {'关节':22s} {'偏差°':>7s} {'电流mA':>7s} {'最大偏差°':>9s} {'最大电流mA':>10s}"]
                for j in joints:
                    i = ids[j]
                    e = math.degrees(q[j])
                    lines.append(f"{i:3d} {j:22s} {e:+7.2f} {st[i][0]:+7d} {peak_e[j]:9.2f} {peak_i[j]:10d}" + ("  <- 没回到零" if abs(e) > 1 else ""))
                lines += ["", ""]
                print(f"\033[{n_lines}F" + "\n".join(l + "\033[K" for l in lines), flush=True)
                t_rep = now
            time.sleep(0.01)
    finally:
        bus.safe_stop(); bus.close()
        print("\n* 已清零电流、关扭矩")
        print("==== 汇总 ====")
        for j in joints:
            print(f"  {j:22s} 最大偏差 {peak_e[j]:6.2f}°   最大电流 {peak_i[j]:5d} mA")
        if args.log and rows:
            os.makedirs(os.path.dirname(os.path.abspath(args.log)), exist_ok=True)
            with open(args.log, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["t"] + [f"{j}:{x}" for j in joints for x in ("err_deg", "current_mA")])
                w.writerows(rows)
            print(f"* 已保存 {len(rows)} 行到 {args.log}")


def stops(args, dev, fingers):
    reg, cal, joints, ids, sign = setup(dev, fingers)
    path = os.path.join(gm.ROOT, "config", f"stops_{dev['name']}.json")
    old = json.load(open(path)) if args.stops == "compare" and os.path.exists(path) else None
    if args.stops == "compare" and old is None:
        sys.exit(f"没有 {os.path.relpath(path)}，先在测试之前跑一次 --stops save")
    bus = Bus(ids.values(), port=dev["port"])
    bus.torque_off()
    print(f"* {dev['desc']}：扭矩已关。每个关节推到同一头的机械限位，顶住按回车。"
          f"默认推到【握拳 / 向拇指侧】那头；那头够不着输入 x 改推另一头；s 跳过\n")
    out = {}
    try:
        for j in joints:
            side = old[j]["side"] if old and j in old else "+"
            if old and j not in old:
                continue
            label = gm.physical_positive(j) if side == "+" else gm.physical_negative(j)
            ans = input(f"  {j:22s} 推到【{label}】的限位，顶住回车 ... ").strip().lower()
            if ans == "s":
                continue
            if ans == "x" and not old:
                side = "-"
                input(f"  {j:22s} 改推到【{gm.physical_negative(j)}】的限位，顶住回车 ... ")
            raw, _ = bus.read_avg(15)
            a = math.degrees(sign[j] * ticks_to_rad(wrap_ticks(raw[ids[j]] - cal["joints"][j]["zero_raw"])))
            out[j] = {"side": side, "deg": a}
            if old:
                dd = a - old[j]["deg"]
                print(f"      现在 {a:+7.2f}°，之前 {old[j]['deg']:+7.2f}°，差 {dd:+6.2f}°" + ("  <- 零点漂了" if abs(dd) > 1 else ""))
            else:
                print(f"      {a:+7.2f}°")
    finally:
        bus.close()
    if not old:
        json.dump(out, open(path, "w"), indent=1)
        print(f"\n* 已保存 {os.path.relpath(path)}。大电流测试之后用 --stops compare 再推一遍")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("device", help="A / B / H（见 config/devices.json）")
    ap.add_argument("fingers", nargs="*", help="thumb / index / mid / ring，默认全部")
    ap.add_argument("--limit", type=int, default=1750, help="电流上限 mA（默认 1750 = XL330 上限）")
    ap.add_argument("--kp", type=int, default=800)
    ap.add_argument("--kd", type=int, default=50)
    ap.add_argument("--log", default=None)
    ap.add_argument("--stops", choices=["save", "compare"], default=None)
    args = ap.parse_args()
    dev = devices.get(args.device)
    fingers = args.fingers or list(gm.FINGERS)
    bad = [f for f in fingers if f not in gm.FINGERS]
    if bad:
        sys.exit(f"不认识的手指 {bad}")
    (stops if args.stops else hold)(args, dev, fingers)


if __name__ == "__main__":
    main()
