#!/usr/bin/env python3
"""
用机械限位把手套 B 的每个关节零点对齐到手套 A（两边同时、逐个关节，扭矩全程关闭）。

  python tools/align_stops.py thumb index                 # 拇指 + 食指全部关节
  python tools/align_stops.py --joint index_pip_motor     # 只做一个关节
  python tools/align_stops.py thumb --dry                 # 只看结果，不写文件

原理：两只手套是同一个 CAD，每个关节两头的机械限位在两边是同一个物理角度。对每个关节：
  1. 两只手套的这个关节都推到【握拳 / 向拇指侧】那一头的限位，顶住回车
  2. 都推到【反方向】那一头的限位，顶住回车（某一头够不到或会撞，输入 s 跳过）
  B 的偏移 = 两头 (A 读数 − B 读数) 的平均 —— 让这两个参照姿态误差平方和最小的那个偏移。
  两头的差值不一样（> 3°）说明两边行程不同（连杆 / 装配 / 限位件不一样），偏移只能修一半，脚本会提示。
只改 B 的零点，不改方向，不改 A。前提：两边方向都已标对（先跑过 calibrate.py）。
其他关节放哪都行，摆开别撞就好；不用戴手套，两只手各推一只。
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import glove_model as gm  # noqa: E402
from dxl_bus import (Bus, TICKS_PER_REV, effective_sign, load_calibration, load_registry,  # noqa: E402
                     save_calibration, ticks_to_rad, wrap_ticks)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("fingers", nargs="*", help="thumb / index / mid / ring，默认 thumb index")
    ap.add_argument("--joint", action="append", default=[], help="只做这些关节（可重复）")
    ap.add_argument("--a-port", default="/dev/ttyACM0")
    ap.add_argument("--b-port", default="/dev/ttyUSB0")
    ap.add_argument("--dry", action="store_true", help="只显示结果，不写 calibration_B.json")
    args = ap.parse_args()
    fingers = args.fingers or ["thumb", "index"]
    joints = args.joint or [j for f in fingers for j in gm.FINGERS[f]]
    reg, axes = load_registry(), gm.joint_axes()
    calA, calB = load_calibration("A"), load_calibration("B")
    joints = [j for j in joints if j in calA["joints"] and j in calB["joints"]]
    if not joints:
        sys.exit("没有两边都标定过的关节，先用 calibrate.py 标方向")
    ids = [reg[j] for j in joints]
    A = Bus(ids, port=args.a_port); A.torque_off()
    B = Bus(ids, port=args.b_port); B.torque_off()
    sA = {j: effective_sign(calA["joints"][j], j, axes) for j in joints}
    sB = {j: effective_sign(calB["joints"][j], j, axes) for j in joints}

    def read(bus, cal, s):
        raw, _ = bus.read_avg(15)
        return {j: math.degrees(s[j] * ticks_to_rad(wrap_ticks(raw[reg[j]] - cal["joints"][j]["zero_raw"])))
                for j in joints}

    print(f"* 两边扭矩已关。{len(joints)} 个关节，每个关节推两头的限位。其他关节摆开别撞。\n")
    result = {}
    try:
        for n, j in enumerate(joints, start=1):
            ends = [("+", gm.physical_positive(j)), ("−", "反方向（伸直 / 远离拇指）")]
            diffs, qa, qb = [], [], []
            print(f"== [{n}/{len(joints)}] {j}（ID {reg[j]}）")
            for tag, label in ends:
                ans = input(f"   两只手套的 {j} 都推到【{label}】那一头的限位，顶住，按回车（s 跳过这一端）... ").strip().lower()
                if ans == "s":
                    continue
                a, b = read(A, calA, sA)[j], read(B, calB, sB)[j]
                qa.append(a); qb.append(b); diffs.append(a - b)
                print(f"      A {a:+7.1f}°   B {b:+7.1f}°   差 {a - b:+5.1f}°")
            if not diffs:
                print("      两头都跳过了，这个关节不改\n")
                continue
            off = sum(diffs) / len(diffs)
            note = ""
            if len(diffs) == 2:
                spanA, spanB = qa[0] - qa[1], qb[0] - qb[1]
                if spanA * spanB < 0:
                    print(f"      !! 两边行程方向相反（A {spanA:+.0f}°，B {spanB:+.0f}°）：有一边方向标反了，这个关节不改，先用 calibrate.py --joint 重标\n")
                    continue
                spread = abs(diffs[0] - diffs[1])
                note = f"，A 行程 {spanA:.0f}°，B 行程 {spanB:.0f}°，两头差值相差 {spread:.1f}°"
                if spread > 3:
                    note += "  <- 两边行程不一样（连杆 / 装配 / 限位件），偏移只能把两头的误差平分"
            result[j] = off
            print(f"      B 偏移 {off:+.1f}°{note}\n")
    finally:
        A.close(); B.close()

    if not result:
        print("* 没有要改的关节")
        return
    print("==== 汇总（B 读数要加上的偏移）====")
    for j, o in result.items():
        print(f"  {j:24s} {o:+6.1f}°")
    if args.dry:
        print("* --dry：没有写文件")
        return
    if input("写入 config/calibration_B.json？[y/N] ").strip().lower() != "y":
        print("* 没有写文件")
        return
    k = TICKS_PER_REV / (2 * math.pi)
    for j, o in result.items():
        # B 的新读数 = 旧读数 + o  ->  零点挪 −sign·o 个 tick
        z = calB["joints"][j]["zero_raw"] - sB[j] * math.radians(o) * k
        calB["joints"][j]["zero_raw"] = float(z % TICKS_PER_REV)
    save_calibration(calB, "B")
    print("* 已写入。用 python tools/twin_jog.py 逐个关节验收；config/map_AB.json 是按旧零点拟合的，先别用 --map")


if __name__ == "__main__":
    main()
