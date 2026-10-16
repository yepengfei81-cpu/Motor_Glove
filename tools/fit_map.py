#!/usr/bin/env python3
"""
拟合主手 -> 从手的关节映射（每个关节 q_从 = a·q_主 + b），解决"主手闭合时人的指尖先碰上、从手指套还差一截"的问题。

  python tools/fit_map.py --finger thumb --finger index                          # 主手 A(ttyACM0) -> 从手 B(ttyUSB0)
  python tools/fit_map.py --finger thumb --finger index --closed 3               # 张开 + 3 种捏合，最小二乘拟合（推荐）
  python tools/fit_map.py --finger thumb --finger index --leader-port /dev/ttyUSB0 --leader-glove B --follower-port /dev/ttyACM0 --follower-glove A

流程（戴着主手遥操作，不用碰从手）：
  1. 张开：拇指食指张开，从手跟着，回车
  2. 捏合：捏到你的指尖碰上，回车；脚本让从手沿同一方向继续合，直到它的指套碰上（电流上来、走不动）自动停下并记录
结果存到 config/map_<主><从>.json，之后 bilateral.py --map config/map_AB.json 使用。
"""
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import glove_model as gm  # noqa: E402
from dxl_bus import Bus, effective_sign, load_calibration, load_registry, ticks_to_rad, wrap_ticks  # noqa: E402

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--finger", action="append", default=[])
ap.add_argument("--leader-port", default="/dev/ttyACM0"); ap.add_argument("--leader-glove", default="A")
ap.add_argument("--follower-port", default="/dev/ttyUSB0"); ap.add_argument("--follower-glove", default="B")
ap.add_argument("--closed", type=int, default=1, help="采几次不同的捏合姿态（默认 1；建议 3：指尖对指尖、指腹对指腹、偏侧）")
args = ap.parse_args()
fingers = args.finger or ["thumb", "index"]
joints = [j for f in fingers for j in gm.FINGERS[f]]
reg, axes = load_registry(), gm.joint_axes()
calL, calF = load_calibration(args.leader_glove), load_calibration(args.follower_glove)
joints = [j for j in joints if j in calL["joints"] and j in calF["joints"]]
ids = [reg[j] for j in joints]
L = Bus(ids, port=args.leader_port); L.torque_off()
F = Bus(ids, port=args.follower_port); F.torque_off(); F.clear_hardware_errors()


def angles(bus, cal, n=1):
    raw = bus.read_raw() if n == 1 else bus.read_avg(n)[0]
    return {j: effective_sign(cal["joints"][j], j, axes) * ticks_to_rad(wrap_ticks(raw[reg[j]] - cal["joints"][j]["zero_raw"])) for j in joints}


import time  # noqa: E402
from dxl_bus import TICKS_PER_REV  # noqa: E402
sF = {j: effective_sign(calF["joints"][j], j, axes) for j in joints}
raw0, _ = F.read_avg(3)
zF = {j: calF["joints"][j]["zero_raw"] + round((raw0[reg[j]] - calF["joints"][j]["zero_raw"]) / TICKS_PER_REV) * TICKS_PER_REV for j in joints}


def goal_ticks(q):
    return {reg[j]: zF[j] + sF[j] * q[j] * TICKS_PER_REV / (2 * math.pi) for j in joints}


def follower_angles():
    st = F.read_state()
    return ({j: sF[j] * ticks_to_rad(st[reg[j]][2] - zF[j]) for j in joints}, {j: abs(st[reg[j]][0]) for j in joints})


# 遥操作采集：从手跟着主手走，你只管戴着主手做动作
F.enter_position_mode(200, watchdog_ms=0)      # 这个工具里读写之间有停顿，不开看门狗
if F.check_torque("从手"):
    sys.exit("从手有电机开不了扭矩，先解决再采")
stop = {"flag": False}


def track(prompt):
    """从手跟随主手，直到你按回车；返回那一刻的主手角度"""
    import threading
    stop["flag"] = False
    th = threading.Thread(target=lambda: (input(prompt), stop.update(flag=True)), daemon=True); th.start()
    while not stop["flag"]:
        qL = angles(L, calL); F.write_positions(goal_ticks(qL)); time.sleep(0.01)
    return angles(L, calL, 10)


qL0 = track("* [1/2] 戴着主手，拇指食指张开，从手会跟着；稳住后按回车 ...")
time.sleep(0.5); qF0, _ = follower_angles()
print("      " + "  ".join(f"{j.split('_', 1)[1]}: 主 {math.degrees(qL0[j]):+6.1f}° 从 {math.degrees(qF0[j]):+6.1f}°" for j in joints))
poses = [(qL0, qF0)]
hints = ["指尖对指尖", "指腹对指腹", "偏向食指侧面捏", "再换一个角度捏"]
for k in range(args.closed):
    qL1 = track(f"* [{k + 2}/{args.closed + 1}] 捏合（{hints[min(k, len(hints) - 1)]}）到你的指尖碰上，稳住后按回车 -> 从手会继续往里合直到指套碰上 ...")
    s_ = 1.0; qF1 = None
    while s_ < 1.8:
        s_ += 0.01
        goal = {j: qL0[j] + s_ * (qL1[j] - qL0[j]) for j in joints}
        F.write_positions(goal_ticks(goal)); time.sleep(0.05)
        qF, iF = follower_angles()
        lag = {j: abs(goal[j] - qF[j]) for j in joints}
        blocked = [j for j in joints if iF[j] > 120 and lag[j] > math.radians(4)]
        if blocked:
            time.sleep(0.3); qF1, _ = follower_angles()
            print(f"      从手在 s = {s_:.2f} 处碰上了（被挡住的关节: {', '.join(x.split('_', 1)[1] for x in blocked)}）")
            break
    if qF1 is None:
        qF1, _ = follower_angles(); print("      !! 合到 1.8 倍都没碰上，用最后的姿态；检查从手的指套是不是对不上")
    poses.append((qL1, qF1))
    # 下一次捏合前先把从手放回跟随（张开时它会跟着你张开）
F.safe_stop()
m = {}
for j in joints:
    xs = [qL[j] for qL, _ in poses]; ys = [qF[j] for _, qF in poses]
    n = len(xs); mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx > math.radians(5) ** 2:                                   # 主手这个关节有足够行程才拟合斜率，否则只平移
        a = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
        a = max(0.5, min(2.0, a))
    else:
        a = 1.0
    b = my - a * mx
    resid = max(abs(y - (a * x + b)) for x, y in zip(xs, ys))
    m[j] = {"a": a, "b": b}
    print(f"  {j:22s} q从 = {a:.2f}·q主 {b * 180 / math.pi:+.1f}°   （{n} 个姿态，最大残差 {math.degrees(resid):.1f}°）")
out = os.path.join(gm.ROOT, "config", f"map_{args.leader_glove}{args.follower_glove}.json")
json.dump(m, open(out, "w"), indent=1)
print(f"* 已存 {os.path.relpath(out)}；使用：python tools/bilateral.py thumb index --map {os.path.relpath(out)}")
L.close(); F.close()
