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
从手是机械手：python tools/mg.py fitmap A H --closed 6   （拇指依次捏食指、中指、无名指；拟合在手套空间，遥操作时再套手套->机械手映射）
自动判断"碰上"要拇指和对方手指两边同时被顶住并持续 0.2 s；机械手关节有摩擦老是误判就 --block-ma 200，或者 --manual 自己看着按回车。
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
ap.add_argument("--follower-model", choices=["glove", "hand"], default="glove", help="从手是手套还是直驱机械手")
ap.add_argument("--hint", action="append", default=None, help="每次捏合的提示语，按顺序（默认按手指自动生成）")
ap.add_argument("--block-ma", type=float, default=150, help="从手关节电流超过这个值（mA）且落后目标才算被挡住（默认 150；有摩擦的关节误判就调大）")
ap.add_argument("--block-lag", type=float, default=6, help="从手关节落后目标超过这个角度（度）才算被挡住（默认 6）")
ap.add_argument("--manual", action="store_true", help="不自动判断碰没碰上：从手慢慢合，你看着它的指尖碰上了按回车")
args = ap.parse_args()
fingers = args.finger or ["thumb", "index"]
joints = [j for f in fingers for j in gm.FINGERS[f]]
reg, axes = load_registry(), gm.joint_axes()
axes_f = gm.joint_axes(gm.scene_of(args.follower_model))
calL, calF = load_calibration(args.leader_glove), load_calibration(args.follower_glove)
joints = [j for j in joints if j in calL["joints"] and j in calF["joints"]]
ids = [reg[j] for j in joints]
from retarget import JointMap  # noqa: E402
g2h = JointMap(joints, to_hand=(args.follower_model == "hand"))     # 只含手套->机械手那一段；拟合在手套空间里做
L = Bus(ids, port=args.leader_port); L.torque_off()
F = Bus(ids, port=args.follower_port); F.torque_off(); F.clear_hardware_errors()


def angles(bus, cal, n=1, ax=axes):
    raw = bus.read_raw() if n == 1 else bus.read_avg(n)[0]
    return {j: effective_sign(cal["joints"][j], j, ax) * ticks_to_rad(wrap_ticks(raw[reg[j]] - cal["joints"][j]["zero_raw"])) for j in joints}


import time  # noqa: E402
from dxl_bus import TICKS_PER_REV  # noqa: E402
sF = {j: effective_sign(calF["joints"][j], j, axes_f) for j in joints}
raw0, _ = F.read_avg(3)
zF = {j: calF["joints"][j]["zero_raw"] + round((raw0[reg[j]] - calF["joints"][j]["zero_raw"]) / TICKS_PER_REV) * TICKS_PER_REV for j in joints}


def goal_ticks(q):
    """q 是手套空间的目标；从手是机械手时先换到机械手空间"""
    q = g2h(q)
    return {reg[j]: zF[j] + sF[j] * q[j] * TICKS_PER_REV / (2 * math.pi) for j in joints}


def follower_angles():
    """返回手套空间的从手角度（机械手的读数反变换回来），和各关节电流"""
    st = F.read_state()
    q = {j: sF[j] * ticks_to_rad(st[reg[j]][2] - zF[j]) for j in joints}
    return (g2h.hand_to_glove(q), {j: abs(st[reg[j]][0]) for j in joints})


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


qL0 = track(f"* [1/{args.closed + 1}] 戴着主手，手指全部张开，从手会跟着；稳住后按回车 ...")
time.sleep(0.5); qF0, _ = follower_angles()
print("      " + "  ".join(f"{j.split('_', 1)[1]}: 主 {math.degrees(qL0[j]):+6.1f}° 从 {math.degrees(qF0[j]):+6.1f}°" for j in joints))
poses = [(qL0, qF0)]
others = [f for f in fingers if f != "thumb"]
if args.hint:
    hints = args.hint
elif len(others) > 1:
    hints = [f"拇指捏{ {'index': '食指', 'mid': '中指', 'ring': '无名指'}[f] }" for f in others] * 3
else:
    hints = ["指尖对指尖", "指腹对指腹", "偏向食指侧面捏", "再换一个角度捏"]
for k in range(args.closed):
    qL1 = track(f"* [{k + 2}/{args.closed + 1}] 捏合（{hints[min(k, len(hints) - 1)]}）到你的指尖碰上，稳住后按回车 -> 从手会继续往里合直到它的指尖碰上 ...")
    s_ = 1.0; qF1 = None; n_block = 0
    moving = [j for j in joints if abs(qL1[j] - qL0[j]) > math.radians(5)]              # 只看这次捏合里真的在动的关节
    if args.manual:
        stop["flag"] = False
        import threading
        threading.Thread(target=lambda: (input("      从手正在慢慢合，看到它的指尖碰上就按回车 ..."), stop.update(flag=True)), daemon=True).start()
    while s_ < 1.8:
        s_ += 0.01 if not args.manual else 0.004
        goal = {j: qL0[j] + s_ * (qL1[j] - qL0[j]) for j in joints}
        F.write_positions(goal_ticks(goal)); time.sleep(0.05)
        qF, iF = follower_angles()
        if args.manual:
            if stop["flag"]:
                qF1, _ = follower_angles(); print(f"      s = {s_:.2f} 处你确认碰上了"); break
            continue
        lag = {j: abs(goal[j] - qF[j]) for j in joints}
        blocked = [j for j in moving if iF[j] > args.block_ma and lag[j] > math.radians(args.block_lag)]
        # 指尖对指尖真碰上时，拇指和对方手指两边都会被顶住；只有一边被挡住多半是那个关节自己的摩擦
        both = any(j.startswith("thumb") for j in blocked) and any(not j.startswith("thumb") for j in blocked)
        n_block = n_block + 1 if both else 0
        if n_block >= 4:                                                                 # 持续 0.2 s 才算
            time.sleep(0.3); qF1, _ = follower_angles()
            print(f"      从手在 s = {s_:.2f} 处碰上了（被挡住的关节: {', '.join(x.split('_', 1)[1] for x in blocked)}）")
            break
    if qF1 is None:
        qF1, _ = follower_angles(); print("      !! 合到 1.8 倍都没碰上，用最后的姿态；检查从手的指尖是不是对不上，或者用 --manual 自己看着确认")
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
print(f"* 已存 {os.path.relpath(out)}；使用：python tools/mg.py teleop {args.leader_glove} {args.follower_glove} --map {os.path.relpath(out)}")
L.close(); F.close()
