#!/usr/bin/env python3
"""
零点对齐：把从手（B）某几个关节的零点改成和主手（A）一致。
用法：两只手套摆成同一个姿态（只要这几个关节的姿态一样就行），然后

  python tools/match_zero.py thumb_cmc_abd_motor                      # 对齐一个关节
  python tools/match_zero.py thumb_cmc_abd_motor thumb_cmc_flex_motor # 多个
  python tools/match_zero.py --finger thumb                           # 整根手指
  python tools/match_zero.py --finger thumb --finger index --poses 2  # 拇指+食指，摆两个姿态取平均（推荐：捏合 + 张开）

原理：读 A 的关节角 qA（按 A 的标定），读 B 的编码器 rawB，令 B 的零点 = rawB − sign_B·qA，这样 B 现在也读出 qA。
只改零点，不改方向；方向不对的话还是要用 calibrate.py --joint 重标。扭矩全程关闭。
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import glove_model as gm  # noqa: E402
from dxl_bus import Bus, TICKS_PER_REV, effective_sign, load_calibration, save_calibration, load_registry, ticks_to_rad, wrap_ticks  # noqa: E402

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("joints", nargs="*")
ap.add_argument("--finger", action="append", default=[])
ap.add_argument("--leader-port", default="/dev/ttyACM0")
ap.add_argument("--follower-port", default="/dev/ttyUSB0")
ap.add_argument("--poses", type=int, default=1, help="摆几个不同姿态取平均（默认 1）")
args = ap.parse_args()
joints = list(args.joints) + [j for f in args.finger for j in gm.FINGERS[f]]
if not joints:
    sys.exit("要对齐哪些关节？例如  python tools/match_zero.py thumb_cmc_abd_motor")
reg, calA, calB, axes = load_registry(), load_calibration("A"), load_calibration("B"), gm.joint_axes()
for j in joints:
    if j not in calA["joints"] or j not in calB["joints"]:
        sys.exit(f"{j} 两边都要先标定过（A: {j in calA['joints']}，B: {j in calB['joints']}）")
ids = [reg[j] for j in joints]
A = Bus(ids, port=args.leader_port); A.torque_off()
B = Bus(ids, port=args.follower_port); B.torque_off()
B.clear_hardware_errors()
hints = ["拇指尖碰到食指尖（捏合）", "手张开", "半握"]
zeros = {j: [] for j in joints}
for k in range(args.poses):
    hint = hints[k] if k < len(hints) else f"第 {k + 1} 个姿态"
    input(f"* [{k + 1}/{args.poses}] 先用手把 B（桌上那只）摆成：{hint}，按回车 -> B 会自己锁住这个姿态 ...")
    rawB, _ = B.read_avg(10)
    B.enter_position_mode(200)                                   # B 自己保持姿态，腾出你的手
    B.write_positions({i: rawB[i] for i in ids})
    input(f"      B 已锁住。现在戴上 A，摆成同样的姿态，按回车 ...")
    rawA, _ = A.read_avg(10); rawB, _ = B.read_avg(10)
    B.safe_stop()                                                # 松开 B
    for j in joints:
        i = reg[j]; cA, cB = calA["joints"][j], calB["joints"][j]
        qA = effective_sign(cA, j, axes) * ticks_to_rad(wrap_ticks(rawA[i] - cA["zero_raw"]))
        qB_old = effective_sign(cB, j, axes) * ticks_to_rad(wrap_ticks(rawB[i] - cB["zero_raw"]))
        z = rawB[i] - effective_sign(cB, j, axes) * qA * TICKS_PER_REV / (2 * math.pi)
        zeros[j].append(z)
        print(f"      {j:22s} A = {math.degrees(qA):+6.1f}°，B 读 {math.degrees(qB_old):+6.1f}°（差 {math.degrees(qB_old - qA):+5.1f}°）")
for j in joints:
    zs = zeros[j]; ref = zs[0]
    zs = [ref + wrap_ticks(z - ref) for z in zs]                 # 处理 0/4095 回绕后再平均
    spread = (max(zs) - min(zs)) * 360 / TICKS_PER_REV
    calB["joints"][j]["zero_raw"] = float(sum(zs) / len(zs) % TICKS_PER_REV)
    print(f"  {j:22s} 新零点 {calB['joints'][j]['zero_raw']:.0f}" + (f"，{len(zs)} 个姿态之间的差异 {spread:.1f}°（> 3° 说明方向或机构有问题）" if len(zs) > 1 else ""))
save_calibration(calB, "B")
print("* 已写入 config/calibration_B.json。接着用 bilateral.py --dry-run 确认两边角度一致、方向一致")
A.close(); B.close()
