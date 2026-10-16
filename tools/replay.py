#!/usr/bin/env python3
"""
把 bilateral.py --log 记录的一次抓取在从手上回放（人不在环），演示"力 / 刚度作为动作"：

  python tools/replay.py data/grasp_soft.csv --mode pos          # 只回放位置指令 q*，电流上限固定（= --limit）
  python tools/replay.py data/grasp_soft.csv --mode pos+force    # 方法 2：位置 + 力。每周期把记录的从手电流写成电流上限（Goal Current）
  python tools/replay.py data/adaptive.csv  --mode pos+kp        # 方法 3：位置 + 刚度。每周期回放记录的 P 增益（要 --adaptive-kp 记的 CSV）
  python tools/replay.py data/grasp_soft.csv --mode pos+force --log data/replay_force.csv   # 记录回放时的从手状态
  之后： python tools/plot_log.py data/grasp_soft.csv data/replay_pos.csv data/replay_force.csv   # 人在环 vs 两种回放

演示建议：抓一个软的东西（海绵、纸杯）。pos 模式下从手只管到位，把软物压扁或把电流顶到上限；
pos+force 模式下从手复现"人在力反馈下让机器人出的力"，软物保持原样。

安全：回放前先用小电流慢慢走到第一帧姿态；Ctrl+C 或出错时关扭矩。
"""
import argparse
import math
import os
import signal
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import glove_model as gm  # noqa: E402
from dxl_bus import Bus, MODEL_NAME, TICKS_PER_REV, effective_sign, load_calibration, load_registry, ticks_to_rad  # noqa: E402
from plot_log import load  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("--mode", choices=["pos", "pos+force", "pos+kp"], default="pos")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--glove", default="B")
    ap.add_argument("--limit", type=int, default=300, help="电流上限 mA（pos / pos+kp 固定用它；pos+force 里是力指令的上限）")
    ap.add_argument("--free-current", type=int, default=100, help="pos+force：空中跟踪位置所需的最小电流上限 mA（默认 100）")
    ap.add_argument("--force-lpf", type=float, default=10.0, help="pos+force：记录电流的低通 Hz（默认 10）")
    ap.add_argument("--kp", type=int, default=None, help="固定 P 增益（pos / pos+force；默认用 CSV 里记录的 follower_kp）")
    ap.add_argument("--kd", type=int, default=None)
    ap.add_argument("--speed", type=float, default=1.0, help="回放速度倍率")
    ap.add_argument("--log", default=None, help="把回放时从手的状态存成 CSV（格式和 bilateral.py 一样，qA 列 = 回放的指令）")
    ap.add_argument("--dry-run", action="store_true", help="不开扭矩、不写电机，只走一遍")
    args = ap.parse_args()

    d = load(args.csv)
    joints, t_rec = d["joints"], d["t"]
    if args.mode == "pos+kp" and any(d[j].get("kpB") is None for j in joints):
        sys.exit("CSV 里没有 kpB 列：要用  bilateral.py --adaptive-kp ... --log  记录的文件")
    reg, cal, axes = load_registry(), load_calibration(args.glove), gm.joint_axes()
    missing = [j for j in joints if j not in cal["joints"]]
    if missing:
        sys.exit(f"从手（{args.glove}）没标定这些关节: {missing}")
    ids = {j: reg[j] for j in joints}
    sB = {j: effective_sign(cal["joints"][j], j, axes) for j in joints}
    kp0 = args.kp or int(float(d["settings"].get("follower_kp", 400)))

    # 力指令（pos+force）：记录的从手电流取绝对值、低通，再和"空中能动"的最小电流取大
    force_cmd = {}
    for j in joints:
        i_abs = np.abs(d[j]["iB"]); f = np.empty_like(i_abs); acc = float(i_abs[0])
        for k in range(len(i_abs)):
            dt = (t_rec[k] - t_rec[k - 1]) if k else 0.005
            a = dt / (dt + 1 / (2 * math.pi * args.force_lpf))
            acc += a * (i_abs[k] - acc); f[k] = acc
        force_cmd[j] = np.clip(np.maximum(f, args.free_current), 0, args.limit)

    bus = Bus(ids.values(), port=args.port)
    bus.torque_off()
    cleared, still = bus.clear_hardware_errors()
    if still:
        bus.close(); sys.exit(f"硬件错误没清掉: {still}")
    raw0 = bus.read_raw()
    zB = {}
    for j in joints:
        z = cal["joints"][j]["zero_raw"]
        zB[j] = z + round((raw0[ids[j]] - z) / TICKS_PER_REV) * TICKS_PER_REV
    models = {j: bus.model(ids[j]) for j in joints}

    def goal_ticks(k):
        return {ids[j]: zB[j] + sB[j] * d[j]["qA"][k] * TICKS_PER_REV / (2 * math.pi) for j in joints}

    print(f"* 回放 {args.csv}：{len(t_rec)} 帧 / {t_rec[-1] - t_rec[0]:.1f} s，模式 {args.mode}，{len(joints)} 个关节，"
          f"P 增益 {'按记录的 kpB' if args.mode == 'pos+kp' else kp0}，电流上限 {args.limit} mA"
          + (f"，空中最小电流 {args.free_current} mA" if args.mode == "pos+force" else ""))
    log = []
    try:
        if not args.dry_run:
            bus.enter_position_mode(args.limit)
            bus.set_position_gains(kp0, args.kd)
            bus.write_currents({ids[j]: args.free_current for j in joints})     # 先用小电流慢慢走到第一帧
            bus.write_positions(goal_ticks(0))
            time.sleep(1.5)
            if args.mode != "pos+force":
                bus.write_currents({ids[j]: args.limit for j in joints})
            print("* 扭矩已开，开始回放。Ctrl+C 停止")
        t0 = time.time(); t_gain, t_report, k = t0, t0, 0
        kp_written = {j: float(kp0) for j in joints}
        while k < len(t_rec):
            now = time.time()
            target = t0 + (t_rec[k] - t_rec[0]) / args.speed
            if now < target:
                time.sleep(min(0.002, target - now)); continue
            if not args.dry_run:
                bus.write_positions(goal_ticks(k))
                if args.mode == "pos+force":
                    bus.write_currents({ids[j]: float(force_cmd[j][k]) for j in joints})
                elif args.mode == "pos+kp" and now - t_gain > 0.02:
                    changed = {ids[j]: d[j]["kpB"][k] for j in joints if abs(d[j]["kpB"][k] - kp_written[j]) >= 5}
                    if changed:
                        bus.write_p_gains(changed)
                        for j in joints:
                            if ids[j] in changed:
                                kp_written[j] = float(d[j]["kpB"][k])
                    t_gain = now
            st = bus.read_state()
            if args.log is not None:
                row = [now - t0]
                for j in joints:
                    i = ids[j]
                    kp_now = d[j]["kpB"][k] if args.mode == "pos+kp" else kp0
                    row += [d[j]["qA"][k], 0.0, 0, 0, sB[j] * ticks_to_rad(st[i][2] - zB[j]), sB[j] * st[i][1], st[i][0], round(kp_now)]
                log.append(row)
            if now - t_report > 0.25:
                tot = sum(abs(st[i][0]) for i in ids.values())
                fc = (f"  力指令合计 {sum(force_cmd[j][k] for j in joints):5.0f} mA" if args.mode == "pos+force" else "")
                print(f"\r  t = {now - t0:5.1f} s  帧 {k:5d}/{len(t_rec)}  从手电流合计 {tot:5.0f} mA{fc}", end="", flush=True)
                t_report = now
            k += 1
        print("\n* 回放完成")
    except KeyboardInterrupt:
        print("\n* 手动停止")
    except Exception as e:
        print(f"\n!! 出错，已停止: {e}")
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if args.log is not None and log:
            import csv
            os.makedirs(os.path.dirname(os.path.abspath(args.log)), exist_ok=True)
            with open(args.log, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["t"] + [f"{j}:{kk}" for j in joints for kk in ("qA", "dqA", "iA_meas", "iA_cmd", "qB", "dqB", "iB_meas", "kpB")])
                w.writerow(["# models"] + [f"{j}:{MODEL_NAME[models[j]]}" for j in joints])
                w.writerow(["# settings", f"follower_kp:{kp0}", f"follower_kd:{args.kd or 0}", f"follower_limit_mA:{args.limit}",
                            "leader_kp:0", "deadband_deg:0", f"replay_mode:{args.mode}", f"source:{os.path.basename(args.csv)}"])
                w.writerows(log)
            print(f"* 已保存 {len(log)} 行到 {args.log}")
        if not args.dry_run:
            bus.safe_stop()
            print("* 已清零电流并关扭矩")
        bus.close()


if __name__ == "__main__":
    main()
