#!/usr/bin/env python3
"""
双手套主从遥操作 + 力反馈：手套 A（戴在手上，主）-> 手套 B（放在桌上，从）。

  python tools/bilateral.py --dry-run                 # 只读主手，算出来但不写任何电机
  python tools/bilateral.py                           # 正式：主手电流模式，从手跟随；从手被挡住时主手感到力
  python tools/bilateral.py index mid ring            # 只用几根手指
  python tools/bilateral.py --leader-port /dev/ttyACM0 --follower-port /dev/ttyUSB0
  python tools/bilateral.py --log data/grasp_soft.csv  # 记录每个周期：主/从位置、速度、电流 -> 之后用 plot_log.py 画图
  python tools/bilateral.py --follower-kp 150 --log data/kp150.csv   # 从手设成"软"的（P 增益 150，默认 400）
  python tools/bilateral.py --adaptive-kp 600 100 --log data/adaptive.csv
        # 方法 3 演示：从手刚度随接触电流变化 K_t = f(τ_t)。空中 P=600（硬、跟得准），接触电流滑动平均
        # 达到 --adapt-ref（默认 150 mA）时降到 P=100（软）。每周期的 P 增益记在 CSV 的 kpB 列，plot_log.py 会画出来

前提：两只手套各在一条总线上（例如 A 接 OpenRB，B 接 U2D2 + 另一块 OpenRB 供电），
     ID 都是 1–13（同一张表），B 用  calibrate.py <手指> --glove B --port <B 的串口>  标定过。

控制：
  从手：模式 5（电流限制的位置控制），目标位置 = 主手关节角换算到从手的编码器；电流上限 = 抓握力上限。
  主手：电流模式。反馈力矩 τ = -kp·(|e|-死区)·sign(e)，e = q_主 - q_从：从手被挡住，e 变大，主手被往回推。
       另外可以叠加从手电流：τ += -kc·(I_从 - I_th)（DITTO 的做法），默认 kc = 0。
  自由运动时从手跟得上，e 在死区内，主手不出力。
安全：电流上限写进电机；启动 1.5 s 渐进；总线看门狗 100 ms；退出或出错时两边都清零电流、关扭矩。
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
from dxl_bus import (Bus, KT, MODEL_NAME, TICKS_PER_REV, effective_sign, load_calibration, load_registry,  # noqa: E402
                     ticks_to_rad, wrap_ticks, default_port)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("fingers", nargs="*", help="thumb / index / mid / ring，默认全部")
    ap.add_argument("--leader-port", default=None, help="主手串口（默认 /dev/ttyACM0）")
    ap.add_argument("--follower-port", default=None, help="从手串口（默认 /dev/ttyUSB0；写 none 表示没有从手，只测主手）")
    ap.add_argument("--leader-glove", default="A")
    ap.add_argument("--follower-glove", default="B")
    ap.add_argument("--dry-run", action="store_true", help="不开扭矩、不写任何电机")
    ap.add_argument("--kp", type=float, default=0.5, help="位置误差反馈刚度 N·m/rad（默认 0.5）")
    ap.add_argument("--kc", type=float, default=0.0, help="从手电流反馈增益（主手力矩 / 从手力矩，默认 0 = 关）")
    ap.add_argument("--deadband", type=float, default=2.0, help="位置误差死区 度（默认 2）")
    ap.add_argument("--skip", nargs="*", default=[], metavar="JOINT", help="临时不同步的关节名，例如 --skip thumb_cmc_abd_motor")
    ap.add_argument("--map", default=None, help="fit_map.py 拟合的主->从关节映射 json：从手目标 = a·q主 + b，补偿指尖几何差异")
    ap.add_argument("--contact-ma", type=float, default=100, help="接触门控：从手电流到这个值（mA）时门控全开，一半时开始开；0 = 关掉门控（默认 100）")
    ap.add_argument("--contact-vel", type=float, default=0.8, help="接触门控：从手速度超过这个值（rad/s）视为自由运动不出力，低于 1/4 时视为被挡住（默认 0.8）")
    ap.add_argument("--max-torque", type=float, default=0.08, help="主手每个电机输出力矩上限 N·m（默认 0.08 ≈ 230 mA）")
    ap.add_argument("--friction-ma", type=float, default=0.0, help="主手摩擦补偿：顺着运动方向给这么多 mA 抵消减速箱反拖阻力（默认 0 = 关；试 20–30）")
    ap.add_argument("--leader-limit", type=int, default=500, help="主手电流上限 mA")
    ap.add_argument("--follower-limit", type=int, default=300, help="从手电流上限 mA（= 抓握力上限）")
    ap.add_argument("--budget", type=float, default=2500, help="每条总线电流总和上限 mA")
    ap.add_argument("--lpf", type=float, default=25, help="主手反馈力的低通 Hz（默认 25）")
    ap.add_argument("--hz", type=float, default=300)
    ap.add_argument("--log", default=None, help="把每个周期的数据存成 CSV")
    ap.add_argument("--follower-kp", type=int, default=None, help="从手位置环 P 增益（电机默认 400；低 = 软，高 = 硬），会记进 CSV")
    ap.add_argument("--follower-kd", type=int, default=None, help="从手位置环 D 增益（默认 0）")
    ap.add_argument("--gain", action="append", default=[], metavar="NAME=KP[,KD]",
                    help="单独给某些关节设从手 P/D 增益，NAME 是关节名或前缀，例如 --gain thumb=600,60 --gain index_pip_motor=900")
    ap.add_argument("--cmd-lpf", type=float, default=0.0,
                    help="从手目标角度的低通 Hz：滤掉主手编码器量化和手抖，减小从手抖动（0 = 关；试 10–15）")
    ap.add_argument("--adaptive-kp", type=int, nargs=2, metavar=("KP_MAX", "KP_MIN"), default=None,
                    help="方法 3：从手 P 增益随接触电流变化。空中 = KP_MAX，接触电流(滑动平均)达到 --adapt-ref 时降到 KP_MIN，例如 600 100")
    ap.add_argument("--adapt-ref", type=float, default=150, help="接触电流参考值 mA（滑动平均到这个值时刚度降到 KP_MIN，默认 150）")
    ap.add_argument("--adapt-tau", type=float, default=0.3, help="接触电流滑动平均的时间常数 s（默认 0.3；ACP 用 1 s）")
    args = ap.parse_args()
    fingers = args.fingers or list(gm.FINGERS)
    bad = [f for f in fingers if f not in gm.FINGERS]
    if bad:
        sys.exit(f"不认识的手指 {bad}")
    lport = args.leader_port or "/dev/ttyACM0"
    fport = None if (args.follower_port or "").lower() == "none" else (args.follower_port or "/dev/ttyUSB0")

    reg = load_registry()
    calA, calB = load_calibration(args.leader_glove), load_calibration(args.follower_glove)
    joints = [j for f in fingers for j in gm.FINGERS[f] if j in calA["joints"] and (fport is None or j in calB["joints"]) and j not in args.skip]
    if args.skip:
        print(f"* 按要求跳过: {args.skip}")
    if not joints:
        sys.exit("没有两只手套都标定过的关节；从手用  python tools/calibrate.py <手指> --glove B --port <串口>  标定")
    skipped = [j for f in fingers for j in gm.FINGERS[f] if j not in joints]
    if skipped:
        print(f"* 跳过（从手没标定）: {skipped}")
    axes = gm.joint_axes()
    ids = {j: reg[j] for j in joints}
    sA = {j: effective_sign(calA["joints"][j], j, axes) for j in joints}
    zA = {j: calA["joints"][j]["zero_raw"] for j in joints}
    db = math.radians(args.deadband)
    kp0 = args.adaptive_kp[0] if args.adaptive_kp else (args.follower_kp if args.follower_kp is not None else 400)
    kp_cmd = {j: float(kp0) for j in joints}       # 每个关节当前的从手 P 增益（固定或随电流变化）
    kp_written = dict(kp_cmd)
    iB_avg = {j: 0.0 for j in joints}

    leader = Bus(ids.values(), port=lport)
    leader.torque_off()
    cleared, still = leader.clear_hardware_errors()
    if still:
        leader.close()
        sys.exit(f"主手有硬件错误没清掉: {still}")
    models = {j: leader.model(ids[j]) for j in joints}
    follower = None
    if fport is not None:
        follower = Bus(ids.values(), port=fport)
        follower.torque_off()
        cleared, still = follower.clear_hardware_errors()
        if still:
            leader.close(); follower.close()
            sys.exit(f"从手有硬件错误没清掉: {still}")
        sB = {j: effective_sign(calB["joints"][j], j, axes) for j in joints}
        # 从手零点换算到"当前所在的那一圈"，避免上电位置跨过 0/4095 时转一整圈
        raw0 = follower.read_raw()
        zB = {}
        for j in joints:
            z = calB["joints"][j]["zero_raw"]
            zB[j] = z + round((raw0[ids[j]] - z) / TICKS_PER_REV) * TICKS_PER_REV
        fmodels = {j: follower.model(ids[j]) for j in joints}

    qmap = {}
    if args.map:
        import json
        qmap = {j: v for j, v in json.load(open(args.map)).items() if j in joints}
        print(f"* 关节映射 {args.map}：" + "，".join(f"{j.split('_', 1)[1]} a={v['a']:.2f} b={math.degrees(v['b']):+.1f}°" for j, v in qmap.items()))

    def leader_angles(st):
        """主手关节角；给了 --map 时按映射换算成从手的目标（力反馈的位置差也用换算后的值）"""
        q = {j: sA[j] * ticks_to_rad(wrap_ticks(st[ids[j]][2] - zA[j])) for j in joints}
        for j, v in qmap.items():
            q[j] = v["a"] * q[j] + v["b"]
        return q

    def follower_goal_ticks(q):
        return {ids[j]: zB[j] + sB[j] * q[j] * TICKS_PER_REV / (2 * math.pi) for j in joints}

    print(f"* 主手 {lport}（{args.leader_glove}），从手 {fport or '无'}（{args.follower_glove}）；{len(joints)} 个关节")
    print(f"* kp {args.kp} N·m/rad，kc {args.kc}，死区 {args.deadband}°，接触门控 {args.contact_ma:.0f} mA / {args.contact_vel} rad/s，"
          f"主手力矩上限 {args.max_torque} N·m，摩擦补偿 {args.friction_ma:.0f} mA，从手电流上限 {args.follower_limit} mA")
    if args.adaptive_kp:
        print(f"* 从手刚度随接触电流变化：P {args.adaptive_kp[0]} (空中) → {args.adaptive_kp[1]} (电流均值 ≥ {args.adapt_ref:.0f} mA)，时间常数 {args.adapt_tau} s")
    else:
        print(f"* 从手 P 增益固定 {kp0}")
    try:
        if not args.dry_run:
            if follower is not None:
                follower.enter_position_mode(args.follower_limit)
                follower.set_position_gains(kp0, args.follower_kd)
                for g in args.gain:
                    name, _, val = g.partition("=")
                    kp_s, _, kd_s = val.partition(",")
                    hit = [j for j in joints if j.startswith(name)]
                    if not hit:
                        print(f"!! --gain {g}: 没有关节名以 {name} 开头")
                    for j in hit:
                        if kp_s:
                            follower.pkt.write2ByteTxRx(follower.port, ids[j], 84, int(kp_s))
                            kp_cmd[j] = kp_written[j] = float(kp_s)
                        if kd_s:
                            follower.pkt.write2ByteTxRx(follower.port, ids[j], 80, int(kd_s))
                    if hit:
                        print(f"* 从手增益 {', '.join(hit)}: P {kp_s or '不变'}，D {kd_s or '不变'}")
                # 从手用 1.5 s 从自己现在的姿态滑到主手当前姿态：两只手套开始时不需要摆成一样，也不需要在零位
                qA0 = leader_angles(leader.read_state())
                raw = follower.read_raw()
                qB0 = {j: sB[j] * ticks_to_rad(raw[ids[j]] - zB[j]) for j in joints}
                print("* 从手正在滑到主手的姿态 ...", flush=True)
                t_s = time.time()
                while True:
                    s_ = min(1.0, (time.time() - t_s) / 1.5)
                    follower.write_positions(follower_goal_ticks({j: qB0[j] + (qA0[j] - qB0[j]) * s_ for j in joints}))
                    if s_ >= 1.0:
                        break
                    time.sleep(0.02)
            leader.enter_current_mode(args.leader_limit)
            leader.check_torque("主手")
            if follower is not None:
                follower.check_torque("从手")
            print("* 扭矩已开。Ctrl+C 退出，退出时两边都关扭矩")
        alerts = []                                    # 最近的保护报警，固定显示在表格下面，不被刷新盖掉
        n_lines = len(joints) + 6
        print("\n" * n_lines, end="")
        period = 1.0 / args.hz
        t0 = time.time(); t_prev, t_report, t_wd, n_loop = 0.0, t0, t0, 0
        t_gain = t0
        vA = leader.voltage()
        vB = follower.voltage() if follower is not None else float("nan")
        q_cmd = None                                   # 低通后的从手目标
        tau_f = {j: 0.0 for j in joints}
        gate = {j: 0.0 for j in joints}                # 连续门控 0–1：从手电流大且几乎不动 = 被挡住
        contact = {j: False for j in joints}
        errors = 0
        log = []
        while True:
            t_loop = time.time()
            try:
                stA = leader.read_state()
                stB = follower.read_state() if follower is not None else None
                errors = 0
            except IOError as e:
                errors += 1
                for b in (leader, follower):                       # SDK 一次收发失败后可能把串口标成"占用"，手动复位
                    if b is not None:
                        b.port.is_using = False
                if errors >= 5:
                    raise IOError(f"连续 5 次读取失败：{e}")
                continue
            qA = leader_angles(stA)
            if stB is not None:
                qB = {j: sB[j] * ticks_to_rad(stB[ids[j]][2] - zB[j]) for j in joints}
                if not args.dry_run:
                    if args.cmd_lpf > 0:
                        dt_c = (t_loop - t_prev) if t_prev else period
                        a_c = dt_c / (dt_c + 1 / (2 * math.pi * args.cmd_lpf))
                        q_cmd = dict(qA) if q_cmd is None else {j: q_cmd[j] + a_c * (qA[j] - q_cmd[j]) for j in joints}
                        follower.write_positions(follower_goal_ticks(q_cmd))
                    else:
                        follower.write_positions(follower_goal_ticks(qA))
            else:
                qB = dict(qA)
            ramp = min(1.0, (t_loop - t0) / 1.5)
            dt = t_loop - t_prev if t_prev else period
            t_prev = t_loop
            alpha = 1.0 if args.lpf <= 0 else dt / (dt + 1 / (2 * math.pi * args.lpf))
            if stB is not None and args.adaptive_kp is not None:
                # 方法 3：K_t = f(τ_t)。接触电流（滑动平均）越大，从手刚度越低；空中回到最高
                kmax, kmin = args.adaptive_kp
                beta = dt / (dt + args.adapt_tau)
                for j in joints:
                    iB_avg[j] += beta * (abs(stB[ids[j]][0]) - iB_avg[j])
                    kp_cmd[j] = kmax - (kmax - kmin) * min(1.0, iB_avg[j] / args.adapt_ref)
                if not args.dry_run and t_loop - t_gain > 0.02:     # 最多 50 Hz 写增益，变化 ≥ 5 才写
                    changed = {ids[j]: kp_cmd[j] for j in joints if abs(kp_cmd[j] - kp_written[j]) >= 5}
                    if changed:
                        follower.write_p_gains(changed)
                        for j in joints:
                            if ids[j] in changed:
                                kp_written[j] = kp_cmd[j]
                    t_gain = t_loop
            err, cur = {}, {}
            for j in joints:
                e = qA[j] - qB[j]
                err[j] = e
                if stB is None or args.contact_ma <= 0:
                    g = 1.0
                else:
                    iB = abs(stB[ids[j]][0]); velB = abs(stB[ids[j]][1])
                    g_i = min(1.0, max(0.0, (iB - 0.5 * args.contact_ma) / (0.5 * args.contact_ma)))        # 电流：半值开始，全值全开
                    g_v = min(1.0, max(0.0, (args.contact_vel - velB) / (0.75 * args.contact_vel)))          # 速度：不动全开，快动关闭
                    g = g_i * g_v
                a_g = dt / (dt + 0.05)                                                                      # 50 ms 低通，渐入渐出不抖
                gate[j] += a_g * (g - gate[j])
                contact[j] = gate[j] > 0.5
                t = -args.kp * (abs(e) - db) * math.copysign(1, e) * gate[j] if abs(e) > db else 0.0
                if stB is not None and args.kc > 0:
                    # 从手电流（带符号，从手坐标）-> 从手力矩 -> 主手反向；也经过接触门控，空中跟踪的电流不反射
                    tauB = sB[j] * stB[ids[j]][0] / 1000 * KT[fmodels[j]]
                    th = 0.03
                    if abs(tauB) > th:
                        t -= args.kc * (abs(tauB) - th) * math.copysign(1, tauB) * gate[j]
                t = float(np.clip(t * ramp, -args.max_torque, args.max_torque))
                tau_f[j] += alpha * (t - tau_f[j])
                ff = 0.0
                if args.friction_ma > 0:                                   # 摩擦补偿：顺着主手运动方向；|v| < 0.15 rad/s 不补，免得静止时抖
                    velA = sA[j] * stA[ids[j]][1]
                    v0 = 0.15
                    ff = args.friction_ma * math.tanh((abs(velA) - v0) / 0.3) * math.copysign(1, velA) * ramp if abs(velA) > v0 else 0.0
                cur[ids[j]] = float(np.clip(sA[j] * (tau_f[j] / KT[models[j]] * 1000 + ff), -args.leader_limit, args.leader_limit))
            total = sum(abs(v) for v in cur.values())
            if total > args.budget:
                cur = {i: v * args.budget / total for i, v in cur.items()}
            if args.log is not None:
                row = [t_loop - t0]
                for j in joints:
                    i = ids[j]
                    row += [qA[j], sA[j] * stA[i][1], stA[i][0], cur[i], qB[j],
                            (sB[j] * stB[i][1]) if stB else 0.0, (stB[i][0] if stB else 0), round(kp_cmd[j])]
                log.append(row)
            if not args.dry_run:
                leader.write_currents(cur)
                if t_loop - t_wd > 1.0:
                    vA = leader.voltage(); vB = follower.voltage() if follower is not None else float("nan")
                    for name, b in (("主手", leader), ("从手", follower)):
                        if b is None:
                            continue
                        if b.check_watchdog():
                            alerts.append(f"{time.strftime('%H:%M:%S')} {name}看门狗触发过，已重新打开")
                        hw = b.hardware_errors()
                        if hw:
                            msg = f"{name}硬件保护: " + "，".join(f"ID {i} {'/'.join(v)}" for i, v in hw.items()) + "（扭矩已被电机关掉）"
                            if not alerts or alerts[-1][9:] != msg:
                                alerts.append(f"{time.strftime('%H:%M:%S')} {msg}")
                        del alerts[:-3]
                    t_wd = t_loop
            n_loop += 1
            now = time.time()
            if now - t_report > 0.25:
                fv = lambda v: "--" if (v != v or v < 1.0) else f"{v:.1f} V"
                lines = [f"控制 {n_loop / (now - t_report):4.0f} Hz | 主手 {fv(vA)} 电流合计 指令 {sum(abs(v) for v in cur.values()):5.0f} mA  实测 {sum(abs(stA[i][0]) for i in cur):5.0f} mA"
                         + (f" | 从手 {fv(vB)} 电流合计 {sum(abs(stB[i][0]) for i in cur):5.0f} mA" if stB else ""),
                         f"{'ID':>3} {'关节':22s} {'主手°':>7s} {'从手°':>7s} {'差°':>6s} {'主手mA':>7s} {'从手mA':>7s} {'从手Kp':>6s} 接触"]
                for j in joints:
                    i = ids[j]
                    lines.append(f"{i:3d} {j:22s} {math.degrees(qA[j]):+7.1f} {math.degrees(qB[j]):+7.1f} {math.degrees(err[j]):+6.1f} {cur[i]:+7.0f} {(stB[i][0] if stB else 0):+7d} {kp_cmd[j]:6.0f}  {'●' if contact[j] else ''}")
                lines += ["!! " + a for a in alerts] + [""] * (3 - len(alerts))
                lines.append("")
                print(f"\033[{n_lines}F" + "\n".join(l + "\033[K" for l in lines), flush=True)
                n_loop, t_report = 0, now
            sleep = period - (time.time() - t_loop)
            if sleep > 0:
                time.sleep(sleep)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"\n!! 出错，已停止: {e}")
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if args.log is not None and log:
            import csv
            os.makedirs(os.path.dirname(os.path.abspath(args.log)), exist_ok=True)
            with open(args.log, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["t"] + [f"{j}:{k}" for j in joints for k in ("qA", "dqA", "iA_meas", "iA_cmd", "qB", "dqB", "iB_meas", "kpB")])
                w.writerow(["# models"] + [f"{j}:{MODEL_NAME[models[j]]}" for j in joints])
                w.writerow(["# settings", f"follower_kp:{kp0}",
                            f"follower_kd:{args.follower_kd if args.follower_kd is not None else 0}", f"follower_limit_mA:{args.follower_limit}",
                            f"leader_kp:{args.kp}", f"deadband_deg:{args.deadband}",
                            f"adaptive_kp:{'%d/%d/%.0f' % (args.adaptive_kp[0], args.adaptive_kp[1], args.adapt_ref) if args.adaptive_kp else 'off'}"])
                w.writerows(log)
            print(f"* 已保存 {len(log)} 行到 {args.log}")
        if not args.dry_run:
            leader.safe_stop()
            if follower is not None:
                follower.safe_stop()
            print("* 两边都已清零电流并关扭矩")
        leader.close()
        if follower is not None:
            follower.close()


if __name__ == "__main__":
    main()
