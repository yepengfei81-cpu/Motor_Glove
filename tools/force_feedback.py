#!/usr/bin/env python3
"""
全手力反馈：真手套 -> 仿真里的手（虚拟机械手）-> 碰到积木 -> 电流反馈给真手套。

  python tools/force_feedback.py --dry-run         # 先试运行：不开扭矩，只显示将要输出的电流
  python tools/force_feedback.py                   # 全手 13 个电机，抓积木场景
  python tools/force_feedback.py index mid ring    # 只用其中几根手指
  python tools/force_feedback.py --scene block     # 旧场景：手掌朝下，地上立着一个固定的方块

抓积木场景（默认）：
  手掌朝下，手保持原来的高度；4 cm 积木放在地上（默认在拇指和食指中间，零位时不接触）。
  另外弹出一个滑块面板：拖动积木 X / Y / 转角 / 大小 / 摩擦系数、手的高度，勾选抓取辅助、自动抬手。
  拇指和另一根手指同时捏住积木 0.4 s：抓取辅助把积木"粘"在手上（勾了自动抬手就抬起 8 cm）；
  手指张开超过 15°（或者完全离开积木）就放开。抓取辅助是因为这只手的拇指和食指捏 4 cm 物体时
  两边的力不在一条线上，纯靠摩擦一抬就滑；在面板里取消勾选就是纯物理抓取。

力反馈（每个电机关节单独算，在模型的关节空间里）：
  e = q_手套 - q_仿真        仿真手被挡住时 e 变大
  τ = -kp·(|e|-死区)·sign(e) - kd·dq_手套（只在继续往里压时加阻尼）→ 40 Hz 低通 → 封顶
  只有这根手指在仿真里碰到了手以外的东西（积木、地面）才输出；没碰到时（例如关节限位、连杆滞后
  造成的偏差）一律不出力，避免"空中被推"和振荡。墙是单向的：只往"压进去"的反方向推，
  手指往回退时不会被往积木方向拉。
  电流 = 方向 × τ / Kt（按电机型号换算）；13 个电机的电流总和超过 --budget 时等比例缩小。
控制在后台线程里按固定频率跑，窗口和面板在主线程每秒刷新 30 次，互不拖慢。
安全：电流上限写进电机；启动 1.5 s 内逐渐加力；总线看门狗 100 ms；退出或出错时电流清零并关扭矩。
"""
import argparse
import math
import os
import signal
import sys
import threading
import time

import numpy as np
import mujoco
import mujoco.viewer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import glove_model as gm  # noqa: E402
import sim_scenes  # noqa: E402
from dxl_bus import (Bus, KT, MODEL_NAME, STALL_TORQUE, effective_sign, load_calibration, load_registry,  # noqa: E402
                     ticks_to_rad, wrap_ticks)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("fingers", nargs="*", help="thumb / index / mid / ring，默认全部四根手指")
    ap.add_argument("--scene", choices=["grasp", "block"], default="grasp")
    ap.add_argument("--dry-run", action="store_true", help="不开扭矩，只显示将要输出的电流")
    ap.add_argument("--kp", type=float, default=0.4, help="墙的刚度 N·m/rad（默认 0.4）")
    ap.add_argument("--kd", type=float, default=0.004, help="阻尼 N·m·s/rad（默认 0.004）")
    ap.add_argument("--deadband", type=float, default=2.0, help="死区 度（默认 2）")
    ap.add_argument("--max-torque", type=float, default=0.06, help="每个电机输出力矩上限 N·m（默认 0.06）")
    ap.add_argument("--current-limit", type=int, default=500, help="写进电机的电流上限 mA（默认 500）")
    ap.add_argument("--budget", type=float, default=2500, help="13 个电机电流总和上限 mA（默认 2500，OpenRB 最多 3 A）")
    ap.add_argument("--lpf", type=float, default=40, help="输出低通截止频率 Hz（默认 40，0 = 不滤波）")
    ap.add_argument("--hz", type=float, default=300, help="控制频率上限（默认 300）")
    ap.add_argument("--cube", type=float, default=4.0, help="积木边长 cm（默认 4）")
    ap.add_argument("--friction", type=float, default=1.0, help="积木摩擦系数（默认 1.0，面板里也能调）")
    ap.add_argument("--no-assist", action="store_true", help="一开始就关掉抓取辅助（面板里也能切换）")
    ap.add_argument("--no-panel", action="store_true", help="不开滑块面板")
    ap.add_argument("--no-viewer", action="store_true")
    args = ap.parse_args()
    fingers = args.fingers or list(gm.FINGERS)
    bad = [f for f in fingers if f not in gm.FINGERS]
    if bad:
        sys.exit(f"不认识的手指 {bad}，可选: {', '.join(gm.FINGERS)}")

    reg, cal = load_registry(), load_calibration()
    joints = [j for f in fingers for j in gm.FINGERS[f] if j in cal["joints"]]
    if not joints:
        sys.exit(f"{fingers} 还没有标定")
    axes = gm.joint_axes()
    model_dir = gm.directions()
    ids = {j: reg[j] for j in joints}
    sign = {j: effective_sign(cal["joints"][j], j, axes) for j in joints}
    zero = {j: cal["joints"][j]["zero_raw"] for j in joints}
    finger_of = {j: gm.finger_of(j) for j in joints}
    bus = Bus(ids.values())
    bus.torque_off()
    name_of = {i: j for j, i in ids.items()}
    cleared, still = bus.clear_hardware_errors()
    for i, e in cleared.items():
        print(f"* ID {i}（{name_of[i]}）有硬件保护 {'、'.join(e)}，已重启清除" + ("" if i not in still else "，但重启后还在"))
    if still:
        bus.close()
        sys.exit("!! 这些电机重启后仍有硬件错误，先检查（过热就等它凉下来；过载看看是不是被卡住了）: "
                 + ", ".join(f"ID {i} {name_of[i]} {'、'.join(e)}" for i, e in still.items()))
    models = {j: bus.model(ids[j]) for j in joints}
    unknown = [j for j, mdl in models.items() if mdl not in KT]
    if unknown:
        bus.close()
        sys.exit(f"不认识这些电机的型号: { {j: models[j] for j in unknown} }")

    panel = None
    if args.scene == "grasp":
        x0, y0 = sim_scenes.default_cube_xy(args.cube / 100)
        m, info = sim_scenes.grasp_scene(x0, y0, width=args.cube / 100)
        lift = sim_scenes.HandLift(info, assist=not args.no_assist)
        if not args.no_panel:
            import cube_panel
            panel = cube_panel.CubePanel(x0, y0, assist=not args.no_assist, size=args.cube, friction=args.friction)
        sim_scenes.set_cube_friction(m, info, args.friction)
    else:
        m, info, lift = mujoco.MjModel.from_xml_path(gm.SCENE), None, None
    d = mujoco.MjData(m)
    act = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i): i for i in range(m.nu)}
    for name, i in act.items():                       # 虚拟电机力矩上限 = 同型号堵转力矩
        m.actuator_forcelimited[i] = 1
        m.actuator_forcerange[i] = [-STALL_TORQUE[models.get(name, 1200)], STALL_TORQUE[models.get(name, 1200)]]
    qadr = {j: m.jnt_qposadr[m.joint(j).id] for j in joints}
    db = math.radians(args.deadband)
    if lift:
        lift.update(m, d, 0.0)
    mujoco.mj_forward(m, d)

    print(f"\n* {'试运行（不开扭矩）' if args.dry_run else '力反馈'}：{len(joints)} 个电机；kp {args.kp}，kd {args.kd}，死区 {args.deadband}°，"
          f"力矩上限 {args.max_torque} N·m，电流上限 {args.current_limit} mA/个，总和 ≤ {args.budget:.0f} mA，低通 {args.lpf} Hz")
    viewer = None if args.no_viewer else mujoco.viewer.launch_passive(m, d)
    if viewer is not None and info is not None:
        viewer.cam.lookat[:] = d.xpos[info["cube_body"]]
        viewer.cam.distance, viewer.cam.elevation = 0.35, -20

    lock = threading.Lock()
    stop = threading.Event()
    shared = {"rate": 0.0, "err": {j: 0.0 for j in joints}, "cur": {ids[j]: 0.0 for j in joints},
              "meas": {ids[j]: 0 for j in joints}, "touch": {}, "grasped": False, "error": None, "alert": ""}

    def control():
        """后台线程：读手套 -> 推进仿真 -> 算电流 -> 写电机"""
        period = 1.0 / args.hz
        t0 = time.time()
        t_sim, t_prev, t_rate, t_wd, n_loop, errors = t0, 0.0, t0, t0, 0, 0
        tau_f = {j: 0.0 for j in joints}
        gate = {f: 0.0 for f in fingers}
        wall = {j: 0 for j in joints}
        try:
            while not stop.is_set():
                t_loop = time.time()
                try:
                    st = bus.read_state()
                    errors = 0
                except IOError as e:
                    errors += 1
                    if errors >= 5:
                        raise IOError(f"连续 5 次读取失败：{e}")
                    continue
                qg = {j: sign[j] * ticks_to_rad(wrap_ticks(st[ids[j]][2] - zero[j])) for j in joints}
                dqg = {j: sign[j] * st[ids[j]][1] for j in joints}
                with lock:
                    for j in joints:
                        d.ctrl[act[j]] = qg[j]
                    steps, grasped = 0, shared["grasped"]
                    while t_sim < t_loop and steps < 50:
                        if lift:
                            grasped, _ = lift.update(m, d, m.opt.timestep)
                        mujoco.mj_step(m, d)
                        t_sim += m.opt.timestep
                        steps += 1
                    t_sim = max(t_sim, t_loop - 0.1)
                    touch = sim_scenes.finger_contacts(m, d)
                    qs = {j: float(d.qpos[qadr[j]]) for j in joints}

                ramp = min(1.0, (t_loop - t0) / 1.5)
                dt = t_loop - t_prev if t_prev else period
                t_prev = t_loop
                alpha = 1.0 if args.lpf <= 0 else dt / (dt + 1 / (2 * math.pi * args.lpf))
                for f in fingers:                         # 接触门控：碰到了就 1，没碰到 50 ms 内降到 0
                    target = 1.0 if touch.get(f, 0.0) > 0.05 else 0.0
                    gate[f] += (target - gate[f]) * min(1.0, dt / 0.05)
                err, cur = {}, {}
                for j in joints:
                    e = qg[j] - qs[j]
                    err[j] = e
                    # 单向墙：接触开始后第一次超出死区时记下"压进去"的方向，之后只在这个方向出力，
                    # 手指往回退、角度差反号时不出力（不会把你往积木方向拉）；离开接触就清掉
                    if gate[finger_of[j]] < 0.05:
                        wall[j] = 0
                    elif wall[j] == 0 and abs(e) > db:
                        wall[j] = 1 if e > 0 else -1
                    t = 0.0
                    if abs(e) > db and wall[j] != 0 and e * wall[j] > 0:
                        t = -args.kp * (abs(e) - db) * math.copysign(1, e)
                        if dqg[j] * e > 0:
                            t -= args.kd * dqg[j]
                    t = float(np.clip(t * ramp * gate[finger_of[j]], -args.max_torque, args.max_torque))
                    tau_f[j] += alpha * (t - tau_f[j])
                    cur[ids[j]] = float(np.clip(sign[j] * tau_f[j] / KT[models[j]] * 1000, -args.current_limit, args.current_limit))
                total = sum(abs(v) for v in cur.values())
                if total > args.budget:
                    cur = {i: v * args.budget / total for i, v in cur.items()}
                if not args.dry_run:
                    bus.write_currents(cur)
                    if t_loop - t_wd > 1.0:
                        tripped = bus.check_watchdog()
                        if tripped:
                            shared["alert"] = f"总线看门狗触发过（ID {tripped}），已重新打开"
                        hw = bus.hardware_errors()
                        if hw:
                            shared["alert"] = "硬件保护: " + "; ".join(f"ID {i} {name_of[i]} {'、'.join(e)}" for i, e in hw.items()) + \
                                              "（这个电机已停止出力，退出后重新运行会自动重启它）"
                        t_wd = t_loop

                n_loop += 1
                if t_loop - t_rate > 0.5:
                    shared["rate"] = n_loop / (t_loop - t_rate)
                    n_loop, t_rate = 0, t_loop
                shared.update(err=err, cur=cur, meas={i: st[i][0] for i in cur}, touch=touch, grasped=grasped)
                sleep = period - (time.time() - t_loop)
                if sleep > 0:
                    time.sleep(sleep)
        except Exception as e:
            shared["error"] = e
            stop.set()

    worker = threading.Thread(target=control, daemon=True)
    n_lines = len(joints) + 5
    t_print = 0.0
    try:
        if not args.dry_run:
            bus.enter_current_mode(args.current_limit)
            print("* 电机已切到电流模式并开扭矩。关掉窗口或 Ctrl+C 退出，退出时自动关扭矩")
        worker.start()
        print("\n" * n_lines, end="")
        while not stop.is_set() and (viewer is None or viewer.is_running()):
            with lock:
                if panel is not None:
                    if panel.closed:
                        panel = None
                    else:
                        cube_z = (d.xpos[info["cube_body"]][2] - info["half"]) * 100
                        panel.update(f"抓住: {'是' if shared['grasped'] else '否'}   积木离地 {cube_z:+.1f} cm")
                        size = panel.take_cube_size()
                        if size is not None:
                            sim_scenes.set_cube_size(m, d, info, size)
                            panel._moved = True              # 改了大小就按滑块位置重新放一次（落在地上）
                        mv = panel.take_cube_move()
                        if mv is not None:
                            lift.drop(d)
                            sim_scenes.place_cube(m, d, info, *mv)
                        sim_scenes.set_cube_friction(m, info, panel.friction.get())
                        lift.assist, lift.auto, lift.manual = panel.assist.get(), panel.auto.get(), panel.hand_height
                        if not lift.assist and lift.attached:
                            lift.drop(d)
                if viewer is not None:
                    viewer.sync()
                cube_line = (f"抓住 {'是' if shared['grasped'] else '否'}  手抬 {(lift.height + lift.manual)*100:4.1f} cm  "
                             f"积木离地 {(d.xpos[info['cube_body']][2] - info['half'])*100:+5.1f} cm") if lift else ""
            now = time.time()
            if now - t_print > 0.25:
                cur, meas, err, touch = shared["cur"], shared["meas"], shared["err"], shared["touch"]
                lines = [f"控制 {shared['rate']:4.0f} Hz  电流合计: 指令 {sum(abs(v) for v in cur.values()):5.0f} mA  实测 {sum(abs(v) for v in meas.values()):5.0f} mA",
                         cube_line,
                         f"{'ID':>3} {'关节':22s} {'型号':16s} {'差值':>7s} {'指令mA':>7s} {'实测mA':>7s}"]
                for j in joints:
                    i = ids[j]
                    lines.append(f"{i:3d} {j:22s} {MODEL_NAME[models[j]]:16s} {math.degrees(err[j]):+6.1f}° {cur[i]:+7.0f} {meas[i]:+7d}")
                lines.append("接触力: " + ("  ".join(f"{k} {v:.1f}N" for k, v in touch.items()) if touch else "无"))
                lines.append(("!! " + shared["alert"]) if shared["alert"] else "")
                print(f"\033[{n_lines}F" + "\n".join(l + "\033[K" for l in lines), flush=True)
                t_print = now
            time.sleep(1 / 30)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        shared["error"] = e
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)       # 收尾时再按 Ctrl+C 也不打断关扭矩
        stop.set()
        if worker.is_alive():
            worker.join(timeout=1.0)
        if shared["error"] is not None:
            print(f"\n!! 出错，已停止: {shared['error']}")
        if not args.dry_run:
            bus.safe_stop()
            print("* 已清零电流并关扭矩")
        bus.close()
        if panel is not None:
            panel.close()
        if viewer is not None:
            viewer.close()
            time.sleep(0.2)


if __name__ == "__main__":
    main()
