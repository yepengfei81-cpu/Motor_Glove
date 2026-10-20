#!/usr/bin/env python3
"""
两只手套同时用位置模式（模式 5）驱动，面板上每个关节一个滑块，同一个角度同时发给两边，用来逐个电机查"到底哪个关节对不上"。

  python tools/twin_jog.py                          # 默认拇指 + 食指，A = /dev/ttyACM0，B = /dev/ttyUSB0
  python tools/twin_jog.py thumb index mid ring     # 全手
  python tools/twin_jog.py --limit 600 --kp 800     # 电流上限 / 位置环 P 增益（两边相同）

启动：两边都切到位置模式，用 2 s 慢慢走到各自标定的零位（按 config/calibration.json 和 calibration_B.json）。
面板每一行（一个关节）：
  滑块            目标角度（°，模型方向：正 = 握拳 / 向拇指侧），同时发给 A 和 B
  B 微调          只加在 B 上的偏移（°）。如果同一个角度下两边手指看起来不一样，调这个直到两边一致
  读数            A 实际角度 / 电流，B 实际角度 / 电流，以及各自离目标差多少
怎么判断问题在哪：
  · 两边"离目标差"都 < 1–2°，但手指看起来不一样  -> 那个关节的零点（标定时摆的姿态）两边不一致，用"B 微调"对齐后点"保存 B 微调"
  · 某一边"离目标差"很大、电流顶到上限          -> 那边被机械卡住 / 连杆变形 / 电机力不够，不是标定问题
  · 两边都到位、看起来也一样，只有闭合时差一截    -> 指尖几何（主手里有人的手指），不是电机问题
按钮：全部回零 / 保存 B 微调到 calibration_B.json / 关扭矩退出（关窗口也会关扭矩）。
"""
import argparse
import math
import os
import sys
import time
import tkinter as tk

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import glove_model as gm  # noqa: E402
from dxl_bus import (Bus, TICKS_PER_REV, effective_sign, load_calibration, load_registry,  # noqa: E402
                     save_calibration, ticks_to_rad)

RATE_DEG_S = 90.0          # 命令角度向滑块目标移动的最大速度，避免一下跳过去


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("fingers", nargs="*", help="thumb / index / mid / ring，默认 thumb index")
    ap.add_argument("--a-port", default="/dev/ttyACM0")
    ap.add_argument("--b-port", default="/dev/ttyUSB0")
    ap.add_argument("--limit", type=int, default=500, help="两边的电流上限 mA（默认 500）")
    ap.add_argument("--kp", type=int, default=600, help="两边位置环 P 增益（默认 600，出厂 400）")
    ap.add_argument("--kd", type=int, default=50, help="两边位置环 D 增益（默认 50）")
    args = ap.parse_args()
    fingers = args.fingers or ["thumb", "index"]
    bad = [f for f in fingers if f not in gm.FINGERS]
    if bad:
        sys.exit(f"不认识的手指 {bad}")

    reg, axes = load_registry(), gm.joint_axes()
    calA, calB = load_calibration("A"), load_calibration("B")
    joints = [j for f in fingers for j in gm.FINGERS[f] if j in calA["joints"] and j in calB["joints"]]
    skipped = [j for f in fingers for j in gm.FINGERS[f] if j not in joints]
    if skipped:
        print(f"* 跳过（有一边没标定）: {skipped}")
    if not joints:
        sys.exit("没有两边都标定过的关节")
    ids = {j: reg[j] for j in joints}

    buses, sign, zero = {}, {}, {}
    for side, port, cal in (("A", args.a_port, calA), ("B", args.b_port, calB)):
        b = Bus(ids.values(), port=port)
        b.torque_off()
        _, still = b.clear_hardware_errors()
        if still:
            print(f"!! {side} 有硬件错误没清掉: {still}")
        raw = b.read_raw()
        sign[side] = {j: effective_sign(cal["joints"][j], j, axes) for j in joints}
        # 零点换算到电机当前所在的那一圈，避免模式 5（多圈）下转一整圈
        zero[side] = {}
        for j in joints:
            z = cal["joints"][j]["zero_raw"]
            zero[side][j] = z + round((raw[ids[j]] - z) / TICKS_PER_REV) * TICKS_PER_REV
        buses[side] = b
    k = TICKS_PER_REV / (2 * math.pi)

    def angles(side, st):
        return {j: sign[side][j] * ticks_to_rad(st[ids[j]][2] - zero[side][j]) for j in joints}

    def goal(side, q):
        return {ids[j]: zero[side][j] + sign[side][j] * q[j] * k for j in joints}

    # 进位置模式（不开看门狗：面板空闲时不会持续发包），两边同样的增益和电流上限
    for side in ("A", "B"):
        buses[side].enter_position_mode(args.limit, watchdog_ms=0)
        buses[side].set_position_gains(args.kp, args.kd)
        buses[side].check_torque(f"手套 {side} ")
    q_start = {s: angles(s, buses[s].read_state()) for s in ("A", "B")}
    print(f"* 两边已进位置模式（电流上限 {args.limit} mA，P {args.kp}，D {args.kd}），正在 2 s 回零 ...")
    t0 = time.time()
    while True:
        a = min(1.0, (time.time() - t0) / 2.0)
        for s in ("A", "B"):
            buses[s].write_positions(goal(s, {j: q_start[s][j] * (1 - a) for j in joints}))
        if a >= 1.0:
            break
        time.sleep(0.02)

    # ---------------- 面板 ----------------
    root = tk.Tk()
    root.title("两只手套同步点动（位置模式）")
    root.geometry("+30+40")
    font = ("Noto Sans CJK TC", 10)
    mono = ("DejaVu Sans Mono", 10)
    target = {j: tk.DoubleVar(value=0.0) for j in joints}          # 滑块目标（°）
    off_b = {j: tk.DoubleVar(value=0.0) for j in joints}           # B 微调（°）
    cmd = {j: 0.0 for j in joints}                                 # 实际发出的命令（°），按 RATE_DEG_S 追目标
    readout = {}
    STEP = 0.2

    def nudge(j, d):
        off_b[j].set(round(max(-20.0, min(20.0, val(off_b[j]) + d)), 1))

    def val(var):
        try:
            return float(var.get())
        except (tk.TclError, ValueError):
            return 0.0
    tk.Label(root, text="开始时两边都在各自标定的零点。看着两只手，调「B 微调」（只动 B）直到同一个关节两边一样，再点保存。"
                        "左边的滑块同时动两边，用来换个角度复查。",
             font=font).grid(row=0, column=0, columnspan=4, sticky="w", padx=8, pady=(8, 4))
    for r, j in enumerate(joints, start=1):
        rng = 45 if "abd" in j else 100
        tk.Label(root, text=f"ID {ids[j]:2d}  {j}", font=font, width=26, anchor="w").grid(row=r, column=0, padx=6)
        tk.Scale(root, from_=-rng, to=rng, resolution=0.5, orient=tk.HORIZONTAL, length=360, variable=target[j],
                 font=font, showvalue=True).grid(row=r, column=1, padx=4)
        fr = tk.Frame(root)
        fr.grid(row=r, column=2, padx=4)
        tk.Button(fr, text="−", width=2, font=font, command=lambda j=j: nudge(j, -STEP)).pack(side=tk.LEFT)
        tk.Scale(fr, from_=-20, to=20, resolution=0.1, orient=tk.HORIZONTAL, length=220, variable=off_b[j],
                 font=font, showvalue=True).pack(side=tk.LEFT)
        tk.Button(fr, text="+", width=2, font=font, command=lambda j=j: nudge(j, STEP)).pack(side=tk.LEFT)
        lab = tk.Label(root, text="", font=mono, anchor="w", width=62)
        lab.grid(row=r, column=3, padx=6, sticky="w")
        readout[j] = lab
    tk.Label(root, text="B 微调 (°)：± 每次 0.2°，或者拖滑块", font=font).grid(row=len(joints) + 1, column=2)
    status = tk.Label(root, text="", font=mono, anchor="w")
    status.grid(row=len(joints) + 2, column=0, columnspan=4, sticky="w", padx=8)

    def zero_all():
        for j in joints:
            target[j].set(0.0)

    def save_offsets():
        changed = []
        for j in joints:
            o = val(off_b[j])
            if abs(o) < 0.05:
                continue
            # B 在命令 q + o 时和 A 在 q 时一致 -> B 的读数要减掉 o -> 零点挪 sign·o 个 tick
            shift = sign["B"][j] * math.radians(o) * k
            calB["joints"][j]["zero_raw"] = float((calB["joints"][j]["zero_raw"] + shift) % TICKS_PER_REV)
            zero["B"][j] += shift
            off_b[j].set(0.0)
            changed.append(f"{j} {o:+.1f}°")
        if changed:
            save_calibration(calB, "B")
            status.config(text="已写入 config/calibration_B.json: " + ", ".join(changed) + "（如果用 --map，map_AB.json 需要重拟合）")
        else:
            status.config(text="B 微调都是 0，没有可保存的")

    closing = {"flag": False}

    def quit_all():
        closing["flag"] = True

    bar = tk.Frame(root)
    bar.grid(row=len(joints) + 3, column=0, columnspan=4, pady=8)
    for text, fn in (("全部回零", zero_all), ("保存 B 微调到 calibration_B.json", save_offsets), ("关扭矩退出", quit_all)):
        tk.Button(bar, text=text, font=font, command=fn).pack(side=tk.LEFT, padx=8)
    root.protocol("WM_DELETE_WINDOW", quit_all)

    state = {"t": time.time(), "t_ui": 0.0, "t_v": 0.0, "volt": {"A": float("nan"), "B": float("nan")}, "err": {}}

    def tick():
        if closing["flag"]:
            root.destroy()
            return
        now = time.time()
        dt = now - state["t"]
        state["t"] = now
        step = RATE_DEG_S * dt
        for j in joints:
            d = val(target[j]) - cmd[j]
            cmd[j] += max(-step, min(step, d))
        qa = {j: math.radians(cmd[j]) for j in joints}
        qb = {j: math.radians(cmd[j] + val(off_b[j])) for j in joints}
        try:
            buses["A"].write_positions(goal("A", qa))
            buses["B"].write_positions(goal("B", qb))
            stA, stB = buses["A"].read_state(), buses["B"].read_state()
        except IOError as e:
            for s in ("A", "B"):
                buses[s].port.is_using = False
            status.config(text=f"通信出错，已跳过一帧: {e}")
            root.after(20, tick)
            return
        if now - state["t_v"] > 1.0:
            for s in ("A", "B"):
                state["volt"][s] = buses[s].voltage()
                state["err"][s] = buses[s].hardware_errors()
            state["t_v"] = now
        if now - state["t_ui"] > 0.1:
            qA, qB = angles("A", stA), angles("B", stB)
            for j in joints:
                i = ids[j]
                ea = math.degrees(qA[j]) - cmd[j]
                eb = math.degrees(qB[j]) - (cmd[j] + val(off_b[j]))
                flag = "  <- 没到位" if max(abs(ea), abs(eb)) > 3 else ""
                readout[j].config(text=f"A {math.degrees(qA[j]):+6.1f}° {stA[i][0]:+5d}mA (差{ea:+5.1f}) | "
                                       f"B {math.degrees(qB[j]):+6.1f}° {stB[i][0]:+5d}mA (差{eb:+5.1f}){flag}")
            errs = "  ".join(f"{s} 硬件保护 {e}" for s, e in state["err"].items() if e)
            if not status.cget("text").startswith("已写入"):
                status.config(text=f"A {state['volt']['A']:.1f} V   B {state['volt']['B']:.1f} V   {errs}")
            state["t_ui"] = now
        root.after(20, tick)

    root.after(20, tick)
    try:
        root.mainloop()
    finally:
        for s in ("A", "B"):
            try:
                buses[s].safe_stop()
                buses[s].close()
            except Exception:
                pass
        print("* 两边都已关扭矩")


if __name__ == "__main__":
    main()
