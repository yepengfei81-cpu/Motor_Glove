#!/usr/bin/env python3
"""
位置模式点动 + 零点微调面板。一台设备时微调它自己的零点；两台设备时以第一台为准，微调第二台的零点。

  python tools/twin_jog.py --dev A                    # 只接手套 A：回到标定零点，逐个关节微调零点（不用重新判断方向）
  python tools/twin_jog.py --dev H thumb              # 机械手，只看拇指
  python tools/twin_jog.py --dev A --dev B            # 两只手套：同一个角度发给两边，微调 B 让它和 A 一致
  python tools/twin_jog.py --dev A --limit 600 --kp 800
  （设备名、串口、模型见 config/devices.json；也可以用 mg.py：  mg.py A jog  /  mg.py jog A B）

启动：切到位置模式（模式 5），2 s 慢慢走到各自标定的零点。面板每一行（一个关节）：
  滑块        目标角度（°，模型方向），所有设备同时走到这个角度；用来换个角度复查
  零点微调    只加在被微调的那台设备上的偏移（°）；− / + 每次 0.2°，也可以拖滑块
  读数        每台设备的实际角度 / 电流 / 离目标差多少
用法：
  · 一台设备：看着手，把每个关节调到真正的零位姿态（和标定时仿真窗口里的零位一样），点"保存零点微调"
  · 两台设备：同一个角度下把第二台调到和第一台看起来一样，点保存
  保存只改零点（zero_raw），方向不动；偏移会折算进零点，手不会动，微调值归零
判断：
  · "离目标差" < 1–2° 但姿态看起来不对  -> 零点的问题，用微调修
  · "离目标差"很大、电流顶到上限       -> 被机械卡住 / 连杆变形 / 力不够，不是零点问题
"""
import argparse
import math
import os
import sys
import time
import tkinter as tk

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import devices  # noqa: E402
import glove_model as gm  # noqa: E402
from dxl_bus import (Bus, TICKS_PER_REV, calib_path, effective_sign, load_calibration, load_registry,  # noqa: E402
                     save_calibration, ticks_to_rad)

RATE_DEG_S = 90.0          # 命令角度向滑块目标移动的最大速度，避免一下跳过去
STEP = 0.2                 # ± 按钮每次的零点微调量（°）


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("fingers", nargs="*", help="thumb / index / mid / ring，默认全部")
    ap.add_argument("--dev", action="append", default=[], help="设备名（A / B / H），给一个或两个；默认 A B")
    ap.add_argument("--limit", type=int, default=500, help="电流上限 mA（默认 500）")
    ap.add_argument("--kp", type=int, default=600, help="位置环 P 增益（默认 600，出厂 400）")
    ap.add_argument("--kd", type=int, default=50, help="位置环 D 增益（默认 50）")
    args = ap.parse_args()
    names = args.dev or ["A", "B"]
    if len(names) not in (1, 2) or len(set(names)) != len(names):
        sys.exit("--dev 给一个或两个不同的设备")
    devs = {n: devices.get(n) for n in names}
    if len({d["model"] for d in devs.values()}) > 1:
        sys.exit("两台设备的模型不同（手套 / 机械手），关节角不能直接对照；机械手和手套的映射见 config/glove_to_hand.json")
    trim = names[-1]                                     # 被微调零点的设备
    fingers = args.fingers or list(gm.FINGERS)
    bad = [f for f in fingers if f not in gm.FINGERS]
    if bad:
        sys.exit(f"不认识的手指 {bad}")

    reg = load_registry()
    axes = gm.joint_axes(gm.scene_of(devs[trim]["model"]))
    cal = {n: load_calibration(n) for n in names}
    joints = [j for f in fingers for j in gm.FINGERS[f] if all(j in cal[n]["joints"] for n in names)]
    skipped = [j for f in fingers for j in gm.FINGERS[f] if j not in joints]
    if skipped:
        print(f"* 跳过（没标定）: {skipped}")
    if not joints:
        sys.exit("没有标定过的关节")
    ids = {j: reg[j] for j in joints}

    buses, sign, zero = {}, {}, {}
    for n in names:
        b = Bus(ids.values(), port=devs[n]["port"])
        b.torque_off()
        _, still = b.clear_hardware_errors()
        if still:
            print(f"!! {n} 有硬件错误没清掉: {still}")
        raw = b.read_raw()
        sign[n] = {j: effective_sign(cal[n]["joints"][j], j, axes) for j in joints}
        zero[n] = {}
        for j in joints:                                 # 零点换算到电机当前所在的那一圈，避免模式 5（多圈）下转一整圈
            z = cal[n]["joints"][j]["zero_raw"]
            zero[n][j] = z + round((raw[ids[j]] - z) / TICKS_PER_REV) * TICKS_PER_REV
        buses[n] = b
    k = TICKS_PER_REV / (2 * math.pi)

    def angles(n, st):
        return {j: sign[n][j] * ticks_to_rad(st[ids[j]][2] - zero[n][j]) for j in joints}

    def goal(n, q):
        return {ids[j]: zero[n][j] + sign[n][j] * q[j] * k for j in joints}

    for n in names:                                      # 不开看门狗：面板空闲时不会持续发包
        buses[n].enter_position_mode(args.limit, watchdog_ms=0)
        buses[n].set_position_gains(args.kp, args.kd)
        buses[n].check_torque(f"{n} ")
    q_start = {n: angles(n, buses[n].read_state()) for n in names}
    print(f"* {' + '.join(devs[n]['desc'] for n in names)}：位置模式（电流上限 {args.limit} mA，P {args.kp}，D {args.kd}），2 s 回零 ...")
    t0 = time.time()
    while True:
        a = min(1.0, (time.time() - t0) / 2.0)
        for n in names:
            buses[n].write_positions(goal(n, {j: q_start[n][j] * (1 - a) for j in joints}))
        if a >= 1.0:
            break
        time.sleep(0.02)

    # ---------------- 面板 ----------------
    root = tk.Tk()
    root.title(f"零点微调：{trim}" + (f"（以 {names[0]} 为准）" if len(names) == 2 else ""))
    root.geometry("+30+40")
    font = ("Noto Sans CJK TC", 10)
    mono = ("DejaVu Sans Mono", 10)
    target = {j: tk.DoubleVar(value=0.0) for j in joints}          # 滑块目标（°）
    off = {j: tk.DoubleVar(value=0.0) for j in joints}             # 零点微调（°），只加在 trim 上
    cmd = {j: 0.0 for j in joints}                                 # 实际发出的命令（°），按 RATE_DEG_S 追目标
    readout = {}

    def val(var):
        try:
            return float(var.get())
        except (tk.TclError, ValueError):
            return 0.0

    def nudge(j, d):
        off[j].set(round(max(-20.0, min(20.0, val(off[j]) + d)), 1))

    hint = ("开始时在标定零点。看着手，把每个关节用「零点微调」调到真正的零位姿态，再点保存。"
            if len(names) == 1 else
            f"开始时两边都在各自标定的零点。调「零点微调」（只动 {trim}）直到同一个关节两边一样，再点保存。")
    tk.Label(root, text=hint + "左边的滑块让关节转到别的角度复查。", font=font).grid(row=0, column=0, columnspan=4, sticky="w", padx=8, pady=(8, 4))
    for r, j in enumerate(joints, start=1):
        rng = 45 if "abd" in j else 100
        tk.Label(root, text=f"ID {ids[j]:2d}  {j}", font=font, width=26, anchor="w").grid(row=r, column=0, padx=6)
        tk.Scale(root, from_=-rng, to=rng, resolution=0.5, orient=tk.HORIZONTAL, length=320, variable=target[j],
                 font=font, showvalue=True).grid(row=r, column=1, padx=4)
        fr = tk.Frame(root)
        fr.grid(row=r, column=2, padx=4)
        tk.Button(fr, text="−", width=2, font=font, command=lambda j=j: nudge(j, -STEP)).pack(side=tk.LEFT)
        tk.Scale(fr, from_=-20, to=20, resolution=0.1, orient=tk.HORIZONTAL, length=200, variable=off[j],
                 font=font, showvalue=True).pack(side=tk.LEFT)
        tk.Button(fr, text="+", width=2, font=font, command=lambda j=j: nudge(j, STEP)).pack(side=tk.LEFT)
        lab = tk.Label(root, text="", font=mono, anchor="w", width=31 * len(names))
        lab.grid(row=r, column=3, padx=6, sticky="w")
        readout[j] = lab
    tk.Label(root, text=f"{trim} 零点微调 (°)：± 每次 {STEP}°，或者拖滑块", font=font).grid(row=len(joints) + 1, column=2)
    status = tk.Label(root, text="", font=mono, anchor="w")
    status.grid(row=len(joints) + 2, column=0, columnspan=4, sticky="w", padx=8)

    def zero_all():
        for j in joints:
            target[j].set(0.0)

    def save_offsets():
        changed = []
        for j in joints:
            o = val(off[j])
            if abs(o) < 0.05:
                continue
            # 命令 q + o 时姿态才对 -> 读数要减掉 o -> 零点挪 sign·o 个 tick（方向不动）
            shift = sign[trim][j] * math.radians(o) * k
            cal[trim]["joints"][j]["zero_raw"] = float((cal[trim]["joints"][j]["zero_raw"] + shift) % TICKS_PER_REV)
            zero[trim][j] += shift
            off[j].set(0.0)
            changed.append(f"{j} {o:+.1f}°")
        if changed:
            save_calibration(cal[trim], trim)
            status.config(text=f"已写入 {os.path.relpath(calib_path(trim))}: " + ", ".join(changed))
        else:
            status.config(text="零点微调都是 0，没有可保存的")

    closing = {"flag": False}
    bar = tk.Frame(root)
    bar.grid(row=len(joints) + 3, column=0, columnspan=4, pady=8)
    for text, fn in (("全部回零", zero_all), (f"保存零点微调（{trim}）", save_offsets), ("关扭矩退出", lambda: closing.update(flag=True))):
        tk.Button(bar, text=text, font=font, command=fn).pack(side=tk.LEFT, padx=8)
    root.protocol("WM_DELETE_WINDOW", lambda: closing.update(flag=True))

    state = {"t": time.time(), "t_ui": 0.0, "t_v": 0.0, "volt": {n: float("nan") for n in names}, "err": {}}

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
        q_cmd = {n: {j: math.radians(cmd[j] + (val(off[j]) if n == trim else 0.0)) for j in joints} for n in names}
        try:
            st = {}
            for n in names:
                buses[n].write_positions(goal(n, q_cmd[n]))
            for n in names:
                st[n] = buses[n].read_state()
        except IOError as e:
            for n in names:
                buses[n].port.is_using = False
            status.config(text=f"通信出错，已跳过一帧: {e}")
            root.after(20, tick)
            return
        if now - state["t_v"] > 1.0:
            for n in names:
                state["volt"][n] = buses[n].voltage()
                state["err"][n] = buses[n].hardware_errors()
            state["t_v"] = now
        if now - state["t_ui"] > 0.1:
            q = {n: angles(n, st[n]) for n in names}
            for j in joints:
                i = ids[j]
                parts, worst = [], 0.0
                for n in names:
                    e = math.degrees(q[n][j]) - math.degrees(q_cmd[n][j])
                    worst = max(worst, abs(e))
                    parts.append(f"{n} {math.degrees(q[n][j]):+6.1f}° {st[n][i][0]:+5d}mA (差{e:+5.1f})")
                readout[j].config(text=" | ".join(parts) + ("  <- 没到位" if worst > 3 else ""))
            errs = "  ".join(f"{n} 硬件保护 {e}" for n, e in state["err"].items() if e)
            if not status.cget("text").startswith("已写入"):
                status.config(text="   ".join(f"{n} {state['volt'][n]:.1f} V" for n in names) + f"   {errs}")
            state["t_ui"] = now
        root.after(20, tick)

    root.after(20, tick)
    try:
        root.mainloop()
    finally:
        for n in names:
            try:
                buses[n].safe_stop()
                buses[n].close()
            except Exception:
                pass
        print("* 已关扭矩")


if __name__ == "__main__":
    main()
