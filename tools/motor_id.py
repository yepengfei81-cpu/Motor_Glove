#!/usr/bin/env python3
"""
单电机辨识（台架：电机固定在桌上，输出轴装一根杆）。每个实验录一份 CSV 到 data/motor_id/，fit 把它们拟成一组参数
存到 config/motor_params/<型号>_<设备>.json：J（惯量）、b（粘滞）、tc（库仑摩擦，正反向）、ts（静摩擦）、Kt_eff（有效力矩常数）、
电流环延迟、速度信号滞后。

轴朝向：
  --axis vertical    轴竖直、杆在水平面里转，没有重力项（摩擦、惯量实验用这个）
  --axis horizontal  轴水平、杆像钟摆，有重力项 m·g·r·sin(θ)（挂砝码测 Kt 用这个；其他实验也能做，fit 会把重力减掉）

实验（按顺序做，每条命令跑完自动存文件）：
  python tools/motor_id.py T1 spin   --axis vertical           # 电流脉冲把杆甩起来，然后零电流让它自己停：粘滞 + 库仑摩擦（反驱摩擦，就是手感）
  python tools/motor_id.py T1 ramp   --axis vertical           # 电流慢慢爬到杆开始动，正反向各几次：静摩擦
  python tools/motor_id.py T1 chirp  --axis vertical           # 电流 chirp（0.3–20 Hz），记位置：惯量 J、粘滞 b、延迟
  python tools/motor_id.py T1 step                             # 电流阶跃：电机报的速度相对位置差分的滞后、电流环上升时间
  python tools/motor_id.py T1 hold   --axis horizontal --mass 0.3 --arm 0.018 --limit 500
                                     # 轴水平、杆上挂砝码（用线吊着也行，线要竖直）：先扭矩关让它自然下垂记角度，
                                     # 再位置模式保持在相对下垂 ±30/±60/±90°，保持电流 vs 重力矩 -> Kt_eff；两个方向逼近的差 -> 库仑摩擦
                                     # 做完其他实验前把砝码摘掉
  python tools/motor_id.py T1 lift   --axis horizontal --mass 0.3 --arm 0.018   # 同样挂砝码：电流模式给固定电流看杆抬到几度，Kt 的交叉验证
  python tools/motor_id.py T1 sweep  --axis horizontal --mass 0.3 --arm 0.018 --turns 2 --rate 15
                                     # 挂砝码、位置模式 15°/s 连续正转 2 圈再反转 2 圈，全程记保持电流：电流-角度正弦图，两方向之差 = 摩擦
  python tools/motor_id.py T1 fit                              # 用上面所有文件拟合，打印并保存参数
  python tools/motor_id.py T1 spring --k 0.3                   # 手感演示：虚拟弹簧，推杆松手看停在哪（摩擦造成的死区）
  python tools/motor_id.py T1 spring --comp --b 0.002          # 同样的弹簧 + 阻尼 + 摩擦前馈：补摩擦必须配阻尼，否则没东西耗能会振
  python tools/motor_id.py T1 fit --kt 0.146                   # 没做 hold 实验就用手册的 Kt（M077 0.146，M288 0.354 N·m/A）

惯量 J：杆很轻（1 g）时 J 基本就是电机自己反射到输出轴的惯量，由 chirp 拟出来，spin 的 b/J、tc/J 再乘它；
     杆重的话可以 --rod-mass 杆质量 kg --rod-len 杆长 m（均匀杆 J = m·L²/3）或直接 --J 给定。
轴水平也能做 spin/ramp/chirp/step：1 g 的杆重力矩不到 0.1 mN·m，可以忽略；但砝码一定要摘掉。
安全：杆周围留空，电流上限默认 300 mA（M077 堵转 1.47 A，别超过 600）。Ctrl+C 随时停，停时电流清零关扭矩。
"""
import argparse
import csv
import json
import math
import os
import signal
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import devices  # noqa: E402
from dxl_bus import Bus, KT, MODEL_NAME, TICKS_PER_REV, load_calibration, load_registry, ticks_to_rad  # noqa: E402

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
OUT = os.path.join(ROOT, "data", "motor_id")
G = 9.81


# ---------------- 硬件 ----------------
class Motor:
    def __init__(self, dev, limit_ma):
        reg, cal = load_registry(), load_calibration(dev["name"])
        self.joint = next(iter(cal["joints"]))
        self.id = reg[self.joint]
        self.zero = cal["joints"][self.joint]["zero_raw"]
        self.sign = cal["joints"][self.joint]["sign"]
        self.bus = Bus([self.id], port=dev["port"])
        self.model = self.bus.model(self.id)
        self.kt_nominal = KT[self.model]
        self.limit = limit_ma
        print(f"* {dev['desc']}：ID {self.id}，{MODEL_NAME.get(self.model, self.model)}，手册 Kt {self.kt_nominal:.3f} N·m/A，波特率 {self.bus.baud}")
        self.stop = False
        self.mode = "current"                                  # 指令单位：current 时 mA，pwm 时占空比 %
        signal.signal(signal.SIGINT, lambda *_: setattr(self, "stop", True))

    def pwm_mode(self, limit_pct):
        """电压(PWM)模式：指令是占空比 %，limit 是 PWM 上限 %（40% ≈ 6 V 堵转 0.7 A）"""
        self.bus.enter_pwm_mode(limit_pct, watchdog_ms=1000)
        self.bus.check_torque()
        self.mode, self.limit = "pwm", limit_pct

    def drive_mode(self, args):
        if getattr(args, "mode", "current") == "pwm":
            self.pwm_mode(args.pwm_limit)
        else:
            self.current_mode()

    def write_cmd(self, v):
        """按当前模式写指令：电流 mA 或占空比 %"""
        if self.mode == "pwm":
            self.bus.write_pwms({self.id: self.sign * v})
        else:
            self.write_current(v)

    def current_mode(self):
        # 看门狗放宽到 1 s：脚本里的停顿都在 0.5 s 以内；进程挂掉 1 s 内电机也会自己停
        self.bus.enter_current_mode(self.limit, watchdog_ms=1000)
        self.bus.check_torque()

    def position_mode(self, limit_ma, p=800, d=0):
        self.bus.enter_position_mode(limit_ma, watchdog_ms=0)
        self.bus.set_position_gains(p, d)
        self.bus.check_torque()

    def read(self):
        cur, vel, pos = self.bus.read_state()[self.id]
        return self.sign * cur, self.sign * vel, self.sign * ticks_to_rad(pos - self.zero)

    def write_current(self, ma):
        self.bus.write_currents({self.id: self.sign * ma})

    def pause(self, sec):
        """两段实验之间的停顿：每 50 ms 写一次 0 电流喂总线看门狗（100 ms 不写指令电机就会把目标锁死，后面的脉冲全被吞掉），
        停顿完再检查一次看门狗有没有触发过"""
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < sec and not self.stop:
            self.write_cmd(0); time.sleep(0.05)
        tripped = self.bus.check_watchdog()
        if tripped:
            print("  （看门狗触发过，已恢复）")

    def write_pos(self, q):
        self.bus.write_positions({self.id: self.zero + self.sign * q * TICKS_PER_REV / (2 * math.pi)})

    def close(self):
        self.bus.safe_stop(); self.bus.close()


def log_run(m, name, duration, cmd_fn, hz=1000, extra=None):
    """以 hz 跑 duration 秒：每周期 cmd_fn(t) 给指令电流 mA，记录 t, i_cmd, i_meas, vel, pos。返回 numpy 数组"""
    rows = []; t0 = time.perf_counter(); period = 1 / hz; t_next = t0
    while not m.stop:
        t = time.perf_counter() - t0
        if t > duration:
            break
        i_cmd = float(np.clip(cmd_fn(t), -m.limit, m.limit))
        m.write_cmd(i_cmd)
        cur, vel, pos = m.read()
        rows.append((t, i_cmd, cur, vel, pos))
        t_next += period
        while time.perf_counter() < t_next:
            pass
    m.write_cmd(0)
    m.bus.check_watchdog()
    arr = np.array(rows)
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, f"{name}.csv")
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        if extra:
            w.writerow(["# " + json.dumps(extra)])
        w.writerow(["t", "pwm_cmd_pct" if m.mode == "pwm" else "i_cmd_mA", "i_meas_mA", "vel_rad_s", "pos_rad"])
        w.writerows(rows)
    print(f"  存 {os.path.relpath(path)}（{len(rows)} 行，{len(rows) / max(duration, 1e-6):.0f} Hz）")
    return arr


def _pos_vel(arr, fc=80.0):
    """位置差分速度（零相位双向一阶低通 fc Hz）。电机报的速度滞后 22 ms，看几毫秒的事只能用位置"""
    t, p = arr[:, 0], arr[:, 4]
    dt = float(np.median(np.diff(t)))
    a = dt / (dt + 1 / (2 * math.pi * fc))
    y = p.copy()
    for i in range(1, len(y)):
        y[i] = y[i - 1] + a * (p[i] - y[i - 1])
    z = y[::-1].copy()
    for i in range(1, len(z)):
        z[i] = z[i - 1] + a * (y[::-1][i] - z[i - 1])
    pf = z[::-1]
    return np.gradient(pf, t), dt


def model_params(dev):
    try:
        return json.load(open(os.path.join(ROOT, "config", "motor_params", f"{dev}.json")))
    except FileNotFoundError:
        return {}


def analyze_pwm_spin(arr, pulse_s, J):
    """脉冲结束后占空比 0：绕组被电压源短路，应当几毫秒就刹住（τ = J·R/Kt²）；要是滑行几百毫秒，说明 PWM=0 是断开而不是短路"""
    t = arr[:, 0]; p = arr[:, 4]
    v, dt = _pos_vel(arr)
    k_end = int(np.searchsorted(t, pulse_s))
    v0 = float(np.mean(v[max(0, k_end - 30):k_end]))                   # 脉冲末 30 ms 的速度
    q_end = p[k_end]
    moved = p[k_end:] - q_end
    k_stop = k_end + int(np.argmax(np.abs(v[k_end:]) < 0.2)) if np.any(np.abs(v[k_end:]) < 0.2) else len(t) - 1
    dist = float(moved[k_stop - k_end]) if k_stop > k_end else 0.0
    tau = abs(dist / v0) if abs(v0) > 0.3 else float("nan")
    print(f"    脉冲末速度 {v0:+.2f} rad/s，占空比归零后再转 {math.degrees(dist):+.1f}°，{(t[k_stop] - t[k_end]) * 1e3:.0f} ms 停下"
          f" → 等效时间常数 τ≈{tau * 1e3:.1f} ms，电磁阻尼 b≈J/τ = {J / tau * 1e3 if tau == tau and tau > 0 else float('nan'):.1f} mN·m·s/rad"
          f"（关扭矩滑行是 ~380 ms；短路绕组理论 J·R/Kt² ≈ {J * 3.3 / 0.18 ** 2 * 1e3:.1f} ms）")


def analyze_pwm_step(arr, seq, duty_pct, V, J, kt):
    """占空比阶跃：命令到位置开始加速的延迟（位置二次外推）、稳态转速 → Ke、起始加速度 → R、速度上升时间常数 → R"""
    t = arr[:, 0]; p = arr[:, 4]; im = arr[:, 2]
    v, dt = _pos_vel(arr)
    fs = 1 / dt
    tick = 2 * math.pi / 4096
    edges = []; acc = 0.0
    for (d, _), (_, val_next) in zip(seq[:-1], seq[1:]):           # 每段结束的时刻和下一段的占空比
        acc += d; edges.append((acc, val_next))
    print(f"    {'阶跃':>6s} {'延迟ms':>7s} {'稳态rad/s':>9s} {'Ke V·s/rad':>10s} {'τ上升ms':>7s} {'R(由τ)Ω':>8s} {'实测电流起点ms':>12s}")
    for (te, newv) in edges:
        k = int(np.searchsorted(t, te)); prev = arr[max(0, k - 2), 1]
        di = newv - prev
        if abs(di) < 1e-6 or k < 40 or k + int(0.35 * fs) > len(t):
            continue
        sgn = 1.0 if di > 0 else -1.0
        pre = slice(k - 30, k); coef = np.polyfit(t[pre], p[pre], 1)
        tt = t[k - 30:k + int(0.3 * fs)] - t[k]
        resid = sgn * (p[k - 30:k + int(0.3 * fs)] - np.polyval(coef, t[k - 30:k + int(0.3 * fs)]))
        idx = np.where((resid > 2.5 * tick) & (tt > 0))[0]
        delay = float("nan"); a0 = float("nan")
        if len(idx):
            i1 = idx[0]; w = slice(i1, min(i1 + int(0.010 * fs), len(tt)))
            a2, a1, c0 = np.polyfit(tt[w], resid[w], 2)
            roots = np.roots([a2, a1, c0]); roots = roots[np.isreal(roots)].real
            roots = roots[(roots > -0.005) & (roots < tt[i1])]
            delay = roots.max() if len(roots) else tt[i1]
            a0 = 2 * a2
        # 稳态速度：段末 120 ms
        k2 = int(np.searchsorted(t, te + 0.4)) - 1
        v_ss = float(np.mean(v[k2 - int(0.12 * fs):k2])) if k2 > k else float("nan")
        dv = v_ss - float(np.mean(v[k - 30:k]))
        ke = abs(di) / 100 * V / abs(dv) if abs(dv) > 0.2 else float("nan")
        # 上升时间常数：速度到 63% 的时刻减延迟
        tau_r = float("nan")
        if dv == dv and abs(dv) > 0.2 and delay == delay:
            seg = sgn * (v[k:k2] - v[k - 1]); hit = np.where(seg > 0.632 * abs(dv))[0]
            if len(hit):
                tau_r = t[k + hit[0]] - t[k] - delay
        r_tau = tau_r * kt ** 2 / J if tau_r == tau_r else float("nan")
        im_idx = np.where(sgn * (im[k:k + 60] - im[k - 1]) > 0.3 * abs(di) / 100 * V / 3.3 * 1000)[0]
        t_im = (t[k + im_idx[0]] - t[k]) * 1e3 if len(im_idx) else float("nan")
        print(f"    {prev:+3.0f}→{newv:+3.0f}% {delay * 1e3:7.1f} {v_ss:9.2f} {ke:10.3f} {tau_r * 1e3:7.1f} {r_tau:8.2f} {t_im:12.1f}")
    print("    （合成数据验证：延迟列偏大约 1.5 ms，Ke 偏大约 10%（摩擦吃掉一点转速）；R = τ·Kt²/J，XL330-M077 堵转 1.5 A@5 V 对应 ≈3.3 Ω；"
          "延迟若 ≤4 ms，电压模式就能把 K 开到 1–2 N·m/rad 还不振）")


# ---------------- 实验 ----------------
def exp_spin(m, args):
    """正反各 3 次：电流脉冲甩起来，然后 0 电流看它自己停。
    --coast：脉冲后直接关扭矩（H 桥断开）让它滑行，和零电流对比：差的那部分就是电流模式下的电磁阻尼（反电动势被电流环抵消不干净）"""
    m.drive_mode(args)
    tag = "coast" if args.coast else "spin"
    if m.mode == "pwm":
        # 电压模式：脉冲用占空比，脉冲后占空比 0（绕组被短路式驱动）→ 看是几毫秒刹住还是滑行几百毫秒
        mp = model_params(args.device); J = args.J or mp.get("J_used", 2.6e-5) * 0.18 / 0.146
        for k, sgn in enumerate([1, -1, 1, -1, 1, -1]):
            pulse = sgn * args.pulse_pct
            print(f"  脉冲 {k + 1}/6：占空比 {pulse:+.0f}% × {args.pulse_s:.2f} s，然后占空比 0 看 {args.decay_s:.1f} s")
            arr = log_run(m, f"{args.device}_pwmspin{k}", args.pulse_s + args.decay_s, lambda t: pulse if t < args.pulse_s else 0.0,
                          extra={"exp": "pwmspin", "axis": args.axis, "pulse_pct": pulse, "pulse_s": args.pulse_s})
            analyze_pwm_spin(arr, args.pulse_s, J)
            m.pause(0.5)
            if m.stop:
                break
        return
    for k, sgn in enumerate([1, -1, 1, -1, 1, -1]):
        pulse = sgn * args.pulse_ma
        print(f"  脉冲 {k + 1}/6：{pulse:+.0f} mA × {args.pulse_s:.2f} s，然后{'关扭矩滑行' if args.coast else '零电流'} {args.decay_s:.1f} s")
        if args.coast:
            # 脉冲段自己记，脉冲结束立刻关扭矩，继续只读
            rows = []; t0 = time.perf_counter(); t_next = t0; off = False
            while not m.stop:
                t = time.perf_counter() - t0
                if t > args.pulse_s + args.decay_s:
                    break
                if t < args.pulse_s:
                    m.write_current(pulse)
                elif not off:
                    m.bus.torque_off(); off = True
                cur, vel, pos = m.read()
                rows.append((t, pulse if t < args.pulse_s else 0.0, cur, vel, pos))
                t_next += 0.001
                while time.perf_counter() < t_next:
                    pass
            os.makedirs(OUT, exist_ok=True)
            path = os.path.join(OUT, f"{args.device}_coast{k}.csv")
            with open(path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["# " + json.dumps({"exp": "coast", "axis": args.axis, "pulse_ma": pulse, "pulse_s": args.pulse_s})])
                w.writerow(["t", "i_cmd_mA", "i_meas_mA", "vel_rad_s", "pos_rad"]); w.writerows(rows)
            print(f"  存 {os.path.relpath(path)}（{len(rows)} 行）")
            m.current_mode()                                  # 下一次脉冲前重新开扭矩
        else:
            log_run(m, f"{args.device}_spin{k}", args.pulse_s + args.decay_s, lambda t: pulse if t < args.pulse_s else 0.0,
                    extra={"exp": "spin", "axis": args.axis, "pulse_ma": pulse, "pulse_s": args.pulse_s})
        m.pause(0.5)
        if m.stop:
            break


def exp_ramp(m, args):
    """电流以 ramp_rate mA/s 慢慢爬，杆转过 3° 就算起动，记下那一刻的电流。正反各 n 次，每次换个起始位置"""
    m.current_mode()
    res = []
    for k in range(2 * args.n):
        sgn = 1 if k % 2 == 0 else -1
        _, _, p0 = m.read()
        t0 = time.perf_counter(); i_break = None; rows = []
        while not m.stop:
            t = time.perf_counter() - t0
            i = sgn * args.ramp_rate * t
            if abs(i) > m.limit:
                break
            m.write_current(i)
            cur, vel, pos = m.read(); rows.append((t, i, cur, vel, pos))
            if abs(pos - p0) > math.radians(3):
                i_break = i; break
            time.sleep(0.002)
        m.pause(0.5)
        # 往回走一点，下次从新位置开始
        print(f"  第 {k + 1} 次（{'正' if sgn > 0 else '反'}向）起动电流 {i_break if i_break is not None else float('nan'):+.0f} mA  起始 {math.degrees(p0):+.0f}°")
        res.append({"dir": sgn, "i_break_ma": i_break, "pos0": p0})
        if m.stop:
            break
    os.makedirs(OUT, exist_ok=True)
    json.dump(res, open(os.path.join(OUT, f"{args.device}_ramp.json"), "w"), indent=1)
    print(f"  存 data/motor_id/{args.device}_ramp.json")


def exp_chirp(m, args):
    """电流 chirp：幅值 amp，频率 f0 -> f1 对数扫，时长 T；叠加一个慢的方波防止杆一直往一边漂"""
    m.current_mode()
    f0, f1, T, A = args.f0, args.f1, args.chirp_s, args.amp_ma
    k = math.log(f1 / f0) / T
    def cmd(t):
        phase = 2 * math.pi * f0 * (math.exp(k * t) - 1) / k
        return A * math.sin(phase)
    print(f"  chirp {f0}–{f1} Hz，{A} mA，{T} s")
    log_run(m, f"{args.device}_chirp", T, cmd, extra={"exp": "chirp", "axis": args.axis, "f0": f0, "f1": f1, "amp_ma": A})


def exp_step(m, args):
    """电流阶跃 0 -> +A -> 0 -> -A -> 0，每段 0.4 s：看电机报的速度相对位置差分滞后多少、电流实测的上升时间"""
    m.drive_mode(args)
    A = args.amp_pct if m.mode == "pwm" else args.amp_ma
    seq = [(0.3, 0), (0.4, A), (0.4, 0), (0.4, -A), (0.4, 0)]
    def cmd(t):
        acc = 0
        for d, v in seq:
            acc += d
            if t < acc:
                return v
        return 0
    if m.mode == "pwm":
        vt = m.bus.fast_voltage_temp(); V = vt[m.id][0] if vt else 6.0
        mp = model_params(args.device); J = args.J or mp.get("J_used", 2.6e-5) * 0.18 / 0.146
        kt = args.kt or mp.get("kt_em_estimate") or mp.get("kt_drive_net") or 0.18
        print(f"  占空比阶跃 ±{A:.0f}%（{V:.1f} V → ±{A / 100 * V:.2f} V，空载约 ±{A / 100 * V / kt:.1f} rad/s），J {J * 1e5:.2f}e-5，Kt {kt:.3f}")
        arr = log_run(m, f"{args.device}_pwmstep", sum(d for d, _ in seq), cmd, extra={"exp": "pwmstep", "amp_pct": A, "V": V})
        analyze_pwm_step(arr, seq, A, V, J, kt)
        return
    log_run(m, f"{args.device}_step", sum(d for d, _ in seq), cmd, extra={"exp": "step", "amp_ma": A})


def exp_hold(m, args):
    """轴水平、杆上挂砝码：位置模式保持在几个角度（从两个方向逼近），记保持电流。重力矩 = m·g·r·sin(θ)"""
    if args.mass is None or args.arm is None:
        sys.exit("hold 需要 --mass（砝码 kg）和 --arm（砝码到轴心 m）")
    m.position_mode(args.limit, 800, 0)
    # 先找自然下垂位置：扭矩关、等砝码稳住，读角度；之后的角度都相对它
    m.bus.torque_off(); time.sleep(1.5)
    _, _, hang = m.read()
    print(f"  砝码自然下垂时的角度 {math.degrees(hang):+.1f}°，下面的角度都相对它（+ 往一边摆，- 往另一边）")
    m.position_mode(args.limit, 800, 0)
    angles = [hang + math.radians(a) for a in args.angles]
    res = [{"theta": hang, "approach": 0, "i_hold_ma": 0.0, "note": "hang"}]
    print("  每个角度从两边逼近各保持 1.5 s")
    def glide(q_to, sec):
        """从现在的位置平滑走到 q_to，不甩砝码"""
        _, _, q_from = m.read()
        n = max(10, int(sec / 0.02))
        for k in range(1, n + 1):
            s_ = 0.5 - 0.5 * math.cos(math.pi * k / n)
            m.write_pos(q_from + (q_to - q_from) * s_); time.sleep(0.02)

    for a in angles:
        for approach in (+1, -1):
            start = a - approach * math.radians(15)
            glide(start, 1.5)                                    # 先慢慢到逼近起点
            glide(a, 1.0)                                        # 再从一侧逼近目标
            time.sleep(args.settle)                              # 等砝码停摆
            cs = []
            for _ in range(100):
                cur, _, pos = m.read(); cs.append(cur); time.sleep(0.01)
            i_hold, i_std = float(np.mean(cs)), float(np.std(cs))
            tau_g = args.mass * G * args.arm * math.sin(pos - hang)          # 相对下垂位置的重力矩
            print(f"  相对下垂 {math.degrees(a - hang):+5.0f}°（{'正向' if approach > 0 else '反向'}逼近）保持电流 {i_hold:+7.1f} mA（离散 ±{i_std:.1f}）"
                  f"，重力矩 {1000 * tau_g:+7.1f} mN·m" + ("  <- 电流还在晃，砝码没停" if i_std > 8 else ""))
            res.append({"theta": pos, "approach": approach, "i_hold_ma": i_hold, "i_std_ma": i_std, "tau_g": tau_g})
            if m.stop:
                break
        if m.stop:
            break
    # 结束：电流上限慢慢放到 0，砝码自己垂下去，不甩
    for cap in np.linspace(args.limit, 0, 50):
        m.bus.write_currents({m.id: int(cap)}); time.sleep(0.03)
    os.makedirs(OUT, exist_ok=True)
    json.dump({"mass": args.mass, "arm": args.arm, "hang": hang, "points": res}, open(os.path.join(OUT, f"{args.device}_hold.json"), "w"), indent=1)
    print(f"  存 data/motor_id/{args.device}_hold.json")


def exp_sweep(m, args):
    """轴水平、挂砝码：位置模式以恒定角速度连续转 turns 圈，再反向转回来，全程记角度和保持电流。
    电流对角度应该是一条正弦（重力矩），正转和反转两条曲线的差就是摩擦，两圈之间的差就是重复性"""
    if args.mass is None or args.arm is None:
        sys.exit("sweep 需要 --mass 和 --arm")
    m.position_mode(args.limit, 800, 0)
    m.bus.torque_off(); time.sleep(1.5)
    _, _, hang = m.read()
    print(f"  砝码自然下垂角 {math.degrees(hang):+.1f}°；以 {args.rate:.0f}°/s 正转 {args.turns} 圈，停 2 s，再反转 {args.turns} 圈回来")
    m.position_mode(args.limit, 800, 0)
    rows = []
    total = math.radians(360 * args.turns)
    w = math.radians(args.rate)
    t0 = time.perf_counter()
    for direction, q_from, q_to in ((+1, hang, hang + total), (-1, hang + total, hang)):
        T = total / w
        ts = time.perf_counter()
        while not m.stop:
            t = time.perf_counter() - ts
            if t > T:
                break
            q = q_from + direction * w * t
            m.write_pos(q)
            cur, _, pos = m.read()
            rows.append((time.perf_counter() - t0, direction, q - hang, pos - hang, cur))
            time.sleep(0.008)
        if m.stop:
            break
        tp = time.perf_counter()
        while time.perf_counter() - tp < 2.0:
            m.write_pos(q_to); cur, _, pos = m.read()
            rows.append((time.perf_counter() - t0, 0, q_to - hang, pos - hang, cur)); time.sleep(0.01)
    for cap in np.linspace(args.limit, 0, 50):
        m.bus.write_currents({m.id: int(cap)}); time.sleep(0.03)
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, f"{args.device}_sweep_{round(args.mass * 1000)}g.csv")
    with open(path, "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["# " + json.dumps({"exp": "sweep", "mass": args.mass, "arm": args.arm, "hang": hang, "rate": args.rate, "turns": args.turns})])
        wr.writerow(["t", "dir", "q_target_rad", "pos_rad", "i_meas_mA"])
        wr.writerows(rows)
    print(f"  存 {os.path.relpath(path)}（{len(rows)} 行）；看结果：python tools/motor_id.py {args.device} fit")


def exp_lift(m, args):
    """轴水平、挂砝码：电流模式给一串固定电流，杆抬到平衡角 θ：Kt·i = m·g·r·sin θ。不依赖位置环和电流读数。
    每个电流从下面抬上去、再从上面降下来各测一次，两次的差来自摩擦"""
    if args.mass is None or args.arm is None:
        sys.exit("lift 需要 --mass 和 --arm")
    m.current_mode()
    m.pause(1.5)                                     # 零电流等砝码垂稳（持续喂看门狗）
    _, _, hang = m.read()
    print(f"  自然下垂角 {math.degrees(hang):+.1f}°")
    mgr = args.mass * G * args.arm
    res = []
    for sgn in (+1, -1):
        levels = [float(x) for x in args.lift_ma]
        for i_ma in levels + levels[::-1][1:]:          # 上去再下来
            i = sgn * i_ma
            for k in range(40):                           # 0.8 s 渐变到位，免得甩过头
                m.write_current(i * (k + 1) / 40 if k < 40 else i); time.sleep(0.02)
            t0 = time.perf_counter()
            while time.perf_counter() - t0 < 1.5:
                m.write_current(i); time.sleep(0.02)
            ps = []
            for _ in range(50):
                m.write_current(i); _, _, pos = m.read(); ps.append(pos); time.sleep(0.01)
            th = float(np.mean(ps)) - hang
            if abs(th) > math.radians(100):
                print(f"  {i:+.0f} mA：杆翻过头了（{math.degrees(th):+.0f}°），电流太大，这一点作废"); continue
            kt = mgr * math.sin(th) / (i / 1000) if abs(i) > 1 else float("nan")
            print(f"  {i:+6.0f} mA -> 平衡角 {math.degrees(th):+6.1f}°  -> Kt = {kt:.4f} N·m/A")
            res.append({"i_ma": i, "theta": th, "kt": kt})
            if m.stop:
                break
        for k in range(40):
            m.write_current(i * (39 - k) / 40); time.sleep(0.02)
        if m.stop:
            break
    os.makedirs(OUT, exist_ok=True)
    json.dump({"mass": args.mass, "arm": args.arm, "hang": hang, "points": res}, open(os.path.join(OUT, f"{args.device}_lift.json"), "w"), indent=1)
    kts = [r["kt"] for r in res if 10 < abs(math.degrees(r["theta"])) < 80]
    if kts:
        print(f"  平衡角在 10–80° 的点：Kt 均值 {np.mean(kts):.4f}，离散 ±{np.std(kts):.4f} N·m/A（手册 {m.kt_nominal:.3f}）")
    print(f"  存 data/motor_id/{args.device}_lift.json")


def exp_spring(m, args):
    """
    手感演示：单电机当虚拟弹簧 τ = −K·θ（电流模式 1 kHz）。用手推杆再松开，看它回不回得到零位。
    --comp 打开摩擦前馈：τ += f̂·tanh(v̂/v0)，f̂ = 空载库仑 + k·F_径向，F_径向 ≈ |τ|/r_hand（手推杆的位置到轴的距离）。
    每次松手后打印停下来的位置：K·θ_停 就是没被补掉的摩擦。
    """
    P = {}
    pp = os.path.join(ROOT, "config", "motor_params", f"{args.device}.json")
    if os.path.exists(pp):
        P = json.load(open(pp))
    kt = P.get("kt_em_estimate", 0.27)
    tc0 = P.get("tc_backdrive", 0.0029) * (kt / 0.146)             # spin 的摩擦是按手册 Kt 算的，换算到电磁 Kt
    k_load = P.get("load_friction_model", {}).get("friction_mNm_per_N_radial", 11.0) / 1000
    K = args.k
    J = P.get("J_chirp", 2.6e-6) * (kt / 0.146)                  # chirp 的 J 是按手册 Kt 拟的，换算到电磁 Kt
    b_crit = 2 * math.sqrt(K * J)
    print(f"* 惯量 J ≈ {J * 1e6:.0f} g·cm²，这个弹簧的临界阻尼 B = 2√(KJ) = {b_crit * 1000:.2f} mN·m·s/rad；你给的 B = {args.b * 1000:.2f}"
          + ("  <- 没有阻尼：摩擦是唯一的阻尼，一旦把摩擦补掉就会振" if args.b == 0 and args.comp else ""))
    print(f"* 弹簧 K = {K:.2f} N·m/rad；Kt_em {kt:.3f}；摩擦前馈 {'开' if args.comp else '关'}"
          + (f"：空载 {tc0 * 1000:.1f} mN·m + {k_load * 1000:.1f} mN·m/N × |τ|/{args.r_hand * 1000:.0f} mm，比例 {args.comp_gain:.0%}" if args.comp else ""))
    m.current_mode()
    m.pause(0.3)
    _, _, q0 = m.read()
    print(f"* 现在的位置当零位（{math.degrees(q0):+.1f}°）。推杆、松手，看停在哪。Ctrl+C 退出")
    tau_max = args.limit / 1000 * kt
    print(f"* 电流上限 {args.limit} mA ≈ {tau_max * 1000:.0f} mN·m，弹簧线性区 ±{math.degrees(tau_max / K):.0f}°。推杆时慢一点、在线性区里来回，感受发涩程度")
    sat_warned = False
    # 速度观测：位置差分 + 低通 40 Hz
    v_f, q_prev, t_prev = 0.0, q0, time.perf_counter()
    rest_t, rest_reported = None, False
    rows = []
    t0 = time.perf_counter()
    while not m.stop:
        t = time.perf_counter()
        dt = max(1e-4, t - t_prev); t_prev = t
        _, _, q = m.read()
        v_raw = (q - q_prev) / dt; q_prev = q
        a_f = dt / (dt + 1 / (2 * math.pi * 40))
        v_f += a_f * (v_raw - v_f)
        th = q - q0
        tau = float(np.clip(-K * th - args.b * v_f, -tau_max, tau_max))   # 弹簧 + 阻尼，先按电流上限截住
        ff = 0.0
        if args.comp:
            f_hat = min(tc0 + k_load * abs(tau) / args.r_hand, 0.4 * tau_max)
            ff = args.comp_gain * f_hat * math.tanh(v_f / args.v0)
        i_cmd = float(np.clip((tau + ff) / kt * 1000, -args.limit, args.limit))
        m.write_current(i_cmd)
        cur_meas, _, _ = m.read() if False else (0, 0, 0)
        rows.append((t - t0, th, v_f, tau, ff, i_cmd))
        if int(t - t0) % 2 == 0 and (t - t0) - int(t - t0) < 0.0015:
            if m.bus.check_watchdog():
                print("  !! 看门狗触发过，已恢复", flush=True)
        saturated = abs(i_cmd) >= args.limit - 1
        if saturated and not sat_warned:
            print(f"  电流到上限了（{args.limit} mA ≈ {tau_max * 1000:.0f} mN·m），弹簧在 ±{math.degrees(tau_max / K):.0f}° 之外就是恒力矩，不是弹簧", flush=True)
            sat_warned = True
        # 松手后停住的位置（只在没饱和、离零位不远时才有意义）
        if abs(v_f) < 0.05 and abs(th) > math.radians(0.5) and not saturated:
            rest_t = rest_t or t
            if t - rest_t > 0.6 and not rest_reported:
                print(f"  停在 {math.degrees(th):+6.2f}°  -> 空载时没补掉的摩擦 ≈ {K * abs(th) * 1000:.1f} mN·m（手已经松开，负载摩擦不在这里体现）", flush=True)
                rest_reported = True
        else:
            rest_t, rest_reported = None, False
        while time.perf_counter() - t < 0.001:
            pass
    m.write_current(0)
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, f"{args.device}_spring_{'comp' if args.comp else 'raw'}.csv")
    with open(path, "w", newline="") as f:
        w = csv.writer(f); w.writerow(["t", "theta_rad", "v_hat", "tau_spring", "ff", "i_cmd_mA"]); w.writerows(rows)
    print(f"  存 {os.path.relpath(path)}")


# ---------------- 拟合 ----------------
def load_csv(path):
    rows = list(csv.reader(open(path)))
    meta = json.loads(rows[0][0][2:]) if rows[0][0].startswith("# ") else {}
    data = np.array([[float(x) for x in r] for r in rows[2 if meta else 1:]])
    return meta, data


def smooth_deriv(t, x, win=15):
    """局部二次多项式（Savitzky–Golay）求一阶和二阶导"""
    n = len(x); half = win // 2
    v = np.zeros(n); a = np.zeros(n)
    for i in range(n):
        lo, hi = max(0, i - half), min(n, i + half + 1)
        tt = t[lo:hi] - t[i]
        if len(tt) < 5:
            continue
        c = np.polyfit(tt, x[lo:hi], 2)
        v[i], a[i] = c[1], 2 * c[0]
    return v, a


def fit(args):
    dev = args.device
    reg, cal = load_registry(), load_calibration(dev)
    joint = next(iter(cal["joints"]))
    out = {"device": dev, "joint": joint, "fitted": time.strftime("%Y-%m-%d %H:%M:%S")}
    # Kt
    kt = args.kt
    hold_p = os.path.join(OUT, f"{dev}_hold.json")
    if os.path.exists(hold_p):
        h = json.load(open(hold_p)); pts = [p for p in h["points"] if p.get("approach")]
        mgr = h["mass"] * G * h["arm"]; hang = h.get("hang", 0.0)
        # 两个方向逼近的平均去掉摩擦，差的一半就是库仑摩擦
        by = {}
        for p in pts:
            by.setdefault(round(math.degrees(p["theta"]) / 5), []).append(p)
        th, cur, fr = [], [], []
        for k, ps in by.items():
            if len(ps) == 2:
                th.append(np.mean([p["theta"] for p in ps])); cur.append(np.mean([p["i_hold_ma"] for p in ps]) / 1000)
                fr.append(abs(ps[0]["i_hold_ma"] - ps[1]["i_hold_ma"]) / 2000)
        if len(th) >= 2:
            th, cur = np.array(th), np.array(cur)
            # 下垂角度可能差几度（摩擦让它停在不是最低点），扫一下 θ0 取残差最小的
            best = None
            for d0 in np.radians(np.arange(-15, 15.1, 0.5)):
                taus = mgr * np.sin(th - hang - d0)
                A = np.vstack([cur, np.ones(len(cur))]).T
                sol, r, *_ = np.linalg.lstsq(A, taus, rcond=None)
                rr = float(np.sum((taus - A @ sol) ** 2))
                if best is None or rr < best[0]:
                    best = (rr, d0, sol)
            _, d0, (k_, c_) = best
            kt_hold = float(abs(k_))
            out["kt_hold"] = kt_hold; out["kt_hold_sign"] = float(np.sign(k_)); out["hold_offset_deg"] = float(math.degrees(d0))
            out["tc_from_hold"] = float(np.mean(fr) * kt_hold)
            print(f"hold：Kt = {kt_hold:.4f} N·m/A（{len(th)} 个角度，下垂角修正 {math.degrees(d0):+.1f}°），电流零偏 {-c_ / k_ * 1000:+.1f} mA，"
                  f"抬升/下放半差 ≈ {np.mean(fr) * 1000:.0f} mA")
            nominal = args.kt or 0.146
            if 0.6 * nominal < kt_hold < 1.6 * nominal and abs(math.degrees(d0)) < 14:
                kt = kt_hold
            else:
                print(f"      !! 这个 Kt 不可信（手册 {nominal:.3f}，或者下垂角修正顶到了边界），不采用：保持电流被负载下的摩擦吃掉了一大半，"
                      f"挂砝码的径向力让输出轴轴承摩擦变大。用 lift 实验交叉验证")
    sweeps = sorted(p_ for p_ in (os.listdir(OUT) if os.path.exists(OUT) else []) if p_.startswith(f"{dev}_sweep") and p_.endswith(".csv"))
    curve = []
    for sweep_name in sweeps:
        sweep_p = os.path.join(OUT, sweep_name)
        meta, d = load_csv(sweep_p)
        mgr = meta["mass"] * G * meta["arm"]
        t, dr, qt, pos, cur = d.T
        print(f"sweep：砝码 {meta['mass'] * 1000:.0f} g × {meta['arm'] * 1000:.0f} mm = {mgr * 1000:.1f} mN·m，{meta['turns']} 圈 × 2 方向")
        # 抬升半圈（电机在推负载）和下放半圈（负载在反驱电机）要分开看：
        #   抬升：按角度分箱，取每箱电流的 90% 分位（粘滑时的"顶住"瞬间）对 重力矩 回归 -> 电机推负载时的净 Kt
        #   下放：电流均值 -> 摩擦替电机扛了多少负载
        res_dir = {}
        for direction in (+1, -1):
            sel = dr == direction
            th = (pos[sel] - 0) % (2 * math.pi); i = cur[sel] / 1000
            tau_g = mgr * np.sin(th)                                  # 正 = 砝码在"正转"方向的那半边
            lifting = (tau_g * direction) > 0.2 * mgr                 # 电机在抬：重力矩方向和运动方向相反
            lowering = (tau_g * direction) < -0.2 * mgr
            bins = np.linspace(0, 2 * math.pi, 37)
            xs, ys = [], []
            for k in range(36):
                b_ = lifting & (th >= bins[k]) & (th < bins[k + 1])
                if b_.sum() > 5:
                    xs.append(abs(mgr * math.sin((bins[k] + bins[k + 1]) / 2))); ys.append(np.percentile(np.abs(i[b_]), 90))
            kt_drive = float(np.dot(xs, ys) / np.dot(ys, ys)) if ys else float("nan")       # τ = Kt · i
            i_low = float(np.mean(np.abs(i[lowering]))) if lowering.sum() else float("nan")
            tau_low = float(np.mean(np.abs(tau_g[lowering]))) if lowering.sum() else float("nan")
            res_dir[direction] = (kt_drive, i_low, tau_low)
            print(f"   {'正转' if direction > 0 else '反转'}：抬升半圈 电流峰值包络 vs 重力矩 -> 推负载时净 Kt ≈ {kt_drive:.3f} N·m/A；"
                  f"下放半圈 平均电流只有 {i_low * 1000:.0f} mA（重力矩平均 {tau_low * 1000:.1f} mN·m，按这个 Kt 本该 {tau_low / kt_drive * 1000:.0f} mA）"
                  f" -> 摩擦扛了 {100 * (1 - i_low * kt_drive / tau_low):.0f}% 的负载")
        if len(res_dir) == 2:
            kt_d = float(np.mean([v[0] for v in res_dir.values()]))
            frac = float(np.mean([1 - v[1] * v[0] / v[2] for v in res_dir.values()]))
            print(f"   结论：电机推负载时 净 Kt ≈ {kt_d:.3f} N·m/A（手册 0.146）；负载反驱电机时摩擦扛掉约 "
                  f"{100 * frac:.0f}% 的负载。径向力 {meta['mass'] * G:.2f} N")
            curve.append({"mass": meta["mass"], "radial_N": meta["mass"] * G, "kt_drive_net": kt_d, "backdrive_friction_fraction": frac,
                          "friction_mNm": float(frac * mgr * 1000 * 2 / math.pi)})       # 下放半圈重力矩平均 = 2/π · mgr
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            plt.rcParams["font.family"] = ["Noto Sans CJK JP", "DejaVu Sans"]; plt.rcParams["axes.unicode_minus"] = False
            fig, ax = plt.subplots(figsize=(9, 5))
            ax.set_facecolor("#fcfcfb"); ax.grid(True, color="#e6e5e1")
            for direction, col, name in ((+1, "#2a78d6", "正转（0–180° 在抬砝码，180–360° 在放）"), (-1, "#eb6834", "反转（180–360° 在抬，0–180° 在放）")):
                sel = dr == direction
                ax.plot(np.degrees(pos[sel]) % 360, cur[sel], ".", ms=2, color=col, label=name)
            th = np.linspace(0, 2 * math.pi, 361)
            ax.plot(np.degrees(th), mgr * np.sin(th) / KT.get(1190, 0.146) * 1000, "--", color="#52514e", lw=1.2, label="手册 Kt 0.146 的预期")
            ax.set_xlabel("相对下垂的角度 °"); ax.set_ylabel("保持电流 mA"); ax.legend(frameon=False)
            ax.set_title(f"{dev} 挂 {meta['mass'] * 1000:.0f} g 连续转 {meta['turns']} 圈：保持电流 vs 角度", loc="left")
            fig.tight_layout(); fig.savefig(os.path.join(OUT, sweep_name.replace(".csv", ".png")), dpi=150, facecolor="#fcfcfb")
            print(f"   图 data/motor_id/{sweep_name.replace('.csv', '.png')}")
        except Exception as e:  # noqa: BLE001
            print(f"   （画图失败：{e}）")
    if curve:
        out["sweeps"] = curve
        out["kt_drive_net"] = float(np.mean([c["kt_drive_net"] for c in curve]))
        print("sweep 汇总（负载摩擦曲线）：")
        for c in sorted(curve, key=lambda c: c["mass"]):
            print(f"   {c['mass'] * 1000:5.0f} g  径向 {c['radial_N']:.2f} N  净 Kt {c['kt_drive_net']:.3f}  反驱时摩擦扛 {100 * c['backdrive_friction_fraction']:.0f}%  ≈ {c['friction_mNm']:.1f} mN·m")
    lift_p = os.path.join(OUT, f"{dev}_lift.json")
    if os.path.exists(lift_p):
        L = json.load(open(lift_p))
        kts = [r["kt"] for r in L["points"] if 10 < abs(math.degrees(r["theta"])) < 80]
        if kts:
            out["kt_lift"] = float(np.mean(kts)); out["kt_lift_std"] = float(np.std(kts))
            print(f"lift：Kt = {np.mean(kts):.4f} ± {np.std(kts):.4f} N·m/A（{len(kts)} 个点）")
            if kt is None:
                kt = float(np.mean(kts))
    if kt is None:
        m_model = None
        try:
            m_model = json.load(open(os.path.join(ROOT, "config", "motors.json"))).get("_models", {}).get(joint)
        except Exception:
            pass
        kt = KT.get(m_model, 0.146)
        print(f"没有 hold 实验也没给 --kt，用 {kt:.3f} N·m/A（手册值）")
    out["kt_used"] = kt
    # J
    J = args.J
    if J is None and args.rod_mass is not None and args.rod_len is not None:
        J = args.rod_mass * args.rod_len ** 2 / 3
        print(f"杆惯量 J = m·L²/3 = {J * 1e6:.0f} g·cm²")
    # ramp：静摩擦
    ramp_p = os.path.join(OUT, f"{dev}_ramp.json")
    if os.path.exists(ramp_p):
        r = json.load(open(ramp_p))
        ip = [x["i_break_ma"] for x in r if x["dir"] > 0 and x["i_break_ma"]]; im = [-x["i_break_ma"] for x in r if x["dir"] < 0 and x["i_break_ma"]]
        if ip and im:
            out["ts_pos"] = float(np.mean(ip) / 1000 * kt); out["ts_neg"] = float(np.mean(im) / 1000 * kt)
            print(f"ramp：静摩擦 正向 {out['ts_pos'] * 1000:.1f} mN·m（{np.mean(ip):.0f} mA，离散 ±{np.std(ip):.0f}），反向 {out['ts_neg'] * 1000:.1f} mN·m")
    # spin：零电流衰减  J·dv/dt = -b·v - tc·sign(v)
    spins = sorted(p for p in os.listdir(OUT) if p.startswith(f"{dev}_spin") and p.endswith(".csv")) if os.path.exists(OUT) else []
    if spins:
        X, Y = [], []
        for p in spins:
            meta, d = load_csv(os.path.join(OUT, p))
            t, pos = d[:, 0], d[:, 4]
            v, a = smooth_deriv(t, pos, 21)
            sel = (t > meta["pulse_s"] + 0.05) & (np.abs(v) > 0.3)          # 零电流段、还在转
            X.append(np.vstack([v[sel], np.sign(v[sel])]).T); Y.append(a[sel])
        X = np.vstack(X); Y = np.concatenate(Y)
        (bJ, tcJ), *_ = np.linalg.lstsq(X, -Y, rcond=None)      # a = -(b/J) v - (tc/J) sign(v)
        out["b_over_J"] = float(bJ); out["tc_over_J"] = float(tcJ)
        print(f"spin：b/J = {bJ:.2f} 1/s，tc/J = {tcJ:.1f} rad/s²（{len(Y)} 个点）")
        out["_spin"] = (float(bJ), float(tcJ))
    # chirp：Kt·i = J·a + b·v + tc·sign(v)  -> 解 J, b, tc（电流用实测值）
    chirp_p = os.path.join(OUT, f"{dev}_chirp.csv")
    if os.path.exists(chirp_p):
        meta, d = load_csv(chirp_p)
        t, i_meas, pos = d[:, 0], d[:, 2] / 1000, d[:, 4]
        v, a = smooth_deriv(t, pos, 15)
        sel = np.abs(v) > 0.2
        A = np.vstack([a[sel], v[sel], np.sign(v[sel])]).T
        (Jc, bc, tcc), *_ = np.linalg.lstsq(A, kt * i_meas[sel], rcond=None)
        resid = kt * i_meas[sel] - A @ np.array([Jc, bc, tcc])
        out["J_chirp"] = float(Jc); out["b_chirp"] = float(bc); out["tc_chirp"] = float(tcc)
        out["chirp_resid_mNm"] = float(np.std(resid) * 1000)
        print(f"chirp：J = {Jc * 1e6:.0f} g·cm²，b = {bc * 1e3:.3f} mN·m·s/rad，tc = {tcc * 1e3:.1f} mN·m，残差 {np.std(resid) * 1000:.1f} mN·m"
              + (f"（杆几何算的 J = {J * 1e6:.0f}）" if J else ""))
        # 延迟：指令电流 -> 实测电流 的互相关
        i_cmd = d[:, 1] / 1000
        best, bl = -1e9, 0
        dtm = np.median(np.diff(t))
        for L in range(0, int(0.03 / dtm)):
            c = np.dot(i_cmd[: len(i_cmd) - L] - i_cmd.mean(), i_meas[L:] - i_meas.mean())
            if c > best:
                best, bl = c, L
        out["current_lag_ms"] = float(bl * dtm * 1000)
        print(f"      电流环：实测电流滞后指令 {bl * dtm * 1000:.1f} ms")
        if J is None:
            J = float(Jc)
    coasts = sorted(p_ for p_ in os.listdir(OUT) if p_.startswith(f"{dev}_coast") and p_.endswith(".csv")) if os.path.exists(OUT) else []
    if coasts:
        X, Y = [], []
        for p_ in coasts:
            meta, d = load_csv(os.path.join(OUT, p_))
            t, pos = d[:, 0], d[:, 4]
            v, a = smooth_deriv(t, pos, 21)
            sel = (t > meta["pulse_s"] + 0.05) & (np.abs(v) > 0.3)
            X.append(np.vstack([v[sel], np.sign(v[sel])]).T); Y.append(a[sel])
        X = np.vstack(X); Y = np.concatenate(Y)
        (bJc, tcJc), *_ = np.linalg.lstsq(X, -Y, rcond=None)
        out["coast_b_over_J"] = float(bJc); out["coast_tc_over_J"] = float(tcJc)
        print(f"coast（关扭矩滑行）：b/J = {bJc:.2f} 1/s，tc/J = {tcJc:.1f} rad/s²（{len(Y)} 个点）")
        if "_spin" in out:
            diff = out['_spin'][0] - bJc
            if diff > 0.3:
                verdict = "电流模式比关扭矩多这么多粘滞，是电流环抵消不掉的反电动势，可以按速度前馈补"
            elif diff < -0.3:
                verdict = "关扭矩反而更粘：XL330 关扭矩时绕组是短接刹车，不是断开。两者在手的速度（1–3 rad/s）下都只有零点几 mN·m，粘手不是电磁阻尼"
            else:
                verdict = "两者一样，电磁阻尼可以忽略"
            print(f"      对比零电流：b/J {out['_spin'][0]:.2f} vs 关扭矩 {bJc:.2f}（差 {diff:+.2f} 1/s）-> {verdict}")
    if "_spin" in out:
        bJ, tcJ = out.pop("_spin")
        if J:
            out["J_used"] = float(J); out["b_spin"] = float(bJ * J); out["tc_backdrive"] = float(tcJ * J)
            print(f"spin + J({J * 1e6:.0f} g·cm²，{'给定' if args.J or args.rod_mass else '来自 chirp'}）-> 粘滞 b = {bJ * J * 1e3:.3f} mN·m·s/rad，反驱库仑摩擦 tc = {tcJ * J * 1e3:.1f} mN·m")
    # step：速度信号滞后
    step_p = os.path.join(OUT, f"{dev}_step.csv")
    if os.path.exists(step_p):
        meta, d = load_csv(step_p)
        t, vel_rep, pos = d[:, 0], d[:, 3], d[:, 4]
        v, _ = smooth_deriv(t, pos, 9)
        dtm = np.median(np.diff(t)); best, bl = -1e9, 0
        for L in range(0, int(0.1 / dtm)):
            c = np.dot(v[: len(v) - L] - v.mean(), vel_rep[L:] - vel_rep.mean())
            if c > best:
                best, bl = c, L
        out["velocity_lag_ms"] = float(bl * dtm * 1000)
        print(f"step：电机报的速度比位置差分滞后 {bl * dtm * 1000:.1f} ms；速度量化 {0.229 * 2 * math.pi / 60:.3f} rad/s")
    os.makedirs(os.path.join(ROOT, "config", "motor_params"), exist_ok=True)
    path = os.path.join(ROOT, "config", "motor_params", f"{dev}.json")
    json.dump(out, open(path, "w"), indent=1, ensure_ascii=False)
    print(f"* 已存 {os.path.relpath(path)}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("device")
    ap.add_argument("exp", choices=["spin", "ramp", "chirp", "step", "hold", "lift", "sweep", "spring", "fit"])
    ap.add_argument("--coast", action="store_true", help="spin 时脉冲后关扭矩滑行（测电磁阻尼用）")
    ap.add_argument("--mode", choices=["current", "pwm"], default="current",
                    help="spin/step 用哪种模式：current 电流模式（默认）；pwm 电压模式，指令是占空比 %%，看电磁阻尼和命令到力矩的延迟")
    ap.add_argument("--pwm-limit", type=float, default=40, help="pwm 模式的 PWM 上限 %%（默认 40：6 V 堵转约 0.7 A）")
    ap.add_argument("--pulse-pct", type=float, default=20, help="pwm 模式 spin 的脉冲占空比 %%（默认 20：空载约 6.7 rad/s）")
    ap.add_argument("--amp-pct", type=float, default=15, help="pwm 模式 step 的阶跃占空比 %%（默认 15：空载约 5 rad/s）")
    ap.add_argument("--k", type=float, default=0.1, help="spring 的刚度 N·m/rad（默认 0.1：300 mA 上限对应 ±46° 线性区）")
    ap.add_argument("--comp", action="store_true", help="spring 时打开摩擦前馈")
    ap.add_argument("--comp-gain", type=float, default=0.7, help="摩擦前馈的比例（默认 0.7；1.0 容易在反向点抖）")
    ap.add_argument("--r-hand", type=float, default=0.025, help="你推杆的位置到轴的距离 m（默认 25 mm），用来把弹簧力矩换成径向力")
    ap.add_argument("--v0", type=float, default=0.5, help="摩擦前馈的速度过渡宽度 rad/s（默认 0.5）")
    ap.add_argument("--b", type=float, default=0.0, help="spring 的阻尼 N·m·s/rad（默认 0；开 --comp 时必须给，建议临界阻尼附近，程序会打印）")
    ap.add_argument("--turns", type=int, default=2, help="sweep 转几圈（默认 2）"); ap.add_argument("--rate", type=float, default=15, help="sweep 角速度 °/s（默认 15）")
    ap.add_argument("--lift-ma", nargs="*", default=["60", "100", "140", "180"], help="lift 实验的电流台阶 mA（别让杆翻过 90°）")
    ap.add_argument("--axis", choices=["vertical", "horizontal"], default="vertical")
    ap.add_argument("--limit", type=int, default=300, help="电流上限 mA（默认 300）")
    ap.add_argument("--pulse-ma", type=float, default=120); ap.add_argument("--pulse-s", type=float, default=0.25); ap.add_argument("--decay-s", type=float, default=2.0)
    ap.add_argument("--ramp-rate", type=float, default=40, help="ramp 的电流爬升速度 mA/s"); ap.add_argument("--n", type=int, default=4)
    ap.add_argument("--f0", type=float, default=0.3); ap.add_argument("--f1", type=float, default=20); ap.add_argument("--chirp-s", type=float, default=30)
    ap.add_argument("--amp-ma", type=float, default=120)
    ap.add_argument("--mass", type=float, default=None); ap.add_argument("--arm", type=float, default=None)
    ap.add_argument("--angles", type=float, nargs="*", default=[-90, -60, -30, 30, 60, 90], help="hold 的角度，相对砝码自然下垂位置（度）")
    ap.add_argument("--settle", type=float, default=3.0, help="hold 每个点到位后等几秒让砝码停摆（默认 3）")
    ap.add_argument("--kt", type=float, default=None); ap.add_argument("--J", type=float, default=None)
    ap.add_argument("--rod-mass", type=float, default=None); ap.add_argument("--rod-len", type=float, default=None)
    args = ap.parse_args()
    if args.exp == "fit":
        fit(args); return
    if args.mode == "pwm" and args.exp not in ("spin", "step"):
        sys.exit("--mode pwm 目前只支持 spin 和 step")
    dev = devices.get(args.device)
    m = Motor(dev, args.limit)
    try:
        {"spin": exp_spin, "ramp": exp_ramp, "chirp": exp_chirp, "step": exp_step, "hold": exp_hold, "lift": exp_lift, "sweep": exp_sweep,
         "spring": exp_spring}[args.exp](m, args)
    finally:
        m.close()
        print("* 电流清零、扭矩关")


if __name__ == "__main__":
    main()
