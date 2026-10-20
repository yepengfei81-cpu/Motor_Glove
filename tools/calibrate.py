#!/usr/bin/env python3
"""
标定一根手指的零点和方向（电机扭矩全程关闭，用手掰）。

  python tools/calibrate.py ring                       # 标定无名指全部关节（thumb / index / mid / ring）
  python tools/calibrate.py ring --joint ring_mcp_abd_motor   # 只重新标定其中一个关节
  python tools/calibrate.py thumb --no-viewer          # 不开仿真窗口（只看文字提示）
  python tools/calibrate.py ring --glove B --port /dev/ttyUSB0   # 第二只手套（从手），标定存到 config/calibration_B.json

流程（默认逐个关节，每次只摆一个关节，其他关节放在哪都行，不会互相撞）：
  对每个关节：
  1. 只把这个关节摆到仿真窗口里的零位角度，扶稳回车 -> 记下它的零点
  2. 往握拳方向转 20~40°（侧摆：向拇指一侧 10~20°），窗口里会演示方向，扶住回车 -> 算出电机方向
  3. 放回零位回车 -> 检查回零误差；超过 3° 可以输入 r 重做这个关节
  每个关节做完就立刻存盘，中途退出不会丢掉已经标好的关节。
  python tools/calibrate.py thumb --whole-finger   # 旧流程：整根手指一起摆零位
结果写进 config/calibration.json，之后 glove_live.py / force_feedback.py 用它把电机读数换成模型关节角。
"""
import argparse
import math
import os
import sys
import threading
import time

import mujoco
import mujoco.viewer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import glove_model as gm  # noqa: E402
from dxl_bus import Bus, load_registry, load_calibration, save_calibration, calib_path, wrap_ticks, ticks_to_rad  # noqa: E402

MIN_TICKS = 150      # 弯曲关节至少转 ~13°，才算看得清方向
MIN_TICKS_ABD = 80   # 侧摆关节行程小，至少 ~7°


def deg(ticks):
    return math.degrees(ticks_to_rad(ticks))


class Demo:
    """后台线程：仿真窗口里让当前关节在零位和"握拳/向拇指侧"方向之间来回动，演示要你转的方向"""

    def __init__(self, finger, use_viewer, scene=gm.SCENE):
        self.m = mujoco.MjModel.from_xml_path(scene)
        self.d = mujoco.MjData(self.m)
        self.m.opt.disableflags = int(self.m.opt.disableflags) | int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
        self.act = {mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, i): i for i in range(self.m.nu)}
        self.tip_body = gm.tip_body(self.m, finger)
        self.joint, self.direction, self.running, self.t_start = None, 1, True, time.time()
        self.viewer = mujoco.viewer.launch_passive(self.m, self.d) if use_viewer else None
        if self.viewer is not None:
            mujoco.mj_forward(self.m, self.d)
            self.viewer.cam.lookat[:] = self.d.xpos[self.tip_body]
            self.viewer.cam.distance = 0.35
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()

    def show(self, joint, direction=1):
        """direction: 模型关节角往哪个方向转是"握拳/向拇指侧"（glove_model.directions() 的值）"""
        self.t_start = time.time()                      # 每次切换都从 0 开始平滑起步，目标角度不会突变
        self.joint, self.direction = joint, direction

    def _run(self):
        t_sim, t_draw = time.time(), 0.0
        while self.running and self.viewer.is_running():
            now = time.time()
            self.d.ctrl[:] = 0
            if self.joint is not None:
                amp = math.radians(15 if "_abd_" in self.joint else 30)
                self.d.ctrl[self.act[self.joint]] = self.direction * amp * (1 - math.cos(2 * math.pi * (now - self.t_start) / 2.5)) / 2
            while t_sim < now:
                mujoco.mj_step(self.m, self.d)
                t_sim += self.m.opt.timestep
            if now - t_draw > 1 / 30:
                self.viewer.sync()
                t_draw = now
            time.sleep(0.005)

    def close(self):
        self.running = False
        if self.viewer is not None:
            self.thread.join(timeout=1)
            self.viewer.close()
            time.sleep(0.2)


def ask_sign(j, k, n, bus, i, zero_i, demo, direction):
    """让用户把关节 j 往握拳 / 向拇指侧转，返回标定方向（只看这一个电机）"""
    demo.show(j, direction)
    span = "10~20°" if "_abd_" in j else "20~40°"
    note = "（拇指以仿真窗口里的演示为准）" if j.startswith("thumb") else "（仿真窗口里在演示）"
    while True:
        ans = input(f"  [{k}/{n} · 2/3] 往 {gm.physical_positive(j)} 转 {span}{note}，扶住，按回车"
                    f"\n        （演示方向不对或者真手套转不过去：输入 r 回车，演示反过来）... ").strip().lower()
        if ans == "r":
            direction = -direction
            demo.show(j, direction)
            print("        演示已反向，照窗口里的新方向转")
            continue
        now, _ = bus.read_avg(10)
        d = wrap_ticks(round(now[i] - zero_i))
        if abs(d) < (MIN_TICKS_ABD if "_abd_" in j else MIN_TICKS):
            print(f"        只转了 {deg(d):+.1f}°，太小，再多转一些")
            continue
        sign = (1 if d > 0 else -1) * direction
        print(f"        编码器 {'增大' if d > 0 else '减小'} {abs(deg(d)):.1f}° -> 标定方向 {sign:+d}")
        return sign


def per_joint(args, joints, ids, bus, demo, model_dir, axes):
    """逐个关节标定：每次只摆这一个关节，其他关节放哪都行；每个关节做完立即存盘"""
    n = len(joints)
    for k, j in enumerate(joints, start=1):
        i = ids[j]
        while True:
            demo.show(None)
            print(f"\n== [{k}/{n}] {j}（ID {i}）==")
            input(f"  [{k}/{n} · 1/3] 只把 {j} 摆到仿真窗口里的零位角度（其他关节放哪都行），扶稳，按回车 ...")
            zero, noise = bus.read_avg(20)
            print(f"        零点编码器 {zero[i]:.1f}   抖动 {noise[i]} tick")
            sign = ask_sign(j, k, n, bus, i, zero[i], demo, model_dir[j])
            demo.show(None)
            input(f"  [{k}/{n} · 3/3] 把 {j} 放回零位，扶稳，按回车 ...")
            back, _ = bus.read_avg(20)
            e = deg(wrap_ticks(round(back[i] - zero[i])))
            if abs(e) < 3:
                print(f"        回零误差 {e:+.1f}°")
                break
            ans = input(f"        回零误差 {e:+.1f}°，偏大。输入 r 回车重做这个关节，直接回车接受 ... ").strip().lower()
            if ans != "r":
                break
        cal = load_calibration(args.glove)
        cal["joints"][j] = {"id": i, "zero_raw": round(zero[i]), "sign": sign, "axis_base": axes[j]}
        save_calibration(cal, args.glove)
        print(f"        已存到 {os.path.relpath(calib_path(args.glove))}")
    print(f"\n* {args.finger} 的 {n} 个关节都标好了。" + ("接着运行  python tools/glove_live.py  看仿真里的手是否跟着动" if args.glove == "A"
          else "接着用  python tools/twin_jog.py  和手套 A 逐个关节对照"))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("finger", choices=list(gm.FINGERS))
    ap.add_argument("--joint", action="append", help="只标定这些关节（可重复）")
    ap.add_argument("--no-viewer", action="store_true")
    ap.add_argument("--glove", default="A", help="手套名：A = 戴在手上的（默认），B = 从手")
    ap.add_argument("--port", default=None, help="串口，默认自动（有 U2D2 用 ttyUSB0，否则 ttyACM0）")
    ap.add_argument("--whole-finger", action="store_true", help="旧流程：整根手指一起摆零位，再逐个关节定方向")
    ap.add_argument("--model", choices=list(gm.MODELS), default="glove", help="仿真窗口和方向用哪个模型：glove（手套）/ hand（直驱机械手）")
    args = ap.parse_args()
    joints = [j for j in gm.FINGERS[args.finger] if not args.joint or j in args.joint]
    if not joints:
        sys.exit(f"{args.joint} 不属于 {args.finger}，可选: {gm.FINGERS[args.finger]}")
    reg = load_registry()
    missing = [j for j in joints if j not in reg]
    if missing:
        sys.exit(f"这些关节还没设 ID: {missing}。先运行  python tools/dxl_setup.py set <关节名>")
    ids = {j: reg[j] for j in joints}
    scene = gm.scene_of(args.model)
    model_dir = gm.directions(scene)
    axes = gm.joint_axes(scene)
    bus = Bus(ids.values(), port=args.port)
    bus.torque_off()
    demo = Demo(args.finger, not args.no_viewer, scene)
    print(f"* {args.finger}: " + ", ".join(f"{j}=ID {i}" for j, i in ids.items()) + "，扭矩已关闭，可以用手掰\n")
    if not args.whole_finger:
        try:
            per_joint(args, joints, ids, bus, demo, model_dir, axes)
        finally:
            demo.close()
            bus.close()
        return
    try:
        demo.show(None)
        input(f"[1/3] 仿真窗口里是零位。把 {args.finger} 摆成同样的姿态，扶稳，按回车 ...")
        zero, noise = bus.read_avg(20)
        for j, i in ids.items():
            print(f"      {j:22s} 零点编码器 {zero[i]:8.1f}   抖动 {noise[i]} tick")

        signs = {}
        for k, j in enumerate(joints):
            direction = model_dir[j]
            demo.show(j, direction)
            span = "10~20°" if "_abd_" in j else "20~40°"
            note = "（拇指以仿真窗口里的演示为准）" if j.startswith("thumb") else "（仿真窗口里在演示）"
            while True:
                ans = input(f"\n[2/3 · {k+1}/{len(joints)}] {j}：往 {gm.physical_positive(j)} 转 {span}{note}，扶住，按回车"
                            f"\n      （演示方向不对或者真手套转不过去：输入 r 回车，演示反过来）... ").strip().lower()
                if ans == "r":
                    direction = -direction
                    demo.show(j, direction)
                    print("      演示已反向，照窗口里的新方向转")
                    continue
                now, _ = bus.read_avg(10)
                dlt = {jj: wrap_ticks(round(now[ii] - zero[ii])) for jj, ii in ids.items()}
                if abs(dlt[j]) < (MIN_TICKS_ABD if "_abd_" in j else MIN_TICKS):
                    print(f"      {j} 只转了 {deg(dlt[j]):+.1f}°，太小，再多转一些")
                    continue
                others = {jj: round(deg(v), 1) for jj, v in dlt.items() if jj != j and abs(v) > abs(dlt[j]) / 3}
                if others:
                    print(f"      注意：其他关节也跟着动了 {others}°（连杆联动的话是正常的）")
                # 用户跟着演示转了，演示在模型里是 direction 方向；编码器变化方向 × direction = 标定方向
                signs[j] = (1 if dlt[j] > 0 else -1) * direction
                print(f"      {j}: 编码器 {'增大' if dlt[j] > 0 else '减小'} {abs(deg(dlt[j])):.1f}° -> 标定方向 {signs[j]:+d}")
                break
            demo.show(None)
            input("      把这个关节放回零位，按回车 ...")

        input(f"\n[3/3] 整根 {args.finger} 摆回零位，扶稳，按回车 ...")
        back, _ = bus.read_avg(20)
        for j, i in ids.items():
            e = deg(wrap_ticks(round(back[i] - zero[i])))
            flag = "" if abs(e) < 3 else "  <- 回零误差偏大，零位不好复现，可以考虑做个定位销/夹具"
            print(f"      {j:22s} 回零误差 {e:+.1f}°{flag}")
    finally:
        demo.close()
        bus.close()

    cal = load_calibration(args.glove)
    for j, i in ids.items():
        cal["joints"][j] = {"id": i, "zero_raw": round(zero[i]), "sign": signs[j], "axis_base": axes[j]}
    save_calibration(cal, args.glove)
    print(f"\n* 已保存到 {os.path.relpath(calib_path(args.glove))}。" + ("接着运行  python tools/glove_live.py  看仿真里的手是否跟着动" if args.glove == "A" else ""))


if __name__ == "__main__":
    main()
