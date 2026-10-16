#!/usr/bin/env python3
"""
可视化遥操作：手套的 13 个关节实时同步到仿真里的手。手掌立起来朝前（手指朝上），没有方块、没有力反馈，
电机扭矩全程关闭，随便动。

  python tools/teleop_view.py
  python tools/teleop_view.py --palm     # 相机换到手心一侧
"""
import argparse
import math
import os
import sys
import time

import mujoco
import mujoco.viewer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import glove_model as gm  # noqa: E402
import sim_scenes  # noqa: E402
from dxl_bus import Bus, load_registry, load_calibration, joint_angles  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--palm", action="store_true", help="相机放在手心一侧")
    args = ap.parse_args()
    reg, cal = load_registry(), load_calibration()
    names = [j for js in gm.FINGERS.values() for j in js if j in cal["joints"]]
    if not names:
        sys.exit("还没有标定任何关节，先运行 python tools/calibrate.py <手指>")
    axes = gm.joint_axes()
    bus = Bus(reg[n] for n in names)
    bus.torque_off()
    m, cam = sim_scenes.view_scene()
    d = mujoco.MjData(m)
    act = {n: m.actuator(n).id for n in names}
    print(f"* 同步 {len(names)} 个关节，扭矩关闭；关掉窗口或 Ctrl+C 退出")
    viewer = mujoco.viewer.launch_passive(m, d)
    viewer.cam.azimuth = 0.0 if args.palm else cam["azimuth"]
    viewer.cam.elevation, viewer.cam.distance = cam["elevation"], cam["distance"]
    viewer.cam.lookat[:] = cam["lookat"]
    n_read, t_report, t_sim, t_draw = 0, time.time(), time.time(), 0.0
    try:
        while viewer.is_running():
            q = joint_angles(bus.read_raw(), cal, reg, axes)
            n_read += 1
            for name, v in q.items():
                d.ctrl[act[name]] = v
            now = time.time()
            steps = 0
            while t_sim < now and steps < 50:
                mujoco.mj_step(m, d)
                t_sim += m.opt.timestep
                steps += 1
            t_sim = max(t_sim, now - 0.1)
            if now - t_draw > 1 / 30:
                viewer.sync()
                t_draw = now
            if now - t_report > 1.0:
                print(f"\r读取 {n_read / (now - t_report):4.0f} Hz", end="", flush=True)
                n_read, t_report = 0, now
    except KeyboardInterrupt:
        pass
    finally:
        print()
        bus.close()
        viewer.close()
        time.sleep(0.2)


if __name__ == "__main__":
    main()
