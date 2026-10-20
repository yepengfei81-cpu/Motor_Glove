#!/usr/bin/env python3
"""
把真手套的关节角实时同步到 MuJoCo 里的手（电机扭矩关闭，只读位置）。用来检查标定对不对，也测一下通信频率。

  python tools/glove_live.py              # 打开 MuJoCo 窗口（遥操作场景，带方块）
  python tools/glove_live.py --print      # 不开窗口，终端里打印关节角
  python tools/glove_live.py --glove B --port /dev/ttyUSB0 thumb index   # 第二只手套，只同步这两根手指
  python tools/glove_live.py --id-check --port /dev/ttyACM0              # 查 ID 对不对：不用标定，启动时的姿态当零位，
                                                                         # 你动哪个关节，仿真里就该是哪个关节在动（方向可能反，标定后才对）

默认只同步标定文件里已经标定过的关节，其余关节保持零位。
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
from dxl_bus import Bus, load_registry, load_calibration, joint_angles, present_ids  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("fingers", nargs="*", help="只同步这几根手指（thumb index mid ring），默认全部")
    ap.add_argument("--print", action="store_true", help="不开窗口，只打印")
    ap.add_argument("--port", default=None)
    ap.add_argument("--glove", default="A")
    ap.add_argument("--model", choices=list(gm.MODELS), default="glove", help="仿真里显示哪个模型：glove / hand")
    ap.add_argument("--id-check", action="store_true", help="不用标定：启动时的姿态当零位、方向都取 +1，只看是哪个关节在动")
    args = ap.parse_args()
    fingers = args.fingers or list(gm.FINGERS)
    bad = [f for f in fingers if f not in gm.FINGERS]
    if bad:
        sys.exit(f"不认识的手指 {bad}")
    wanted = [j for f in fingers for j in gm.FINGERS[f]]
    reg = load_registry()
    ok = present_ids([reg[n] for n in wanted], port=args.port)
    missing = [n for n in wanted if reg[n] not in ok]
    if missing:
        print(f"* 没应答、不同步（仿真里保持零位）: " + ", ".join(f"{n} (ID {reg[n]})" for n in missing))
    wanted = [n for n in wanted if reg[n] in ok]
    if not wanted:
        sys.exit("一个电机都没应答")
    if args.id_check:
        names = wanted
        bus = Bus([reg[n] for n in names], port=args.port)
        bus.torque_off()
        raw0, _ = bus.read_avg(5)
        cal = {"joints": {n: {"zero_raw": raw0[reg[n]], "sign": 1} for n in names}}
        print(f"* ID 检查模式：现在的姿态当零位，{len(names)} 个关节全部投到仿真里。你动真手套的哪个关节，仿真里就应该是哪个关节在动"
              f"（方向反了没关系，标定会修）；Ctrl+C 退出")
    else:
        cal = load_calibration(args.glove)
        names = [n for n in wanted if n in cal["joints"]]
        if not names:
            sys.exit(f"手套 {args.glove} 还没有标定这些关节，先运行  python tools/calibrate.py <手指> --glove {args.glove}")
        bus = Bus([reg[n] for n in names], port=args.port)
        bus.torque_off()
        print(f"* 手套 {args.glove}：同步 {len(names)} 个关节: {', '.join(names)}；Ctrl+C 退出")
        todo = [f for f in fingers if any(j not in cal["joints"] for j in gm.FINGERS[f])]
        if todo:
            print(f"* 这些手指还没标定，暂时不同步（在仿真里保持零位）: {', '.join(todo)}。"
                  f"标定命令: " + "；".join(f"python tools/calibrate.py {f} --glove {args.glove}" for f in todo))

    axes = gm.joint_axes(gm.scene_of(args.model))
    m = mujoco.MjModel.from_xml_path(gm.scene_of(args.model))
    d = mujoco.MjData(m)
    act = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i): i for i in range(m.nu)}
    viewer = None if args.print else mujoco.viewer.launch_passive(m, d)
    n_read, t_report, t_sim, t_draw = 0, time.time(), time.time(), 0.0
    try:
        while viewer is None or viewer.is_running():
            q = joint_angles(bus.read_raw(), cal, reg, axes)
            n_read += 1
            for name, v in q.items():
                d.ctrl[act[name]] = v
            now = time.time()
            steps = 0
            while t_sim < now and steps < 50:          # 仿真按真实时间推进
                mujoco.mj_step(m, d)
                t_sim += m.opt.timestep
                steps += 1
            t_sim = max(t_sim, now - 0.1)
            if viewer is not None and now - t_draw > 1 / 30:   # 画面只刷新 30 Hz，不拖慢读取
                viewer.sync()
                t_draw = now
            if now - t_report > 1.0:
                hz = n_read / (now - t_report)
                angles = "  ".join(f"{k}={math.degrees(v):+6.1f}°" for k, v in q.items())
                print(f"读取 {hz:5.0f} Hz | {angles}", flush=True)
                n_read, t_report = 0, now
    except KeyboardInterrupt:
        pass
    finally:
        bus.close()
        if viewer is not None:
            viewer.close()
            time.sleep(0.2)


if __name__ == "__main__":
    main()
