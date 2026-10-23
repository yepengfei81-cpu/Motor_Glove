#!/usr/bin/env python3
"""
把真手套的关节角实时同步到 MuJoCo 里的手（电机扭矩关闭，只读位置）。用来检查标定对不对，也测一下通信频率。

  python tools/glove_live.py              # 打开 MuJoCo 窗口：手掌立起来朝前，没有地面和方块（--cam side 从侧面看）
  python tools/glove_live.py --tips       # 画出你的指腹球（标定结果），实时打印拇指到各指的间隙，真捏上时应 ≈ 0
  python tools/glove_live.py --show hand --map config/map_AB.json --scale thumb_mcp_flex_motor=1.3
                                          # 手套 A 驱动仿真里的机械手：预览遥操作映射的效果，不碰真机
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
import sim_scenes  # noqa: E402
from dxl_bus import Bus, load_registry, load_calibration, joint_angles, present_ids  # noqa: E402


def load_tips(glove):
    import json
    path = os.path.join(gm.ROOT, "config", f"tips_{glove}.json")
    if not os.path.exists(path):
        sys.exit(f"没有 {os.path.relpath(path)}，先 python tools/mg.py {glove} tips solve")
    t = json.load(open(path))
    return {"center": {f: t["fingers"][f]["center_m"] for f in gm.FINGERS},
            "radius": {f: t["fingers"][f].get("radius_mm", t["radius_mm"]) / 1000 for f in gm.FINGERS},
            "joint_scale": t.get("joint_scale", {}), "n_samples": t.get("n_samples", 0), "rms_mm": t.get("rms_mm", float("nan"))}


def tip_centers(m, d, tips):
    import numpy as np
    out = {}
    for f in gm.FINGERS:
        b = gm.tip_body(m, f)
        out[f] = d.xmat[b].reshape(3, 3) @ np.asarray(tips["center"][f]) + d.xpos[b]
    return out


def tip_gaps(m, d, tips):
    import numpy as np
    c = tip_centers(m, d, tips)
    return {g: (float(np.linalg.norm(c["thumb"] - c[g])) - tips["radius"]["thumb"] - tips["radius"][g]) * 1000 for g in ("index", "mid", "ring")}


def draw_tips(viewer, m, d, tips):
    import numpy as np
    scn = viewer.user_scn
    scn.ngeom = 0
    for f, p in tip_centers(m, d, tips).items():
        g = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE, np.array([tips["radius"][f], 0, 0]), p, np.eye(3).flatten(),
                            np.array([1, 0.1, 0.1, 0.7] if f == "thumb" else [0.1, 0.3, 1, 0.7], dtype=np.float32))
        scn.ngeom += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("fingers", nargs="*", help="只同步这几根手指（thumb index mid ring），默认全部")
    ap.add_argument("--print", action="store_true", help="不开窗口，只打印")
    ap.add_argument("--port", default=None)
    ap.add_argument("--glove", default="A")
    ap.add_argument("--model", choices=list(gm.MODELS), default="glove", help="这台设备是什么：glove / hand（决定标定怎么解释）")
    ap.add_argument("--show", choices=list(gm.MODELS), default=None,
                    help="仿真里显示哪个模型（默认和 --model 一样）。手套驱动、显示 hand = 预览遥操作时机械手会怎么动，自动套手套->机械手映射")
    ap.add_argument("--map", default=None, help="先套 fit_map.py 的映射（如 config/map_AB.json），和 bilateral.py 一样")
    ap.add_argument("--scale", action="append", default=[], metavar="NAME=K", help="某个关节角先乘 K（如 thumb_mcp_flex_motor=1.3），和 bilateral.py 一样")
    ap.add_argument("--id-check", action="store_true", help="不用标定：启动时的姿态当零位、方向都取 +1，只看是哪个关节在动")
    ap.add_argument("--tips", action="store_true",
                    help="画出 config/tips_<手套>.json 里解出的指腹球（红），并实时打印拇指到各指的间隙 mm；文件里的 joint_scale 也会应用")
    ap.add_argument("--cam", choices=list(sim_scenes.CAMS), default="back",
                    help="视角：back 看手背（举手时自己看到的）/ palm 看手心 / side 从拇指侧看 / top 从指尖俯视")
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
    if missing:
        print("  1 Mbps 下没应答的电机多半是出厂状态（ID 1 @ 57600）或线没通：先用 mg.py <设备> show 扫全部波特率，"
              "出厂的用 set 一个一个设好 ID 再来核对")
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
    show = args.show or args.model
    jmap = None
    if args.map or args.scale or show != args.model:
        from retarget import JointMap
        if show == "glove" and args.model == "hand":
            sys.exit("机械手驱动手套模型没有映射文件，只支持 手套 -> hand")
        jmap = JointMap(names, scale=args.scale, map_path=args.map, to_hand=(show == "hand" and args.model == "glove"))
        print(f"* 显示 {show} 模型，关节角经过映射链（和 bilateral.py 从手目标一样）")
    m, cam = sim_scenes.view_scene(show, cam=args.cam)            # 手掌立起来朝前，没有地面方块，不会碰到东西
    d = mujoco.MjData(m)
    act = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i): i for i in range(m.nu)}
    viewer = None if args.print else mujoco.viewer.launch_passive(m, d)
    if viewer is not None:
        sim_scenes.apply_cam(viewer, cam)
    tips = None
    if args.tips and show != "glove":
        print("!! --tips 只在显示手套模型时有意义，忽略")
    elif args.tips:
        tips = load_tips(args.glove)
        if tips["joint_scale"]:
            print("* 应用经验比例: " + "  ".join(f"{j} ×{v:.2f}" for j, v in tips["joint_scale"].items()))
        print(f"* 指腹球来自 config/tips_{args.glove}.json（{tips['n_samples']} 个样本，拟合残差 {tips['rms_mm']:.1f} mm）")
    n_read, t_report, t_sim, t_draw = 0, time.time(), time.time(), 0.0
    try:
        while viewer is None or viewer.is_running():
            q = joint_angles(bus.read_raw(), cal, reg, axes)
            n_read += 1
            if jmap is not None:
                q = jmap(q)
            for name, v in q.items():
                d.ctrl[act[name]] = v * (tips["joint_scale"].get(name, 1.0) if tips else 1.0)
            now = time.time()
            steps = 0
            while t_sim < now and steps < 50:          # 仿真按真实时间推进
                mujoco.mj_step(m, d)
                t_sim += m.opt.timestep
                steps += 1
            t_sim = max(t_sim, now - 0.1)
            if viewer is not None and now - t_draw > 1 / 30:   # 画面只刷新 30 Hz，不拖慢读取
                if tips:
                    draw_tips(viewer, m, d, tips)
                viewer.sync()
                t_draw = now
            if now - t_report > (0.25 if tips else 1.0):
                hz = n_read / (now - t_report)
                if tips:
                    gaps = tip_gaps(m, d, tips)
                    print("拇指到 " + "  ".join(f"{g}: {v:+5.1f} mm" for g, v in gaps.items()) + "   （0 = 指腹刚好碰上，负 = 互相压进去）", flush=True)
                else:
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
