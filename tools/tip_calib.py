#!/usr/bin/env python3
"""
自碰触标定：用"两个指尖碰到一起"这件事来标定指尖的位置（手套）或关节零点（机械手）。

模型：每个指尖是一个球（球心固定在远节坐标系里，半径已知）。两指接触时，两个球心的距离 = 两个半径之和。
      指尖对指尖、指腹对指腹、偏侧，都满足同一个方程，所以各种捏法都能用。
  手套 A/B：戴着做。未知数 = 4 个指尖球心在各自远节里的位置（12 个数），这是"你的手在手套里"的参数，换人重做。
  机械手 H：扭矩关掉，用手把它的指尖捏到一起。指尖球心来自 CAD，未知数变成 13 个关节的零点误差。

  python tools/tip_calib.py A record          # 录接触样本，追加到 data/contacts_A.json
  python tools/tip_calib.py A solve           # 解指尖球心 -> config/tips_A.json
  python tools/tip_calib.py A check           # 用现有参数看每个样本的残差（mm）
  python tools/tip_calib.py H record
  python tools/tip_calib.py H solve --apply   # 解零点误差并写回 config/calibration_H.json
  python tools/tip_calib.py A solve --zeros thumb   # 指尖和拇指 4 个零点一起解（零点可观性弱，靠先验压住）
  --radius 8            指尖球半径 mm（人的指腹约 8；机械手按 CAD 指尖块给）
  --tip-dof 2|3         手套指尖球心放开的方向数（默认 2：沿指、朝心；都限制在指套附近的小盒子里）
  --tips config/x.json  机械手指尖球心不用网格估计，用这个文件里的
  python tools/tip_calib.py A record --cuffs    # 手套脱下：拇指指套尖端对其他指套尖端，两边都是 CAD 已知点，然后 check 看模型本身对不对
  python tools/tip_calib.py A record --known    # 拇指指腹顶在三根手指【指套尖端的塑料】上：对方是 CAD 已知点，专门标拇指
  python tools/tip_calib.py A solve --scale thumb_mcp_flex_motor   # 经验修正：某个关节真实角 = 测量角 × 比例
  --only pads|cuffs|known   只用某几类样本（戴着的指腹样本 / 脱下的指套样本 / 指腹顶指套）
  solve 之前会自检：某个关节在所有样本里都"过伸"就是符号反了/零点错了，先去处理再解。
  solve / check 之后存一张检查图 data/tips_<设备>.png：灰球 = 先验球心，红球 = 解出的球心。

录制：拇指分别对食指、中指、无名指，每对 6 个样本，提示换不同捏法（指尖对指尖 / 指腹对指腹 / 偏侧 / 更弯 / 更直）。
      相邻手指的侧面接触默认不录（指套并排、侧摆行程小，碰不到）；真能碰到的话加 --lateral。
      按回车记录，u 撤销上一个，s 跳过这一对，q 结束。接触时要真的碰上、轻轻顶住，别用力压（指腹会变形）。
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import devices  # noqa: E402
import glove_model as gm  # noqa: E402
from dxl_bus import (Bus, TICKS_PER_REV, calib_path, effective_sign, load_calibration, load_registry,  # noqa: E402
                     save_calibration, ticks_to_rad, wrap_ticks)
from fk import FK  # noqa: E402

PAIRS = [("thumb", "index"), ("thumb", "mid"), ("thumb", "ring")]          # 拇指对三指；相邻手指的侧面接触外骨骼做不到，默认不录
LATERAL = [("index", "mid"), ("mid", "ring")]
KNOWN = [("thumb", "index_cuff"), ("thumb", "mid_cuff"), ("thumb", "ring_cuff")]   # 拇指指腹顶在指套尖端的塑料上：对方是 CAD 已知点
CUFFS = [("thumb_cuff", "index_cuff"), ("thumb_cuff", "mid_cuff"), ("thumb_cuff", "ring_cuff")]   # 手套脱下来：塑料尖端对塑料尖端，两边都是 CAD 已知点，没有未知数
HINTS_CUFFS = ["尖端对尖端", "拇指 MCP 更弯", "拇指 MCP 更直", "拇指 IP 弯", "对方手指更弯", "随便一个不一样的姿态"]
HINTS = ["指尖对指尖", "指腹对指腹", "偏向一侧", "两指都更弯一些", "两指都更直一些", "随便一个不一样的姿态"]
HINTS_KNOWN = ["拇指 MCP 尽量直（开口大）", "拇指 MCP 弯一半", "拇指 MCP 弯到底", "拇指 IP 伸直", "拇指 IP 弯", "随便一个不一样的姿态"]
W_TIP = 0.05     # 指尖先验权重：偏离先验 20 mm 相当于 1 mm 残差
W_ZERO = 0.3     # 零点先验权重：偏离 3.3° 相当于 1 mm 残差
JOINTS = sum(gm.FINGERS.values(), [])


def samples_path(dev):
    return os.path.join(gm.ROOT, "data", f"contacts_{dev['name']}.json")


def tips_path(dev):
    return os.path.join(gm.ROOT, "config", f"tips_{dev['name']}.json")


# ---------------- 录制 ----------------
def record(args, dev):
    reg, cal = load_registry(), load_calibration(dev["name"])
    axes = gm.joint_axes(gm.scene_of(dev["model"]))
    joints = [j for j in JOINTS if j in cal["joints"]]
    missing = [j for j in JOINTS if j not in joints]
    if missing:
        print(f"!! 没标定的关节（当 0° 处理，涉及它们的手指不要录）: {missing}")
    ids = {j: reg[j] for j in joints}
    sign = {j: effective_sign(cal["joints"][j], j, axes) for j in joints}
    bus = Bus(ids.values(), port=dev["port"])
    bus.torque_off()
    path = samples_path(dev)
    data = json.load(open(path)) if os.path.exists(path) else []
    print(f"* {dev['desc']}：扭矩已关。已有 {len(data)} 个样本（{path}）")
    print("  每对手指录 6 个，碰上后轻轻顶住再按回车。u 撤销，s 跳过这一对，q 结束\n")

    def read():
        raw, noise = bus.read_avg(10)
        q = {j: sign[j] * ticks_to_rad(wrap_ticks(raw[ids[j]] - cal["joints"][j]["zero_raw"])) for j in joints}
        return q, max(noise.values())

    pairs = CUFFS if args.cuffs else KNOWN if args.known else PAIRS + (LATERAL if args.lateral else [])
    if args.known:
        print("  --known：拇指【指腹】顶在对方【指套尖端的塑料】上（不是对方的指腹），顶住再按回车。每个样本换拇指 MCP/IP 的弯曲程度\n")
    if args.cuffs:
        print("  --cuffs：手套脱下来，用手把拇指指套的【塑料尖端】和对方指套的【塑料尖端】碰到一起，顶住再按回车。\n"
              "           两边都是 CAD 已知点，没有任何未知数，solve 不用跑，直接 check 看模型对不对\n")
    try:
        for f, g in pairs:
            hints = HINTS_CUFFS if f.endswith("_cuff") else HINTS_KNOWN if g.endswith("_cuff") else HINTS
            n = 0
            while n < args.per_pair:
                ans = input(f"  [{f} × {g}] {n + 1}/{args.per_pair}  {hints[n % len(hints)]}，碰上后按回车 ... ").strip().lower()
                if ans == "q":
                    raise KeyboardInterrupt
                if ans == "s":
                    break
                if ans == "u":
                    if data and data[-1]["pair"] == [f, g]:
                        data.pop(); n = max(0, n - 1); print("      已撤销")
                    continue
                q, noise = read()
                data.append({"pair": [f, g], "hint": hints[n % len(hints)], "q": q, "t": time.strftime("%Y-%m-%d %H:%M:%S")})
                n += 1
                print("      记下：" + "  ".join(f"{j.split('_', 1)[1].replace('_motor', '')} {math.degrees(q[j]):+.0f}°"
                                              for j in gm.FINGERS[f.replace("_cuff", "")] + gm.FINGERS[g.replace("_cuff", "")]) + (f"  （抖动 {noise} tick）" if noise > 3 else ""))
    except KeyboardInterrupt:
        pass
    finally:
        bus.close()
        os.makedirs(os.path.dirname(path), exist_ok=True)
        json.dump(data, open(path, "w"), indent=1)
        print(f"\n* 共 {len(data)} 个样本，已存 {os.path.relpath(path)}。接着: python tools/tip_calib.py {dev['name']} solve")


# ---------------- 模型与求解 ----------------
def expand_joints(names):
    out = []
    for n in names:
        if n in gm.FINGERS:
            out += gm.FINGERS[n]
        elif n in JOINTS:
            out.append(n)
        elif n == "all":
            out += JOINTS
        else:
            sys.exit(f"不认识的关节/手指 {n}")
    return out


class Problem:
    """
    未知数：
      指尖球心（手套）：每根手指 2 个数 —— 沿手指方向、朝手心方向，各用 L·tanh(x) 限制在 ±TIP_BOX 以内，
                       所以解出来的球心一定还在指套附近；--tip-dof 3 放开侧向（容易掉进"所有球心都缩到手掌里"的假解）。
      零点误差：--zeros 指定的关节（机械手默认全部，手套默认不解），有先验 W_ZERO。
    """
    TIP_BOX = (0.015, 0.012, 0.008)       # 沿指 / 朝心 / 侧向 的最大偏移 m

    def __init__(self, dev, args, use_saved=False):
        import mujoco
        self.fk = FK(dev["model"])
        self.radius = {f: args.radius / 1000 for f in gm.FINGERS}
        tips_file = args.tips or (tips_path(dev) if use_saved and os.path.exists(tips_path(dev)) else None)
        if tips_file:
            t = json.load(open(tips_file))
            self.c0 = {f: np.array(t["fingers"][f]["center_m"]) for f in gm.FINGERS}
            self.radius = {f: t["fingers"][f].get("radius_mm", args.radius) / 1000 for f in gm.FINGERS}
            print(f"* 指尖球心来自 {os.path.relpath(tips_file)}")
        elif dev["model"] == "hand":
            self.c0 = {f: self.fk.mesh_tip(f, self.radius[f]) for f in gm.FINGERS}
            print("* 机械手指尖球心按 CAD 网格估计（最远点往回退一个半径）；想更准就写 config/tips_H.json")
        else:
            # 人的指尖：指套包到指尖，指腹中心约在指套尖端往回半个半径、再往手心 6 mm
            self.c0 = {f: self.fk.mesh_tip(f, self.radius[f] * 0.5, palm_offset=0.006) for f in gm.FINGERS}
        # 每根手指在远节坐标系里的"沿指 / 朝心 / 侧向"单位向量（零位算）
        m, d = self.fk.m, self.fk.d
        d.qpos[:] = 0; mujoco.mj_kinematics(m, d); mujoco.mj_forward(m, d)
        n_w = gm.dorsal_normal(m, d)
        self.axes = {}
        for f, js in gm.FINGERS.items():
            a0, a1 = d.xanchor[self.fk.jid[js[0]]], d.xanchor[self.fk.jid[js[-1]]]
            u = (a1 - a0) / np.linalg.norm(a1 - a0)
            n = -n_w - np.dot(-n_w, u) * u; n /= np.linalg.norm(n)
            R = self.fk.d.xmat[self.fk.tipb[f]].reshape(3, 3)
            self.axes[f] = np.array([R.T @ u, R.T @ n, R.T @ np.cross(u, n)])
        self.cuff = {f: self.fk.mesh_end(f) for f in gm.FINGERS}        # 指套尖端中心（远节系），--known 样本用
        self.tip_dof = args.tip_dof if args.fit_tips else 0
        self.zero_joints = args.zero_joints
        self.scale_joints = getattr(args, "scale_joints", [])

    def unpack(self, x):
        c, dz, sc, k = dict(self.c0), {j: 0.0 for j in JOINTS}, {j: 1.0 for j in JOINTS}, 0
        for f in gm.FINGERS:
            if self.tip_dof:
                off = np.zeros(3)
                for i in range(self.tip_dof):
                    off += self.TIP_BOX[i] * math.tanh(x[k + i]) * self.axes[f][i]
                c[f] = self.c0[f] + off; k += self.tip_dof
        for j in self.zero_joints:
            dz[j] = x[k]; k += 1
        for j in self.scale_joints:
            sc[j] = 1.0 + x[k]; k += 1
        return c, dz, sc

    def nparam(self):
        return 4 * self.tip_dof + len(self.zero_joints) + len(self.scale_joints)

    def residuals(self, x, samples):
        c, dz, sc = self.unpack(x)
        r = []
        for s in samples:
            q = {j: (s["q"].get(j, 0.0) + dz[j]) * sc[j] for j in JOINTS}
            poses = self.fk.distal(q)
            pts, target = [], 0.0
            for name in s["pair"]:
                if name.endswith("_cuff"):               # 指套尖端的已知点，半径 0
                    gf = name[:-5]
                    pts.append(poses[gf][0] @ self.cuff[gf] + poses[gf][1])
                else:                                   # 指腹球
                    pts.append(poses[name][0] @ c[name] + poses[name][1])
                    target += self.radius[name]
            r.append((np.linalg.norm(pts[0] - pts[1]) - target) * 1000)
        k = 4 * self.tip_dof
        r += list(W_TIP * x[:k] * 10)                              # tanh 自变量 1 ≈ 盒子的 76%
        r += list(W_ZERO * np.degrees(x[k:k + len(self.zero_joints)]))
        k += len(self.zero_joints)
        r += list(0.5 * x[k:k + len(self.scale_joints)])          # 比例先验很弱：×1.1 只算 0.05 mm
        return np.array(r)


def lm(fun, x0, n_contact, iters=40):
    """Levenberg–Marquardt，数值雅可比；fun 返回残差向量（前 n_contact 个是接触残差）"""
    x = x0.copy(); lam = 1e-2
    r = fun(x); cost = float(r @ r)
    for it in range(iters):
        J = np.zeros((len(r), len(x)))
        for i in range(len(x)):
            dx = np.zeros_like(x); dx[i] = 1e-5
            J[:, i] = (fun(x + dx) - r) / 1e-5
        A, g = J.T @ J, J.T @ r
        while True:
            step = np.linalg.solve(A + lam * np.diag(np.diag(A) + 1e-9), -g)
            r2 = fun(x + step); c2 = float(r2 @ r2)
            if c2 < cost:
                x, r, cost, lam = x + step, r2, c2, max(lam / 3, 1e-6)
                break
            lam *= 4
            if lam > 1e6:
                return x, r, J
        if np.linalg.norm(step) < 1e-7:
            break
    return x, r, J


def sanity(samples, dev):
    """按方向表把样本换成"弯曲角"，某个关节所有样本都明显过伸就报警（符号反了、或零点错很多）"""
    D = gm.directions(gm.scene_of(dev["model"]))
    print("  样本里各关节的弯曲角范围（负 = 伸直方向；方向表按手心法线判断，对拇指不可靠，拇指只列不判）")
    bad = []
    for j in JOINTS:
        v = [math.degrees(s["q"].get(j, 0.0)) * D[j] for s in samples]
        flag = ""
        if "abd" not in j and max(v) < -15 and not j.startswith("thumb"):
            flag = "  <- 全部过伸，不可能：这个关节的符号反了，或者零点错了很多"; bad.append(j)
        print(f"    {j:22s} {min(v):+5.0f} … {max(v):+5.0f}°{flag}")
    return bad


def render_check(P, samples, c, dz, path, n_show=3):
    """零位 + 几个样本：灰球 = 先验球心，红球 = 解出的球心（手心侧视角）。存 png"""
    os.environ.setdefault("MUJOCO_GL", "osmesa")
    try:
        import mujoco
        from PIL import Image
        m, d = P.fk.m, P.fk.d
        ren = mujoco.Renderer(m, 480, 640)
    except Exception as e:   # noqa: BLE001
        print(f"  （渲染检查图失败，跳过：{e}）")
        return
    d.qpos[:] = 0; mujoco.mj_forward(m, d)
    root = m.body_rootid[P.fk.tipb["index"]]
    rgba0 = m.geom_rgba.copy()
    for g in range(m.ngeom):                                   # 只画这台设备，场景里的物体、地面都藏掉
        if m.body_rootid[m.geom_bodyid[g]] != root:
            m.geom_rgba[g, 3] = 0
    n_w = gm.dorsal_normal(m, d)
    center = np.mean([d.xpos[b] for b in P.fk.tipb.values()] + [d.xpos[1]], axis=0)
    u_idx = d.xpos[P.fk.tipb["index"]] - d.xanchor[P.fk.jid["index_mcp_flex_motor"]]; u_idx /= np.linalg.norm(u_idx)
    v = -n_w * 0.9 + u_idx * 0.4; v /= np.linalg.norm(v)
    cam = mujoco.MjvCamera(); cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = center; cam.distance = 0.28
    cam.elevation = -math.degrees(math.asin(v[2])); cam.azimuth = math.degrees(math.atan2(v[1], v[0])) + 180
    opt = mujoco.MjvOption()

    def sphere(pos, rgba):
        g = ren.scene.geoms[ren.scene.ngeom]
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE, np.array([0.008, 0, 0]), np.asarray(pos, float), np.eye(3).flatten(), np.array(rgba, np.float32))
        ren.scene.ngeom += 1

    imgs = []
    picks = [None] + [i for i in np.linspace(0, len(samples) - 1, n_show).round().astype(int)]
    for i in picks:
        q = {} if i is None else {j: samples[i]["q"].get(j, 0.0) + dz[j] for j in JOINTS}
        d.qpos[:] = P.fk.qpos(q); mujoco.mj_forward(m, d)
        Rb, pb = d.xmat[1].reshape(3, 3), d.xpos[1]
        po = P.fk.distal(q)
        ren.update_scene(d, cam, opt)
        for f in gm.FINGERS:
            sphere(Rb @ (po[f][0] @ P.c0[f] + po[f][1]) + pb, [0.5, 0.5, 0.5, 0.5])
            sphere(Rb @ (po[f][0] @ c[f] + po[f][1]) + pb, [1, 0.1, 0.1, 0.8])
        if i is not None:
            for name in samples[i]["pair"]:
                if name.endswith("_cuff"):
                    gf = name[:-5]
                    sphere(Rb @ (po[gf][0] @ P.cuff[gf] + po[gf][1]) + pb, [0.1, 0.3, 1, 0.9])
        imgs.append(ren.render().copy())
    m.geom_rgba[:] = rgba0
    Image.fromarray(np.concatenate(imgs, axis=1)).save(path)
    print(f"  检查图已存 {os.path.relpath(path)}（左起：零位、样本 {', '.join('#' + str(i) for i in picks[1:])}；灰球先验、红球解）")


def kind_of(s):
    f, g = s["pair"]
    return "cuffs" if f.endswith("_cuff") else "known" if g.endswith("_cuff") else "pads"


def load_samples(dev, only):
    path = samples_path(dev)
    if not os.path.exists(path):
        sys.exit(f"没有样本文件 {path}，先 record")
    samples = json.load(open(path))
    if only:
        samples = [s for s in samples if kind_of(s) in only]
    return path, samples


def solve(args, dev, apply):
    path, samples = load_samples(dev, args.only)
    if len(samples) < 8:
        sys.exit(f"只有 {len(samples)} 个样本，至少 8 个（每对手指都要有几个不同姿态）")
    print(f"* {dev['desc']}：{len(samples)} 个接触样本")
    bad = sanity(samples, dev)
    if bad and not args.force:
        sys.exit(f"!! 先处理 {bad} 的符号/零点（mg.py {dev['name']} live 看一下），或者加 --force 硬解")
    P = Problem(dev, args, use_saved=(dev["model"] == "hand"))
    n = len(samples)
    x0 = np.zeros(P.nparam())
    r0 = P.residuals(x0, samples)[:n]
    what = (f"指尖球心（每指 {P.tip_dof} 个自由度）" if P.tip_dof else "") + (" + " if P.tip_dof and P.zero_joints else "") + \
           (f"{len(P.zero_joints)} 个关节零点" if P.zero_joints else "") + \
           (f" + {len(P.scale_joints)} 个关节比例" if P.scale_joints else "")
    print(f"\n* 解 {what}")
    print(f"  解之前：接触残差 RMS {np.sqrt(np.mean(r0 ** 2)):.2f} mm，最大 {np.abs(r0).max():.2f} mm")
    if P.nparam() == 0:
        sys.exit("没有要解的参数（--fit tips 或 --zeros ...）")
    x, r, J = lm(lambda v: P.residuals(v, samples), x0, n, iters=80)
    rc = r[:n]
    rms = float(np.sqrt(np.mean(rc ** 2)))
    print(f"  解之后：接触残差 RMS {rms:.2f} mm，最大 {np.abs(rc).max():.2f} mm" + ("  <- 还是解释不了：不是指尖位置的问题，查关节零点/符号/机构" if rms > 3 else ""))
    sv = np.linalg.svd(J[:n], compute_uv=False)
    print(f"  可观性：雅可比奇异值 最大 {sv[0]:.2f} 最小 {sv[-1]:.3f}" + ("  <- 有参数几乎看不出来，多录些不同姿态" if sv[-1] < 0.05 * sv[0] else ""))
    worst = np.argsort(-np.abs(rc))[:5]
    print("  残差最大的样本：" + "；".join(f"#{i} {samples[i]['pair'][0]}×{samples[i]['pair'][1]} {samples[i]['hint']} {rc[i]:+.1f}mm" for i in worst))
    c, dz, sc = P.unpack(x)
    if P.scale_joints:
        print("\n  关节比例（真实角 = 测量角 × 比例；只是经验修正，解释不了就去查机构）")
        for j in P.scale_joints:
            print(f"    {j:22s} ×{sc[j]:.2f}" + ("  <- 离 1 很远" if abs(sc[j] - 1) > 0.25 else ""))
    if P.tip_dof:
        print("\n  指尖球心（远节坐标系，mm）        相对先验：沿指 / 朝心 / 侧向（盒子 ±15/12/8）")
        for f in gm.FINGERS:
            d = P.axes[f] @ (c[f] - P.c0[f]) * 1000
            edge = "  <- 顶到盒子边了" if np.any(np.abs(d) > 0.9 * np.array(P.TIP_BOX) * 1000) else ""
            print(f"    {f:6s} ({c[f][0]*1000:+7.2f}, {c[f][1]*1000:+7.2f}, {c[f][2]*1000:+7.2f})   {d[0]:+6.1f} / {d[1]:+6.1f} / {d[2]:+6.1f}{edge}")
        import mujoco
        out = {"radius_mm": args.radius, "device": dev["name"], "model": dev["model"], "n_samples": n, "rms_mm": rms,
               "joint_scale": {j: float(sc[j]) for j in P.scale_joints},
               "fingers": {f: {"body": mujoco.mj_id2name(P.fk.m, mujoco.mjtObj.mjOBJ_BODY, P.fk.tipb[f]),
                               "center_m": [float(v) for v in c[f]], "radius_mm": args.radius,
                               "cuff_end_m": [float(v) for v in P.cuff[f]]} for f in gm.FINGERS}}
        json.dump(out, open(tips_path(dev), "w"), indent=1)
        print(f"  已存 {os.path.relpath(tips_path(dev))}")
    if P.zero_joints:
        print("\n  关节零点误差（真实角 = 测量角 + 误差）")
        for j in P.zero_joints:
            print(f"    {j:22s} {math.degrees(dz[j]):+6.2f}°" + ("  <- 很大，先确认机械上没打滑" if abs(math.degrees(dz[j])) > 10 else ""))
        if apply:
            cal = load_calibration(dev["name"])
            axes = gm.joint_axes(gm.scene_of(dev["model"]))
            k = TICKS_PER_REV / (2 * math.pi)
            for j in P.zero_joints:
                if j in cal["joints"]:
                    s = effective_sign(cal["joints"][j], j, axes)
                    cal["joints"][j]["zero_raw"] = float((cal["joints"][j]["zero_raw"] - s * dz[j] * k) % TICKS_PER_REV)
            save_calibration(cal, dev["name"])
            all_samples = json.load(open(path))
            for s_ in all_samples:                   # 样本里的角度也按新零点修正（所有样本），之后 check 时才对得上
                for j in P.zero_joints:
                    if j in s_["q"]:
                        s_["q"][j] += dz[j]
            json.dump(all_samples, open(path, "w"), indent=1)
            for s_ in samples:
                for j in P.zero_joints:
                    if j in s_["q"]:
                        s_["q"][j] += dz[j]
            dz = {j: 0.0 for j in JOINTS}
            print(f"  已写回 {os.path.relpath(calib_path(dev['name']))}，样本文件里的角度也已同步")
        else:
            print("  （加 --apply 写回标定文件）")
    render_check(P, samples, c, dz, os.path.join(gm.ROOT, "data", f"tips_{dev['name']}.png"))


def check(args, dev):
    _, samples = load_samples(dev, args.only)
    print(f"* {dev['desc']}：{len(samples)} 个样本")
    sanity(samples, dev)
    args.fit_tips = False; args.zero_joints = []; args.scale_joints = []
    P = Problem(dev, args, use_saved=True)
    r = P.residuals(np.zeros(0), samples)
    print(f"\n  用现有参数：接触残差 RMS {np.sqrt(np.mean(r ** 2)):.2f} mm，最大 {np.abs(r).max():.2f} mm")
    for i, (s, v) in enumerate(zip(samples, r)):
        print(f"  #{i:2d} {s['pair'][0]:5s}×{s['pair'][1]:5s} {s['hint']:12s} {v:+6.2f} mm")
    render_check(P, samples, P.c0, {j: 0.0 for j in JOINTS}, os.path.join(gm.ROOT, "data", f"tips_{dev['name']}_check.png"))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("device", help="A / B / H")
    ap.add_argument("cmd", choices=["record", "solve", "check"])
    ap.add_argument("--per-pair", type=int, default=6, help="每对手指录几个样本（默认 6）")
    ap.add_argument("--radius", type=float, default=None, help="指尖球半径 mm（默认：手套 8，机械手 6）")
    ap.add_argument("--fit", choices=["tips", "zeros", "both"], default=None, help="解什么（默认：手套 tips，机械手 zeros）")
    ap.add_argument("--tip-dof", type=int, choices=[2, 3], default=2, help="指尖球心放开几个方向（默认 2：沿指、朝心）")
    ap.add_argument("--zeros", nargs="*", default=None, help="一起解零点的关节或手指，如 --zeros thumb（--fit zeros/both 时默认 all）")
    ap.add_argument("--scale", nargs="*", default=[], help="一起解角度比例的关节（经验修正），如 --scale thumb_mcp_flex_motor")
    ap.add_argument("--known", action="store_true", help="record：拇指指腹顶在食指/中指/无名指指套尖端（CAD 已知点），用来单独标拇指")
    ap.add_argument("--cuffs", action="store_true", help="record：手套脱下，拇指指套尖端对其他指套尖端（两边都是 CAD 已知点），检验模型本身")
    ap.add_argument("--tips", default=None, help="指尖球心文件（机械手按 CAD 量好的）")
    ap.add_argument("--apply", action="store_true", help="solve 时把零点误差写回标定文件")
    ap.add_argument("--force", action="store_true", help="样本自检报警也照样解")
    ap.add_argument("--only", nargs="*", choices=["pads", "cuffs", "known"], default=None,
                    help="只用这几类样本：pads 戴着手套指腹对指腹 / cuffs 手套脱下指套对指套 / known 指腹顶指套尖端")
    ap.add_argument("--lateral", action="store_true", help="record 时也录食指-中指、中指-无名指的侧面接触")
    args = ap.parse_args()
    dev = devices.get(args.device)
    if args.radius is None:
        args.radius = 8.0 if dev["model"] == "glove" else 6.0
    if args.fit is None:
        args.fit = "tips" if dev["model"] == "glove" else "zeros"
    args.fit_tips = args.fit in ("tips", "both")
    if args.zeros is None:
        args.zeros = ["all"] if args.fit in ("zeros", "both") else []
    args.zero_joints = expand_joints(args.zeros)
    args.scale_joints = expand_joints(args.scale)
    {"record": lambda: record(args, dev), "solve": lambda: solve(args, dev, args.apply), "check": lambda: check(args, dev)}[args.cmd]()


if __name__ == "__main__":
    main()
