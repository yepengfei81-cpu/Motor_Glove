#!/usr/bin/env python3
"""
对比手套（glove_mujoco）和同构机械手（hand_mujoco）的运动学，回答"遥操作时关节怎么一一对应、差多少"。

  python tools/compare_kin.py                  # 全部 13 个关节 + 指尖对比
  python tools/compare_kin.py --save           # 另外把映射表存到 config/glove_to_hand.json（遥操作用）

逐个关节：
  1. 真实转轴：只转这一个电机，拟合"输出连杆"在空间里绕的那条固定轴。
     直驱关节这就是电机轴；手套上连杆驱动的关节（PIP、中指 MCP、无名指 MCP、拇指 IP）这是解剖关节的转轴。
  2. 两边轴的夹角（应接近 0° 或 180°；180° = 正方向相反，机械手关节角要取负号）和轴线平移量（mm）
  3. 手套电机角 θ -> 真实关节角 φ 的关系。机械手是直驱的，它的关节要执行 φ(θ)，不是 θ
然后在几个姿态下对比两边的指尖位置（同一组真实关节角）。
坐标：两个模型根连杆的坐标系 = 各自 Onshape 装配体的原点（导出时就是这样），所以要求两个装配体用同一套坐标设计。
"""
import argparse
import json
import math
import os
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import glove_model as gm  # noqa: E402

GLOVE = os.path.join(gm.ROOT, "glove_mujoco", "robot.xml")
HAND = os.path.join(gm.ROOT, "hand_mujoco", "robot.xml")


class Model:
    def __init__(self, path):
        self.m = mujoco.MjModel.from_xml_path(path)
        self.m.opt.disableflags = int(self.m.opt.disableflags) | int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
        self.m.opt.gravity[:] = 0
        self.d = mujoco.MjData(self.m)
        self.root = 1                                            # world 下第一个 body = 根连杆
        self.closed = self.m.neq > 0
        self.act = {mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, i): i for i in range(self.m.nu)}

    def jid(self, name):
        return mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, name)

    def set(self, q):
        """q: {电机关节名: 角度 rad}。有闭链的用执行器 + 仿真收敛，开链的直接写关节角"""
        if self.closed:
            # 闭链（手套的连杆）要小步走过去：一下跳到大角度，仿真里连杆可能落到另一个装配分支
            start = {j: float(self.d.ctrl[self.act[j]]) for j in q}
            n = max(1, int(max([abs(v - start[j]) for j, v in q.items()] + [0]) / math.radians(2)))
            for k in range(1, n + 1):
                for j, v in q.items():
                    self.d.ctrl[self.act[j]] = start[j] + (v - start[j]) * k / n
                for _ in range(300 if k < n else 1500):
                    mujoco.mj_step(self.m, self.d)
        else:
            for j, v in q.items():
                self.d.qpos[self.m.jnt_qposadr[self.jid(j)]] = v
        mujoco.mj_forward(self.m, self.d)

    def to_base(self, p=None, v=None):
        Rb, pb = self.d.xmat[self.root].reshape(3, 3), self.d.xpos[self.root]
        return Rb.T @ (p - pb) if p is not None else Rb.T @ v

    def pose(self, b):
        Rb = self.d.xmat[self.root].reshape(3, 3)
        return Rb.T @ self.d.xmat[b].reshape(3, 3), self.to_base(p=self.d.xpos[b].copy())

    def output_body(self, finger, k):
        """第 k 个电机关节驱动的那段连杆：下一个电机关节的父 body；最后一个关节取远节（手套是被动的 PIP/IP 关节所在 body）"""
        js = gm.FINGERS[finger]
        if k + 1 < len(js):
            return int(self.m.body_parentid[self.m.jnt_bodyid[self.jid(js[k + 1])]])
        tip = gm.TIP_JOINT[finger]
        return int(self.m.jnt_bodyid[self.jid(tip)]) if self.jid(tip) >= 0 else int(self.m.jnt_bodyid[self.jid(js[k])])


def rotvec(R):
    q = np.zeros(4); mujoco.mju_mat2Quat(q, R.ravel())
    v = np.zeros(3); mujoco.mju_quat2Vel(v, q, 1.0)
    return v


def fit_axis(mdl, finger, k, angles):
    """只转第 k 个电机，返回 (轴方向, 轴上一点, 漂移 mm, [(θ, φ)])，都在根坐标系（零位）"""
    j = gm.FINGERS[finger][k]
    b = mdl.output_body(finger, k)
    zero = {jj: 0.0 for js in gm.FINGERS.values() for jj in js}
    Rs, ps, th = [], [], []
    for a in angles:
        q = dict(zero); q[j] = a
        mdl.set(q)
        R, p = mdl.pose(b)
        Rs.append(R); ps.append(p); th.append(mdl.d.qpos[mdl.m.jnt_qposadr[mdl.jid(j)]])
    mdl.set(zero)
    i0 = int(np.argmin(np.abs(np.array(angles))))
    R0 = Rs[i0]
    k_far = int(np.argmax([np.linalg.norm(rotvec(R @ R0.T)) for R in Rs]))
    ax = rotvec(Rs[k_far] @ R0.T); ax /= np.linalg.norm(ax)
    if (th[k_far] - th[i0]) * np.dot(rotvec(Rs[k_far] @ R0.T), ax) < 0:
        ax = -ax                                               # 让 φ 和 θ 同号：轴方向 = 电机正方向对应的转向
    n = len(Rs)
    A = np.zeros((3 * n, 6)); rhs = np.zeros(3 * n)
    for t in range(n):
        A[3 * t:3 * t + 3, :3] = Rs[t]; A[3 * t:3 * t + 3, 3:] = -np.eye(3); rhs[3 * t:3 * t + 3] = -ps[t]
    c = np.linalg.lstsq(A, rhs, rcond=None)[0][3:]
    x = R0.T @ (c - ps[i0])
    drift = max(np.linalg.norm(Rs[t] @ x + ps[t] - c) for t in range(n)) * 1000
    phi = [float(np.dot(rotvec(R @ R0.T), ax)) for R in Rs]
    return ax, c, drift, list(zip([t - th[i0] for t in th], phi))


def line_offset(c1, a1, c2, a2):
    """两条（近似平行的）轴线之间的平移：从线 1 上离 c2 最近的点指向线 2，取垂直于 a1 的分量，mm"""
    p1 = c1 + a1 * np.dot(c2 - c1, a1)
    v = c2 - p1
    v = v - a1 * np.dot(v, a1)
    return v * 1000


def tip(mdl, finger):
    """远节网格上沿手指方向最远的点（根坐标系）"""
    js = gm.FINGERS[finger]
    b = mdl.output_body(finger, len(js) - 1)
    a0 = mdl.to_base(p=mdl.d.xanchor[mdl.jid(js[0])].copy())
    a1 = mdl.to_base(p=mdl.d.xanchor[mdl.jid(js[-1])].copy())
    u = (a1 - a0) / (np.linalg.norm(a1 - a0) + 1e-9)
    pts = []
    for g in range(mdl.m.ngeom):
        if mdl.m.geom_bodyid[g] == b and mdl.m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
            mid = mdl.m.geom_dataid[g]
            v = mdl.m.mesh_vert[mdl.m.mesh_vertadr[mid]: mdl.m.mesh_vertadr[mid] + mdl.m.mesh_vertnum[mid]]
            w = v @ mdl.d.geom_xmat[g].reshape(3, 3).T + mdl.d.geom_xpos[g]
            pts.append(np.array([mdl.to_base(p=x) for x in w]))
    p = np.concatenate(pts)
    return p[np.argmax((p - a1) @ u)]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--save", action="store_true")
    args = ap.parse_args()
    G, H = Model(GLOVE), Model(HAND)
    md = gm.directions()                                       # 手套：哪个方向是握拳
    G.set({}); H.set({})
    result = {}
    print(f"{'关节':24s} {'轴夹角':>7s} {'平移 mm':>8s}  {'平移向量 (x, y, z) mm':>26s}  {'手套漂移':>8s}  {'φ/θ':>6s}  手套 θ→φ（θ = 0/20/40°）")
    for f, js in gm.FINGERS.items():
        for k, j in enumerate(js):
            # 拟合范围：侧摆 ±15°；弯曲到 60°；拇指 IP 是连杆、行程大，扫到 90°（中指 MCP 超过 65° 连杆进入奇异区，不扫）
            span = math.radians(15 if "abd" in j else 90 if j == "thumb_ip_motor" else 60)
            angs = list(np.linspace(-span / 4, span, 19) * md[j])
            ag, cg, dg, curve = fit_axis(G, f, k, angs)
            ah, ch, dh, _ = fit_axis(H, f, k, angs)
            dot = float(np.dot(ag, ah))
            sign = 1 if dot >= 0 else -1
            ang = math.degrees(math.acos(min(1.0, abs(dot))))
            off = line_offset(cg, ag, ch, ah)
            th = np.array([t for t, _ in curve]); ph = np.array([p for _, p in curve])
            V = np.vstack([th ** 3, th ** 2, th]).T                    # 过原点的三次多项式：θ = 0 时 φ = 0
            pf = np.append(np.linalg.lstsq(V, ph, rcond=None)[0], 0.0)
            ratio = float(np.polyfit(th, ph, 1)[0])
            samples = [math.degrees(np.polyval(pf, math.radians(x) * md[j])) * md[j] for x in (0, 20, 40)]
            result[j] = dict(sign=sign, axis_angle_deg=ang, offset_mm=[float(v) for v in off],
                             glove_drift_mm=dg, poly_phi_of_theta=[float(c) for c in pf], ratio=ratio)
            lin = "直驱" if abs(ratio - 1) < 0.01 and dg < 0.3 else "连杆"
            print(f"{j:24s} {ang:6.2f}° {np.linalg.norm(off):7.2f}   ({off[0]:+6.2f}, {off[1]:+6.2f}, {off[2]:+6.2f})   {dg:6.2f}mm  {ratio:5.3f}  "
                  f"{lin} " + " / ".join(f"{s:+.1f}°" for s in samples) + ("  符号相反" if sign < 0 else ""))

    # 指尖对比：两边远节零件不同（手套是指套，机械手是机器人指尖），所以在零位时把手套的指尖点挂到两边各自的远节上，
    # 之后只比较这个"同一个点"跟着关节走出来的位置 —— 结果只反映转轴平移 + 角度映射带来的差别
    G.set({}); H.set({})
    anchor = {}
    for f, js in gm.FINGERS.items():
        t0 = tip(G, f)
        for name, mdl in (("G", G), ("H", H)):
            R0, p0 = mdl.pose(mdl.output_body(f, len(js) - 1))
            anchor[(name, f)] = R0.T @ (t0 - p0)
    def vtip(mdl, name, f):
        R, p = mdl.pose(mdl.output_body(f, len(gm.FINGERS[f]) - 1))
        return R @ anchor[(name, f)] + p
    print("\n同一个指尖点（零位时取手套指尖，挂在两边各自的远节上）在几个姿态下的位置差，mm：")
    poses = {"零位": 0, "半握 30°": 30, "握拳 60°": 60}
    for pname, deg in poses.items():
        th = {j: md[j] * math.radians(deg * (0.3 if "abd" in j else 1)) for js in gm.FINGERS.values() for j in js}
        G.set(th)
        got = {j: G.d.qpos[G.m.jnt_qposadr[G.jid(j)]] for j in th}
        worst = max(abs(math.degrees(got[j] - th[j])) for j in th)
        qh = {j: result[j]["sign"] * float(np.polyval(result[j]["poly_phi_of_theta"], got[j])) for j in th}
        H.set(qh)
        row = [f"{f} {np.linalg.norm(vtip(H, 'H', f) - vtip(G, 'G', f)) * 1000:5.2f}" for f in gm.FINGERS]
        print(f"   {pname:10s} " + "   ".join(row) + f"     （手套仿真里电机角和指令最多差 {worst:.1f}°）")
    G.set({}); H.set({})
    if args.save:
        out = os.path.join(gm.ROOT, "config", "glove_to_hand.json")
        json.dump(result, open(out, "w"), indent=1)
        print(f"\n* 映射表已存 {os.path.relpath(out)}（每个关节：符号、φ(θ) 三次多项式系数、轴平移）")


if __name__ == "__main__":
    main()
