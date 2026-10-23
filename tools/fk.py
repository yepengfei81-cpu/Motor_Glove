"""
快速正运动学：给 13 个电机关节角，算每根手指远节的位姿（基座坐标系），不跑仿真。

手套模型里远节是经过被动关节（PIP / IP / 中指 MCP 的两个连杆铰）挂在树上的，被动关节由同一根手指的一个电机唯一决定。
第一次用时在仿真里把每个驱动电机扫一遍，把"被动关节角 = f(电机角)"拟成三次多项式，缓存到模型目录的 passive_map.json；
之后只要把电机角和多项式算出的被动角一起写进 qpos、调一次 mj_kinematics 就是正确的树正运动学（闭链约束在扫描时已经满足）。
直驱机械手没有被动关节，直接写 qpos。

    fk = FK("glove")                # 或 "hand"
    poses = fk.distal({j: rad, ...})   # {finger: (R 3x3, p 3)}，基座坐标系
"""
import json
import math
import os

import mujoco
import numpy as np

import glove_model as gm

MOTORS = set(sum(gm.FINGERS.values(), []))
MOTOR_MESH = __import__("re").compile(r"^(dc15_a01_\w+_dummy|phs_1_7x20_5_dc10|bts2_m2_6x8|b3b_eh)")   # 电机散件的网格名


class FK:
    def __init__(self, model="glove"):
        self.model = model
        self.scene = gm.scene_of(model)
        self.m = mujoco.MjModel.from_xml_path(self.scene)
        self.d = mujoco.MjData(self.m)
        self.base = 1
        self.jid = {mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_JOINT, j): j for j in range(self.m.njnt)}
        self.tipb = {f: gm.tip_body(self.m, f) for f in gm.FINGERS}
        # 每根手指：基座到远节路径上的关节
        self.path = {}
        for f, b in self.tipb.items():
            js = []
            while b > 0:
                js = [mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_JOINT, self.m.body_jntadr[b] + k)
                      for k in range(self.m.body_jntnum[b])] + js
                b = self.m.body_parentid[b]
            self.path[f] = js
        self.passive = {}                     # 被动关节名 -> (驱动电机名, 多项式系数)
        cache = os.path.join(os.path.dirname(self.scene), "passive_map.json")
        need = [j for f in gm.FINGERS for j in self.path[f] if j not in MOTORS]
        if need:
            if os.path.exists(cache) and os.path.getmtime(cache) > os.path.getmtime(os.path.join(os.path.dirname(self.scene), "robot.xml")):
                self.passive = {k: (v[0], np.array(v[1])) for k, v in json.load(open(cache)).items()}
            else:
                self._fit_passive(need)
                json.dump({k: (v[0], list(map(float, v[1]))) for k, v in self.passive.items()}, open(cache, "w"), indent=1)

    # ---- 被动关节拟合（只做一次） ----
    def _fit_passive(self, need):
        m = mujoco.MjModel.from_xml_path(self.scene)
        m.opt.disableflags = int(m.opt.disableflags) | int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
        m.opt.gravity[:] = 0
        d = mujoco.MjData(m)
        act = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i): i for i in range(m.nu)}
        qadr = lambda n: m.jnt_qposadr[self.jid[n]]
        for _ in range(1500):
            mujoco.mj_step(m, d)
        rest = {n: float(d.qpos[qadr(n)]) for n in need}
        for f, js in gm.FINGERS.items():
            pas = [j for j in self.path[f] if j not in MOTORS]
            if not pas:
                continue
            for motor in js:
                # 这个电机从 0 扫到 ±范围，看哪些被动关节跟着动
                lim = math.radians(65 if motor == "mid_mcp_motor" else 90 if "ip" in motor or "pip" in motor else 60)
                sweep = list(np.linspace(0, lim, 14)) + list(np.linspace(0, -lim / 4, 4))[1:]
                th, ph = [], {p: [] for p in pas}
                mujoco.mj_resetData(m, d)
                for a in sweep:
                    if a == 0 or (a < 0 and a == sweep[14]):
                        mujoco.mj_resetData(m, d)
                    d.ctrl[act[motor]] = a
                    for _ in range(800):
                        mujoco.mj_step(m, d)
                    th.append(float(d.qpos[qadr(motor)]))
                    for p in pas:
                        ph[p].append(float(d.qpos[qadr(p)]))
                th = np.array(th)
                for p in pas:
                    y = np.array(ph[p]) - rest[p]
                    if np.ptp(y) < math.radians(2):
                        continue                                  # 这个电机不驱动它
                    V = np.vstack([th ** 3, th ** 2, th]).T
                    c = np.linalg.lstsq(V, y, rcond=None)[0]
                    self.passive[p] = (motor, np.array([*c, rest[p]]))
        missing = [p for p in need if p not in self.passive]
        if missing:
            raise RuntimeError(f"这些被动关节找不到驱动电机: {missing}")

    # ---- 正运动学 ----
    def qpos(self, q):
        """q: {电机关节名: rad}（没给的电机按 0）-> 完整 qpos"""
        qp = np.zeros(self.m.nq)
        for j in MOTORS:
            qp[self.m.jnt_qposadr[self.jid[j]]] = q.get(j, 0.0)
        for p, (motor, c) in self.passive.items():
            t = q.get(motor, 0.0)
            qp[self.m.jnt_qposadr[self.jid[p]]] = c[0] * t ** 3 + c[1] * t ** 2 + c[2] * t + c[3]
        return qp

    def distal(self, q):
        self.d.qpos[:] = self.qpos(q)
        mujoco.mj_kinematics(self.m, self.d)
        Rb, pb = self.d.xmat[self.base].reshape(3, 3), self.d.xpos[self.base]
        return {f: (Rb.T @ self.d.xmat[b].reshape(3, 3), Rb.T @ (self.d.xpos[b] - pb)) for f, b in self.tipb.items()}

    def point(self, q, finger, local):
        """远节坐标系里的点 local（m）在基座坐标系里的位置"""
        R, p = self.distal(q)[finger]
        return R @ np.asarray(local) + p

    def _cuff_points(self, finger):
        """远节上非电机零件的网格顶点（世界系，零位），以及手指方向 u 和最后一个关节锚点 a1"""
        self.d.qpos[:] = 0
        mujoco.mj_kinematics(self.m, self.d)
        mujoco.mj_forward(self.m, self.d)
        b = self.tipb[finger]
        js = gm.FINGERS[finger]
        a0, a1 = self.d.xanchor[self.jid[js[0]]], self.d.xanchor[self.jid[js[-1]]]
        u = (a1 - a0) / np.linalg.norm(a1 - a0)
        pts = []
        for g in range(self.m.ngeom):
            if self.m.geom_bodyid[g] == b and self.m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
                name = mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_MESH, self.m.geom_dataid[g]) or ""
                if MOTOR_MESH.match(name):
                    continue
                mid = self.m.geom_dataid[g]
                v = self.m.mesh_vert[self.m.mesh_vertadr[mid]: self.m.mesh_vertadr[mid] + self.m.mesh_vertnum[mid]]
                pts.append(v @ self.d.geom_xmat[g].reshape(3, 3).T + self.d.geom_xpos[g])
        return np.concatenate(pts), u, a1

    def mesh_end(self, finger, tol=0.0015):
        """指套最末端那一小片面的中心（远节坐标系）：拇指指腹去顶它时的已知点"""
        P, u, a1 = self._cuff_points(finger)
        h = (P - a1) @ u
        end = P[h > h.max() - tol].mean(axis=0)
        b = self.tipb[finger]
        R, p = self.d.xmat[b].reshape(3, 3), self.d.xpos[b]
        return R.T @ (end - p)

    def mesh_tip(self, finger, radius, palm_offset=0.0):
        """
        从 CAD 网格估计指尖球心：远节上【非电机零件】（指套 / 指尖块）沿手指方向最远的点，往回退 radius，
        再往手心方向挪 palm_offset（指腹在指套的掌侧）。返回远节坐标系里的坐标。零位计算。
        """
        self.d.qpos[:] = 0
        mujoco.mj_kinematics(self.m, self.d)
        mujoco.mj_forward(self.m, self.d)
        b = self.tipb[finger]
        js = gm.FINGERS[finger]
        a0, a1 = self.d.xanchor[self.jid[js[0]]], self.d.xanchor[self.jid[js[-1]]]
        u = (a1 - a0) / np.linalg.norm(a1 - a0)
        palm = -gm.dorsal_normal(self.m, self.d)
        pts = []
        for g in range(self.m.ngeom):
            if self.m.geom_bodyid[g] == b and self.m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
                name = mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_MESH, self.m.geom_dataid[g]) or ""
                if MOTOR_MESH.match(name):
                    continue
                mid = self.m.geom_dataid[g]
                v = self.m.mesh_vert[self.m.mesh_vertadr[mid]: self.m.mesh_vertadr[mid] + self.m.mesh_vertnum[mid]]
                pts.append(v @ self.d.geom_xmat[g].reshape(3, 3).T + self.d.geom_xpos[g])
        P = np.concatenate(pts)
        tip = P[np.argmax((P - a1) @ u)] - u * radius + palm * palm_offset
        R, p = self.d.xmat[b].reshape(3, 3), self.d.xpos[b]
        return R.T @ (tip - p)
