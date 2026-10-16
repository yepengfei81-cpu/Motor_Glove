#!/usr/bin/env python3
"""
把 teleop_scene.xml 里的测试方块摆好（重新导出 MuJoCo 模型后会自动运行）。

手掌朝下（root_pose_mode = palm_down）时：方块立在地面上、手心正下方——
  1. 算出三指（食指、中指、无名指）握拳时指尖落到的位置，方块中心放在它们的正下方；
  2. 方块高度 = 零位时这块区域上方手套的最低点再往下留 GAP，保证零位不碰、握拳能碰到。
其他朝向时：方块悬在指尖握拳时会扫过的位置（静态物体，不受重力）。
"""
import json
import os
import re
import sys

import numpy as np
import mujoco

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import glove_model as gm  # noqa: E402

HALF_W = 0.02   # 方块半宽 (m)
GAP = 0.008     # 零位时方块顶面到手套的间隙 (m)
FLEX = 0.9      # 握拳时 MCP、PIP 电机各转多少 (rad)
FINGERS = ("index", "mid", "ring")

cfg = json.load(open(os.path.join(gm.ROOT, "glove_mujoco", "config.json")))
mode = cfg.get("root_pose_mode", "palm_front")
dirs = gm.directions()
m = mujoco.MjModel.from_xml_path(gm.SCENE)
d = mujoco.MjData(m)
cube_g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "cube")
cube_b = m.geom_bodyid[cube_g]
acts = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(m.nu)]
glove_geoms = [g for g in range(m.ngeom) if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH and m.body_rootid[m.geom_bodyid[g]] != cube_b]
tip_b = {f: m.jnt_bodyid[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, gm.TIP_JOINT[f])] for f in FINGERS}


def verts(bodies=None):
    out = []
    for g in glove_geoms:
        if bodies is not None and m.geom_bodyid[g] not in bodies:
            continue
        mid = m.geom_dataid[g]
        v = m.mesh_vert[m.mesh_vertadr[mid]: m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
        out.append(v @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g])
    return np.concatenate(out)


def settle(ctrl, contact=False):
    flags = int(m.opt.disableflags)
    m.opt.disableflags = flags if contact else flags | int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    mujoco.mj_resetData(m, d)
    d.ctrl[:] = 0
    for a, v in ctrl.items():
        d.ctrl[acts.index(a)] = v
    for _ in range(2500):
        mujoco.mj_step(m, d)
    m.opt.disableflags = flags


fist = {j: dirs[j] * FLEX for f in FINGERS for j in gm.FINGERS[f] if "_abd_" not in j}
m.body_pos[cube_b] = [5, 5, 5]                    # 先把方块移开
settle(fist)
tips = verts(set(tip_b.values()))
xy = np.mean([verts({tip_b[f]})[:, :2].mean(axis=0) for f in FINGERS], axis=0)
tip_low = tips[:, 2].min()

if mode == "palm_down":
    settle({})
    v0 = verts()
    near = v0[(np.abs(v0[:, 0] - xy[0]) < HALF_W + 0.01) & (np.abs(v0[:, 1] - xy[1]) < HALF_W + 0.01)]
    top = (near[:, 2].min() if len(near) else v0[:, 2].min()) - GAP
    half_h = top / 2
    pos = np.array([xy[0], xy[1], half_h])
    size = (HALF_W, HALF_W, half_h)
    note = f"立在地上，高 {top*100:.1f} cm；握拳时指尖最低到 {tip_low*100:.1f} cm"
else:
    pos = np.array([xy[0], xy[1], tip_low + HALF_W])
    size = (HALF_W, HALF_W, HALF_W)
    note = "悬在指尖握拳时扫过的位置"

xml = open(gm.SCENE, encoding="utf-8").read()
xml = re.sub(r'(<body name="cube" )pos="[^"]*"', lambda mm: f'{mm.group(1)}pos="{pos[0]:.4f} {pos[1]:.4f} {pos[2]:.4f}"', xml)
xml = re.sub(r'(<geom name="cube" type="box" )size="[^"]*"', lambda mm: f'{mm.group(1)}size="{size[0]:.4f} {size[1]:.4f} {size[2]:.4f}"', xml)
open(gm.SCENE, "w", encoding="utf-8").write(xml)
print(f"* 方块: {note}，中心 {np.round(pos, 3)}")
