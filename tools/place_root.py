#!/usr/bin/env python3
"""
把导出的模型摆正：Onshape 里的世界坐标系不一定是 Z 朝上，导出后模型可能倒立或陷进地面。

自动规则：用三个 MCP 侧摆电机的轴当手掌法线，用三个 PIP 关节的位置定手指方向：
    palm_front（默认）: 手指朝上(+Z)，手心朝向默认相机(-X)，像竖起手掌面对屏幕
    palm_down         : 手背板平放在地面，机构在上
然后整体抬高到最低点离地 root_clearance（默认 1 cm），XY 居中。
模式和高度在 config.json 里用 "root_pose_mode" / "root_clearance" 指定，glove 和 glove_mujoco 两个目录要写成一样。
结果写到 <目录>/root_pose.json，并改写 robot.xml 根 body 的 pos/quat；
view_bullet.py 会读取同一个文件给 URDF 用。

想手动指定时，在 config.json 里加:
    "root_pose": {"pos": [0, 0, 0.15], "quat_wxyz": [0, 1, 0, 0]}
用法:  python tools/place_root.py glove_mujoco      (或 glove，只写 root_pose.json)
"""
import json
import os
import re
import sys

import numpy as np


def rot_to_quat_wxyz(R):
    from transforms3d.quaternions import mat2quat
    return [float(v) for v in mat2quat(R)]


def auto_pose(xml_path, mode="palm_front", clearance=0.01):
    import mujoco
    # 从内存里加载：根 body 先复位成单位姿态，避免在已经摆正的文件上重复计算
    s = open(xml_path, encoding="utf-8").read()
    s = ROOT_RE.sub(lambda mm: '%spos="%s" quat="%s"%s>' % (mm.group(1), *((mm.group(5) or (mm.group(2) + ";" + mm.group(3))).split(";")), mm.group(4)), s)
    s = s.replace('meshdir="assets"', 'meshdir="%s"' % os.path.join(os.path.dirname(os.path.abspath(xml_path)), "assets"))
    m = mujoco.MjModel.from_xml_string(s)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    root = 1  # worldbody 之后的第一个 body = 基座
    geoms = [g for g in range(m.ngeom) if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH]
    base_geoms = [g for g in geoms if m.geom_bodyid[g] == root]
    g0 = base_geoms[0]
    mesh = m.geom_dataid[g0]
    v = m.mesh_vert[m.mesh_vertadr[mesh]: m.mesh_vertadr[mesh] + m.mesh_vertnum[mesh]]
    Rg = d.geom_xmat[g0].reshape(3, 3)
    vw = v @ Rg.T + d.geom_xpos[g0]
    c_base = vw.mean(axis=0)
    abd = [j for j in range(m.njnt) if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) or "").endswith("mcp_abd_motor")]
    if abd:
        axes = np.array([d.xaxis[j] for j in abd])
        axes[np.dot(axes, axes[0]) < 0] *= -1
        n = axes.mean(axis=0); n /= np.linalg.norm(n)
        how = "mcp_abd 轴"
    else:
        _, _, Vt = np.linalg.svd(vw - c_base, full_matrices=False)
        n = Vt[2]
        how = "PCA"
    if abd:
        # 手背一侧的判断固定在基座坐标系里（见 glove_model.DORSAL_REF_BASE），不随零位变化
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from glove_model import DORSAL_REF_BASE
        if np.dot(n, d.xmat[root].reshape(3, 3) @ DORSAL_REF_BASE) < 0:
            n = -n
    else:
        others = [g for g in geoms if m.geom_bodyid[g] != root]
        c_mech = d.geom_xpos[others].mean(axis=0) if others else c_base
        if np.dot(c_mech - c_base, n) < 0:
            n = -n
    # 手指方向：PIP 关节位置均值 - 基座板中心，投影到手掌平面
    pips = [j for j in range(m.njnt) if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) or "").endswith("_pip")]
    f = (np.mean([d.xanchor[j] for j in pips], axis=0) - c_base) if pips else np.array([1.0, 0, 0])
    f = f - np.dot(f, n) * n
    f /= np.linalg.norm(f)
    if mode == "palm_down":
        # n -> +Z，手指 -> +X
        R = np.array([f, np.cross(n, f), n])
    else:
        # palm_front: 手指 -> +Z，手背法线 n -> +X（手心朝 -X，正对默认相机）
        R = np.array([n, np.cross(f, n), f])
    how += f", {mode}"
    # 用网格顶点（而不是包围球）算最低点，离地高度才准确
    verts = []
    for g in geoms:
        mid = m.geom_dataid[g]
        v = m.mesh_vert[m.mesh_vertadr[mid]: m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
        verts.append(v @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g])
    pos_all = np.concatenate(verts) @ R.T
    zmin = float(pos_all[:, 2].min())
    center = (pos_all.min(axis=0) + pos_all.max(axis=0)) / 2
    t = np.array([-center[0], -center[1], clearance - zmin])
    return {"pos": [float(x) for x in t], "quat_wxyz": rot_to_quat_wxyz(R), "source": f"auto({how})"}


ROOT_RE = re.compile(r'^(    <body name="[^"]+" )pos="([^"]*)" quat="([^"]*)"([^>]*)>(?:<!-- orig_pose: ([^-]*?) -->)?', re.M)


def apply_to_mjcf(xml_path, pose):
    """根 body 是 worldbody 的直接子节点（4 个空格缩进）。原始姿态记在行尾注释里，重复运行不会叠加。"""
    from transforms3d.quaternions import quat2mat, mat2quat, qmult
    s = open(xml_path, encoding="utf-8").read()
    R = quat2mat(pose["quat_wxyz"]); t = np.array(pose["pos"])
    count = [0]

    def repl(mm):
        orig = mm.group(5) or (mm.group(2) + ";" + mm.group(3))
        p0 = np.array([float(v) for v in orig.split(";")[0].split()])
        q0 = np.array([float(v) for v in orig.split(";")[1].split()])
        p1 = R @ p0 + t
        q1 = qmult(mat2quat(R), q0)
        count[0] += 1
        return '%spos="%s" quat="%s"%s><!-- orig_pose: %s -->' % (mm.group(1), "%g %g %g" % tuple(p1), "%g %g %g %g" % tuple(q1), mm.group(4), orig)

    s, n = ROOT_RE.subn(repl, s)
    if n < 1:
        raise SystemExit("没找到根 body")
    open(xml_path, "w", encoding="utf-8").write(s)


def main():
    d = sys.argv[1]
    cfg = json.load(open(os.path.join(d, "config.json"), encoding="utf-8"))
    xml = os.path.join(d, "robot.xml")
    mj_dir = d if os.path.exists(xml) else None
    if "root_pose" in cfg:
        pose = dict(cfg["root_pose"]); pose["source"] = "config.json"
    else:
        src = xml if mj_dir else os.path.join(os.path.dirname(d.rstrip("/")) or ".", os.path.basename(d.rstrip("/")) + "_mujoco", "robot.xml")
        if not os.path.exists(src):
            raise SystemExit("需要先导出 MuJoCo 模型才能自动计算姿态，或在 config.json 里写 root_pose")
        pose = auto_pose(src, cfg.get("root_pose_mode", "palm_front"), cfg.get("root_clearance", 0.01))
    json.dump(pose, open(os.path.join(d, "root_pose.json"), "w"), indent=1)
    if mj_dir:
        apply_to_mjcf(xml, pose)
    print(f"* root pose ({pose['source']}): pos={np.round(pose['pos'], 3).tolist()} quat_wxyz={np.round(pose['quat_wxyz'], 3).tolist()} -> {d}/root_pose.json")


if __name__ == "__main__":
    main()
