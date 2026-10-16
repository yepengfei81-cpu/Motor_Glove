#!/usr/bin/env python3
"""
PyBullet 查看器（替代 onshape-to-robot-bullet）：
URDF 本身表达不了闭链，自带查看器里连杆会散开、没有驱动的关节会在重力下垂落。
这里按 URDF 里的 closing_*_1 / closing_*_2 坐标系补上点约束，把回路闭合，
只给 12 个 *_motor 关节滑条，其余关节自由，重力关掉。

用法:  python tools/view_bullet.py glove            # 打开窗口
       python tools/view_bullet.py glove --test     # 无窗口自检
"""
import math
import os
import re
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
import pybullet as p


def link_index(rid):
    idx = {p.getBodyInfo(rid)[0].decode(): -1}
    for k in range(p.getNumJoints(rid)):
        idx[p.getJointInfo(rid, k)[12].decode()] = k
    return idx


def to_com_frame(rid, link, xyz):
    """URDF link 坐标系下的点 -> PyBullet 该 link 的质心坐标系（createConstraint 要求）"""
    t, q = p.getDynamicsInfo(rid, link)[3:5]
    R = np.array(p.getMatrixFromQuaternion(q)).reshape(3, 3)
    return (R.T @ (np.array(xyz) - np.array(t))).tolist()


def add_loop_constraints(rid, urdf):
    root = ET.parse(urdf).getroot()
    idx = link_index(rid)
    frames = {}
    for j in root.findall("joint"):
        child = j.find("child").get("link")
        if j.get("type") == "fixed" and child.startswith("closing_"):
            o = j.find("origin")
            xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
            frames[child] = (j.find("parent").get("link"), xyz)
    pairs = []
    for name, (parent, xyz) in frames.items():
        m = re.match(r"^(closing_.+?)_1(_z)?$", name)
        if not m:
            continue
        other = f"{m.group(1)}_2{m.group(2) or ''}"
        if other not in frames:
            continue
        pa, pb = idx[parent], idx[frames[other][0]]
        c = p.createConstraint(rid, pa, rid, pb, p.JOINT_POINT2POINT, [0, 0, 0],
                               to_com_frame(rid, pa, xyz), to_com_frame(rid, pb, frames[other][1]))
        p.changeConstraint(c, maxForce=10)
        pairs.append((name, other))
    return pairs, idx


def loop_error(rid, pairs, idx):
    errs = []
    for a, b in pairs:
        pa = np.array(p.getLinkState(rid, idx[a])[4])
        pb = np.array(p.getLinkState(rid, idx[b])[4])
        errs.append(np.linalg.norm(pa - pb))
    return max(errs) if errs else 0.0


def main():
    robot_dir = sys.argv[1]
    test = "--test" in sys.argv
    urdf = robot_dir if robot_dir.endswith(".urdf") else os.path.join(robot_dir, "robot.urdf")
    p.connect(p.DIRECT if test else p.GUI)
    p.setPhysicsEngineParameter(numSolverIterations=300, fixedTimeStep=1 / 240)
    p.setGravity(0, 0, 0)
    pose_file = os.path.join(os.path.dirname(urdf), "root_pose.json")
    base_pos, base_orn = [0, 0, 0], [0, 0, 0, 1]
    if os.path.exists(pose_file):
        import json
        pose = json.load(open(pose_file))
        w, x, y, z = pose["quat_wxyz"]
        base_pos, base_orn = pose["pos"], [x, y, z, w]
    rid = p.loadURDF(urdf, base_pos, base_orn, useFixedBase=True, flags=p.URDF_USE_INERTIA_FROM_FILE)
    pairs, idx = add_loop_constraints(rid, urdf)
    joints = {p.getJointInfo(rid, k)[1].decode(): k for k in range(p.getNumJoints(rid))
              if p.getJointInfo(rid, k)[2] == p.JOINT_REVOLUTE}
    motors = {n: k for n, k in joints.items() if n.endswith("_motor")}
    for n, k in joints.items():
        if n not in motors:
            p.setJointMotorControl2(rid, k, p.VELOCITY_CONTROL, force=0)
    print(f"* 关节 {len(joints)} 个，其中电机 {len(motors)} 个；闭环点约束 {len(pairs)} 个")

    if test:
        targets = {n: 0.0 for n in motors}
        targets["mid_mcp_motor"] = 0.5
        targets["index_pip_motor"] = 0.5
        for n, k in motors.items():
            p.setJointMotorControl2(rid, k, p.POSITION_CONTROL, targets[n], force=50)
        for _ in range(2400):
            p.stepSimulation()
        moved = {n: round(p.getJointState(rid, k)[0], 2) for n, k in joints.items() if abs(p.getJointState(rid, k)[0]) > 0.02}
        print(f"* 自检: 闭环最大误差 {loop_error(rid, pairs, idx)*1000:.1f} mm；动了的关节: {moved}")
        return

    p.resetDebugVisualizerCamera(cameraDistance=0.35, cameraYaw=50, cameraPitch=-30, cameraTargetPosition=[0, 0, 0.08])
    sliders = {n: p.addUserDebugParameter(n, -math.pi, math.pi, 0) for n in motors}
    while p.isConnected():
        for n, k in motors.items():
            p.setJointMotorControl2(rid, k, p.POSITION_CONTROL, p.readUserDebugParameter(sliders[n]), force=50)
        p.stepSimulation()
        time.sleep(1 / 240)


if __name__ == "__main__":
    main()
