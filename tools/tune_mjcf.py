#!/usr/bin/env python3
"""
MuJoCo 模型导出后的修正（export_glove.py 会自动调用，也可以单独运行）:

  1. 远端关节（三指 PIP、拇指 IP）加机械限位：伸直方向 -40°（标定零位可能是微弯的，完全伸直会读成负角度），弯曲方向 110°。
     不加的话，方块把指尖往后顶时，四连杆会被顶过死点翻到另一个分支（指节整个翻过来），
     而且翻转附近会来回跳，传到手套上就是抖动。
  2. 闭环约束（equality connect）收紧：solref 时间常数 20 ms -> 5 ms，减小被外力拉开的程度。
  3. 积分器用 implicitfast：位置执行器的阻尼按隐式处理，目标角度突变、窗口里鼠标拖拽都不会数值发散
     （默认的 Euler 在关节惯量很小时会发散）。

  python tools/tune_mjcf.py glove_mujoco
"""
import math
import os
import re
import sys

import mujoco

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import glove_model as gm  # noqa: E402

DISTAL = {"ring_pip": "ring_pip_motor", "index_pip": "index_pip_motor", "mid_pip": "mid_pip_motor", "thumb_ip": "thumb_ip_motor"}
RANGE_DEG = (-40, 110)   # 伸直方向留够余量（手套完全伸直可能比标定零位还往后 20~30°），离连杆死点仍很远
EQ_SOLREF = "0.005 1"


def joint_re(name):
    return re.compile(r'(<joint axis="[^"]*" name="%s" type="hinge")(?: range="[^"]*")?(/>)' % name)


def main(robot_dir):
    path = os.path.join(robot_dir, "robot.xml")
    s = open(path, encoding="utf-8").read()
    for j in DISTAL:                                   # 先去掉旧的限位，再按当前模型重新算
        s = joint_re(j).sub(r"\1\2", s)
    open(path, "w", encoding="utf-8").write(s)

    scene = os.path.join(robot_dir, "teleop_scene.xml")
    m = mujoco.MjModel.from_xml_path(scene if os.path.exists(scene) else path)
    present = {j: mt for j, mt in DISTAL.items() if mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j) >= 0}
    if not present:                                    # 没有连杆被动关节（例如直驱的机械手）：只加积分器
        if '<option integrator="implicitfast"/>' not in s:
            s = re.sub(r'(<compiler [^>]*/>)', r'\1\n  <option integrator="implicitfast"/>', s, count=1)
        open(path, "w", encoding="utf-8").write(s)
        print("* 没有连杆被动关节，跳过远端限位；积分器 implicitfast")
        return
    model_dir = gm.directions(refresh=True)
    d = mujoco.MjData(m)
    m.opt.disableflags = int(m.opt.disableflags) | int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    act = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i): i for i in range(m.nu)}
    ranges = {}
    for j, motor in present.items():
        mujoco.mj_resetData(m, d)
        d.ctrl[act[motor]] = model_dir[motor] * 0.3       # 电机往"弯曲"转一点，看远端关节往哪边变
        for _ in range(1000):
            mujoco.mj_step(m, d)
        flex = 1 if d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)]] > 0 else -1
        lo, hi = sorted(flex * math.radians(a) for a in RANGE_DEG)
        ranges[j] = (lo, hi)
        s = joint_re(j).sub(lambda mm: f'{mm.group(1)} range="{lo:.4f} {hi:.4f}"{mm.group(2)}', s)
    s = re.sub(r'<connect site1="([^"]*)" site2="([^"]*)"(?: solref="[^"]*")?/>', rf'<connect site1="\1" site2="\2" solref="{EQ_SOLREF}"/>', s)
    if '<option integrator="implicitfast"/>' not in s:
        s = re.sub(r'(<compiler [^>]*/>)', r'\1\n  <option integrator="implicitfast"/>', s, count=1)
    open(path, "w", encoding="utf-8").write(s)
    print("* 远端关节限位: " + ", ".join(f"{j} [{math.degrees(a):.0f}°, {math.degrees(b):.0f}°]" for j, (a, b) in ranges.items())
          + f"；闭环约束 solref {EQ_SOLREF}；积分器 implicitfast")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else os.path.join(gm.ROOT, "glove_mujoco"))
