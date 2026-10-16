#!/usr/bin/env python3
"""
给导出结果补质量/惯量（Onshape 里没有指定材料时，导出的质量全是 1e-9，仿真会发散）。

  URDF : 用每个 link 的 STL 体积 × 密度 计算质量、质心、惯量，写回 <inertial>
  MJCF : 让 MuJoCo 自己按几何体算（compiler inertiafromgeom="true"，visual 几何体密度设 0）

密度默认 1200 kg/m^3（PLA/ABS 量级）。这只是近似，之后在 Onshape 里指定材料后就不再需要。

用法:  python tools/fill_inertia.py glove            # 处理 glove/robot.urdf
       python tools/fill_inertia.py glove_mujoco     # 处理 glove_mujoco/robot.xml
"""
import os
import re
import sys
import xml.etree.ElementTree as ET

import numpy as np
from stl import mesh as stlmesh
from transforms3d.euler import euler2mat

DENSITY = 1200.0  # kg/m^3
MIN_MASS = 0.001  # kg
JOINT_FRICTION = 0.002   # N·m
JOINT_ARMATURE = 0.001   # kg·m²（再小的话，目标角度突变或鼠标拖一下就会数值发散）


def mesh_props(path):
    """返回 (体积, 质心, 单位密度下绕质心的惯量矩阵)，网格不封闭时体积可能为负或 0"""
    m = stlmesh.Mesh.from_file(path)
    vol, cog, inertia = m.get_mass_properties()
    return float(vol), np.asarray(cog, dtype=float), np.asarray(inertia, dtype=float)


def fill_urdf(path):
    tree = ET.parse(path)
    root = tree.getroot()
    base = os.path.dirname(path)
    n_fixed = 0
    for link in root.findall("link"):
        M, first_moment, I_origin = 0.0, np.zeros(3), np.zeros((3, 3))
        for vis in link.findall("visual"):
            mesh_el = vis.find("geometry/mesh")
            if mesh_el is None:
                continue
            fn = re.sub(r"^package://", "", mesh_el.get("filename"))
            fn = os.path.join(base, fn)
            if not os.path.exists(fn):
                continue
            vol, cog, I_c = mesh_props(fn)
            if vol <= 0:
                continue
            origin = vis.find("origin")
            xyz = np.array([float(v) for v in (origin.get("xyz", "0 0 0") if origin is not None else "0 0 0").split()])
            rpy = np.array([float(v) for v in (origin.get("rpy", "0 0 0") if origin is not None else "0 0 0").split()])
            R = euler2mat(*rpy, axes="sxyz")
            m_i = DENSITY * vol
            c_i = R @ cog + xyz                       # 质心到 link 坐标系
            I_i = DENSITY * (R @ I_c @ R.T)           # 绕质心的惯量，转到 link 坐标系
            # 平行轴定理：移到 link 原点
            I_i = I_i + m_i * (np.dot(c_i, c_i) * np.eye(3) - np.outer(c_i, c_i))
            M += m_i
            first_moment += m_i * c_i
            I_origin += I_i
        if M < MIN_MASS:
            M = MIN_MASS
            com = np.zeros(3)
            I_com = np.eye(3) * 1e-7
        else:
            com = first_moment / M
            I_com = I_origin - M * (np.dot(com, com) * np.eye(3) - np.outer(com, com))
            # 保证正定
            I_com = 0.5 * (I_com + I_com.T)
            w, V = np.linalg.eigh(I_com)
            I_com = V @ np.diag(np.maximum(w, 1e-9)) @ V.T
        inertial = link.find("inertial")
        if inertial is None:
            inertial = ET.SubElement(link, "inertial")
        for child in list(inertial):
            inertial.remove(child)
        ET.SubElement(inertial, "origin", xyz="%g %g %g" % tuple(com), rpy="0 0 0")
        ET.SubElement(inertial, "mass", value="%g" % M)
        ET.SubElement(inertial, "inertia", ixx="%g" % I_com[0, 0], ixy="%g" % I_com[0, 1], ixz="%g" % I_com[0, 2],
                      iyy="%g" % I_com[1, 1], iyz="%g" % I_com[1, 2], izz="%g" % I_com[2, 2])
        n_fixed += 1
    tree.write(path, encoding="utf-8", xml_declaration=True)
    total = sum(float(l.find("inertial/mass").get("value")) for l in root.findall("link"))
    print(f"* URDF: {n_fixed} 个 link 已按网格体积补质量，总质量 {total*1000:.0f} g")


def fill_mjcf(path):
    s = open(path, encoding="utf-8").read()
    if 'inertiafromgeom="true"' not in s:
        s = s.replace("<compiler ", '<compiler inertiafromgeom="true" ', 1)
    s = re.sub(r'(<default class="visual">\s*<geom type="mesh")(?![^>]*density=)', r'\1 density="0"', s)
    s = re.sub(r'(<default class="collision">\s*<geom)(?![^>]*density=)', r'\1 density="%g"' % DENSITY, s)
    # 手套自身零件互相穿插，不做自碰撞：contype=1/conaffinity=2 的几何体之间不碰，只和 contype=2 或 conaffinity=1 的外物碰
    s = re.sub(r'(<default class="collision">\s*<geom)(?![^>]*contype=)', r'\1 contype="1" conaffinity="2"', s)
    # onshape-to-robot 默认每个关节 frictionloss 0.1 N·m、armature 0.005，对手指关节太大：
    # PIP 连杆串了几个关节，摩擦加起来超过 XL330-M077 的力矩，虚拟手指会卡住不动
    s = re.sub(r'<joint frictionloss="[^"]*" armature="[^"]*"/>', f'<joint frictionloss="{JOINT_FRICTION}" armature="{JOINT_ARMATURE}"/>', s)
    open(path, "w", encoding="utf-8").write(s)
    print(f"* MJCF: 关节摩擦 {JOINT_FRICTION} N·m、armature {JOINT_ARMATURE}；inertiafromgeom=true，visual 密度 0，collision 密度 %g，自碰撞关闭(contype=1 conaffinity=2)" % DENSITY)


if __name__ == "__main__":
    d = sys.argv[1]
    if os.path.exists(os.path.join(d, "robot.urdf")):
        fill_urdf(os.path.join(d, "robot.urdf"))
    if os.path.exists(os.path.join(d, "robot.xml")):
        fill_mjcf(os.path.join(d, "robot.xml"))
