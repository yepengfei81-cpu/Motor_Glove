"""
骨架图的公共部分：建模型（白底、手掌朝前）、算骨架（骨头 / 关节 / 转轴）、把骨架画进任意 MjvScene、
离屏渲染成 PNG。fig_skeleton.py（命令行）和 skeleton_gui.py（交互界面）都用它。

state 字典（界面上的所有可调项）:
  angles_deg: {关节名: 角度}，正 = 握拳 / 向拇指侧      bone_mm, joint_mm, axis_mm: 粗细和长度 (mm)
  mesh_alpha: 0~1                                          show_mesh/show_bones/show_axes/show_labels
  cam: {azimuth, elevation, distance, lookat}
"""
import json
import math
import os
import sys

import numpy as np
import mujoco

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import glove_model as gm  # noqa: E402
from figtext import T  # noqa: E402

RED, BLUE, GREY = (0.88, 0.18, 0.12, 1), (0.12, 0.42, 0.92, 1), (0.45, 0.45, 0.50, 1)
BONE, JOINT, MESH = (0.80, 0.68, 0.42, 1), (0.25, 0.25, 0.30, 1), (0.45, 0.50, 0.60)
FONT = next((f for f in (("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc")
                          if os.environ.get("FIG_LANG", "en") == "en" else
                          ("/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc", "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf")) if os.path.exists(f)), None)
DEFAULT_STATE = dict(angles_deg={j: 0.0 for js in gm.FINGERS.values() for j in js}, bone_mm=4.5, joint_mm=7.0, axis_mm=45.0,
                     mesh_alpha=0.22, show_mesh=True, show_bones=True, show_axes=True, show_labels=True,
                     cam=dict(azimuth=215.0, elevation=-25.0, distance=None, lookat=None))


def build_model(W=2000, H=1500):
    """手掌朝前、白底、无重力、关掉接触的模型（关节角由位置执行器给）"""
    import place_root
    spec = mujoco.MjSpec.from_file(gm.MJCF)
    pose = place_root.auto_pose(gm.MJCF, "palm_front", 0.0)
    root = spec.body("part_1")
    root.pos, root.quat = pose["pos"], pose["quat_wxyz"]
    spec.visual.headlight.ambient[:] = [0.55, 0.55, 0.55]
    spec.visual.headlight.diffuse[:] = [0.5, 0.5, 0.5]
    spec.visual.headlight.specular[:] = [0.05, 0.05, 0.05]
    spec.visual.global_.offwidth, spec.visual.global_.offheight = W, H
    spec.add_texture(name="white", type=mujoco.mjtTexture.mjTEXTURE_SKYBOX, builtin=mujoco.mjtBuiltin.mjBUILTIN_FLAT,
                     rgb1=[1, 1, 1], rgb2=[1, 1, 1], width=32, height=32)
    m = spec.compile()
    m.opt.disableflags = int(m.opt.disableflags) | int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    m.opt.gravity[:] = 0
    for g in range(m.ngeom):
        if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH and m.geom_group[g] == 3:
            m.geom_rgba[g, 3] = 0                     # collision 副本永远不画
    return m


def apply_mesh_alpha(m, alpha):
    for g in range(m.ngeom):
        if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH and m.geom_group[g] != 3:
            m.geom_rgba[g] = [*MESH, alpha]


def set_pose(m, d, angles_deg, md, settle_steps=1500):
    """按"握拳为正"的角度设置 13 个电机关节，仿真几步让连杆跟上"""
    for j, v in angles_deg.items():
        d.ctrl[m.actuator(j).id] = md[j] * math.radians(v)
    for _ in range(settle_steps):
        mujoco.mj_step(m, d)
    mujoco.mj_forward(m, d)


def fit_camera(m, d):
    meshes = [g for g in range(m.ngeom) if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH]
    lo = (d.geom_xpos[meshes] - m.geom_rbound[meshes, None]).min(0)
    hi = (d.geom_xpos[meshes] + m.geom_rbound[meshes, None]).max(0)
    center, extent = (lo + hi) / 2, float(np.max(hi - lo))
    return center, 0.62 * extent / math.tan(math.radians(m.vis.global_.fovy) / 2)


def tip_point(m, d, finger, direction=None):
    """
    指尖：远节网格上沿手指方向最远的点。direction = 近节方向（MCP→PIP），没给就用离铰链最远的点。
    远节上还装着 PIP 电机，单纯取"离铰链最远"会取到电机的角，所以要沿手指方向取。
    """
    b = m.jnt_bodyid[m.joint(gm.TIP_JOINT[finger]).id]
    anchor = d.xanchor[m.joint(gm.TIP_JOINT[finger]).id]
    pts = []
    for g in range(m.ngeom):
        if m.geom_bodyid[g] == b and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
            mid = m.geom_dataid[g]
            v = m.mesh_vert[m.mesh_vertadr[mid]: m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
            pts.append(v @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g])
    p = np.concatenate(pts)
    if direction is None:
        return p[np.argmax(np.linalg.norm(p - anchor, axis=1))]
    u = np.asarray(direction) / np.linalg.norm(direction)
    return p[np.argmax((p - anchor) @ u)]


def _closest_point(a1, z1, a2, z2):
    """两条直线最近点的中点"""
    w = a1 - a2
    b, dd, e = np.dot(z1, z2), np.dot(z1, w), np.dot(z2, w)
    den = 1 - b * b
    if abs(den) < 1e-6:
        return (a1 + a2 - np.dot(w, z1) * z1) / 2
    t1, t2 = (b * e - dd) / den, (e - b * dd) / den
    return ((a1 + t1 * z1) + (a2 + t2 * z2)) / 2


def _proj(c, a, z):
    """点 c 在直线 (a, z) 上的投影"""
    return a + np.dot(c - a, z) * z


class Anatomy:
    """
    把电机关节换算成人手解剖关节：每根手指一串 (关节名, 中心点, 弯曲轴方向, 侧摆轴方向, 弯曲电机ID, 侧摆电机ID)，
    骨头沿手指中线。中指 MCP 是双平行四边形的虚拟转心，转心在输出连杆坐标系里是固定的，算一次就够。
    """

    def __init__(self, m, md):
        self.md = md
        self.rcm_local, self.rcm_body, self.rcm_sign = None, None, 1
        abd_j = m.joint("mid_mcp_abd_motor").id
        self.rcm_body = int(m.body_parentid[m.jnt_bodyid[abd_j]])        # 平行四边形的输出连杆
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)
        for _ in range(1500):
            mujoco.mj_step(m, d)
        R1, t1 = d.xmat[self.rcm_body].reshape(3, 3).copy(), d.xpos[self.rcm_body].copy()
        d.ctrl[m.actuator("mid_mcp_motor").id] = md["mid_mcp_motor"] * 0.25   # 往握拳方向转一点
        for _ in range(1500):
            mujoco.mj_step(m, d)
        R2, t2 = d.xmat[self.rcm_body].reshape(3, 3).copy(), d.xpos[self.rcm_body].copy()
        x = np.linalg.lstsq(R1 - R2, t2 - t1, rcond=None)[0]              # 两个姿态下世界位置相同的点
        self.rcm_local = x
        rotvec = np.zeros(3)
        q = np.zeros(4)
        mujoco.mju_mat2Quat(q, (R2 @ R1.T).ravel())
        mujoco.mju_quat2Vel(rotvec, q, 1.0)
        axis = d.xaxis[m.joint("mid_mcp_motor").id]
        self.rcm_sign = 1 if np.dot(rotvec, axis) >= 0 else -1            # 输出连杆往握拳方向转时绕电机轴的正负

    def fingers(self, m, d):
        md = self.md
        out = {}
        for f, js in gm.FINGERS.items():
            J = lambda n: m.joint(n).id
            A = lambda n: d.xanchor[J(n)]
            Z = lambda n: d.xaxis[J(n)]
            chain = []
            if f == "thumb":
                abd, flex, mcp, ipm = js
                c = _closest_point(A(abd), Z(abd), A(flex), Z(flex))
                chain.append(("CMC", c, Z(flex) * md[flex], Z(abd) * md[abd], gm.DEFAULT_IDS[flex], gm.DEFAULT_IDS[abd]))
                c = _proj(c, A(mcp), Z(mcp))
                chain.append(("MCP", c, Z(mcp) * md[mcp], None, gm.DEFAULT_IDS[mcp], None))
                h = gm.TIP_JOINT[f]
                c = _proj(c, A(h), Z(h))
                zd = Z(h) * (1 if np.dot(Z(h), chain[-1][2]) >= 0 else -1)
                chain.append(("IP", c, zd, None, gm.DEFAULT_IDS[ipm], None))
            else:
                flex, abd, pipm = js
                if f == "mid":
                    R, t = d.xmat[self.rcm_body].reshape(3, 3), d.xpos[self.rcm_body]
                    rc = R @ self.rcm_local + t
                    zf = Z(flex) * md[flex] * self.rcm_sign
                    c = _closest_point(rc, zf, A(abd), Z(abd))
                else:
                    zf = Z(flex) * md[flex]
                    c = _closest_point(A(flex), Z(flex), A(abd), Z(abd))
                chain.append(("MCP", c, zf, Z(abd) * md[abd], gm.DEFAULT_IDS[flex], gm.DEFAULT_IDS[abd]))
                h = gm.TIP_JOINT[f]
                c = _proj(c, A(h), Z(h))
                zd = Z(h) * (1 if np.dot(Z(h), zf) >= 0 else -1)
                chain.append(("PIP", c, zd, None, gm.DEFAULT_IDS[pipm], None))
            last, prev = chain[-1], chain[-2]
            tip = tip_point(m, d, f, last[1] - prev[1])
            # 指尖落到手指中线上：去掉沿关节轴（侧向）的偏移
            tip = last[1] + (tip - last[1]) - np.dot(tip - last[1], last[2]) * last[2]
            out[f] = dict(chain=chain, tip=tip)
        # 手腕：三指 MCP 中心往近端退 1.4 个近节长度
        mcps = np.array([out[f]["chain"][0][1] for f in ("index", "mid", "ring")])
        pips = np.array([out[f]["chain"][1][1] for f in ("index", "mid", "ring")])
        u = (pips - mcps).mean(axis=0)
        out["wrist"] = mcps.mean(axis=0) - 1.4 * u
        return out


def _geom(scn, gtype, size, pos, rgba, label=""):
    g = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(g, gtype, np.asarray(size, float), np.asarray(pos, float), np.eye(3).ravel(), np.asarray(rgba, np.float32))
    g.label = label
    scn.ngeom += 1


def _line(scn, a, b, width, rgba, gtype=mujoco.mjtGeom.mjGEOM_CAPSULE, label=""):
    g = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(g, gtype, np.zeros(3), np.zeros(3), np.eye(3).ravel(), np.asarray(rgba, np.float32))
    mujoco.mjv_connector(g, gtype, width, np.asarray(a, float), np.asarray(b, float))
    g.label = label
    scn.ngeom += 1


def draw_skeleton(scn, m, d, state, md, anatomy):
    """把骨架画进 scn（界面的 user_scn 或离屏渲染的 scene）。返回标签列表 [(世界坐标, 文字, 颜色)]"""
    labels = []
    bone_r, joint_r, axis_len = state["bone_mm"] / 1000, state["joint_mm"] / 1000, state["axis_mm"] / 1000
    an = anatomy.fingers(m, d)
    if state["show_bones"]:
        for f in gm.FINGERS:
            pts = [an["wrist"]] + [c[1] for c in an[f]["chain"]] + [an[f]["tip"]]
            for a, b in zip(pts[:-1], pts[1:]):
                if np.linalg.norm(b - a) > 1e-4:
                    _line(scn, a, b, bone_r, BONE)
            for c in an[f]["chain"]:
                _geom(scn, mujoco.mjtGeom.mjGEOM_SPHERE, [joint_r, 0, 0], c[1], JOINT)
        _geom(scn, mujoco.mjtGeom.mjGEOM_SPHERE, [joint_r, 0, 0], an["wrist"], JOINT)
    if state["show_axes"]:
        for f in gm.FINGERS:
            for name, c, zf, za, id_f, id_a in an[f]["chain"]:
                _line(scn, c - zf * axis_len / 2, c + zf * axis_len / 2, bone_r, RED, mujoco.mjtGeom.mjGEOM_ARROW)
                labels.append((c + zf * axis_len / 2, str(id_f), RED))
                if za is not None:
                    _line(scn, c - za * axis_len / 2, c + za * axis_len / 2, bone_r, BLUE, mujoco.mjtGeom.mjGEOM_ARROW)
                    labels.append((c - za * axis_len / 2, str(id_a), BLUE))
    return labels if state["show_labels"] else []


def project(m, cam, W, H, p):
    az, el = math.radians(cam["azimuth"]), math.radians(cam["elevation"])
    forward = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
    eye = np.asarray(cam["lookat"]) - cam["distance"] * forward
    right = np.cross(forward, [0, 0, 1.0]); right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    rel = np.asarray(p) - eye
    x, y, z = np.dot(rel, right), np.dot(rel, up), np.dot(rel, forward)
    f = (H / 2) / math.tan(math.radians(m.vis.global_.fovy) / 2)
    return W / 2 + f * x / z, H / 2 - f * y / z


def render_png(state, out, W=2000, H=1500, legend=True):
    """按 state 离屏渲染一张图（在子进程里用 MUJOCO_GL=egl 跑最稳）"""
    from PIL import Image, ImageDraw, ImageFont
    md = gm.directions()
    m = build_model(W, H)
    d = mujoco.MjData(m)
    apply_mesh_alpha(m, state["mesh_alpha"] if state["show_mesh"] else 0.0)
    set_pose(m, d, state["angles_deg"], md)
    cam = dict(state["cam"])
    center, dist = fit_camera(m, d)
    if cam.get("lookat") is None:
        cam["lookat"] = center.tolist()
    if cam.get("distance") is None:
        cam["distance"] = dist
    r = mujoco.Renderer(m, H, W)
    mc = mujoco.MjvCamera()
    mc.azimuth, mc.elevation, mc.distance = cam["azimuth"], cam["elevation"], cam["distance"]
    mc.lookat[:] = cam["lookat"]
    r.update_scene(d, mc, mujoco.MjvOption())
    labels = draw_skeleton(r.scene, m, d, state, md, Anatomy(m, md))
    im = Image.fromarray(r.render())
    dr = ImageDraw.Draw(im)
    font = ImageFont.truetype(FONT, int(H * 0.023)) if FONT else ImageFont.load_default()
    rad = int(H * 0.016)
    for p, txt, col in labels:
        x, y = project(m, cam, W, H, p)
        rgb = tuple(int(c * 255) for c in col[:3])
        tw, th = dr.textbbox((0, 0), txt, font=font)[2:]
        dr.ellipse([x - rad, y - rad, x + rad, y + rad], fill=(255, 255, 255), outline=rgb, width=3)
        dr.text((x - tw / 2, y - th / 2 - rad * 0.15), txt, fill=rgb, font=font)
    if legend:
        lx, ly = int(W * 0.02), int(H * 0.87)
        items = [(RED, T("legend_flex")), (BLUE, T("legend_abd"))] if state["show_axes"] else []
        items += [(BONE, T("legend_bones"))] if state["show_bones"] else []
        for col, txt in items:
            dr.rectangle([lx, ly, lx + 36, ly + 14], fill=tuple(int(c * 255) for c in col[:3]))
            dr.text((lx + 50, ly - 12), txt, fill=(30, 30, 30), font=font)
            ly += int(H * 0.028)
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    im.save(out)
    return out


def load_state(path):
    s = json.load(open(path))
    st = json.loads(json.dumps(DEFAULT_STATE))
    st.update({k: v for k, v in s.items() if k != "angles_deg"})
    st["angles_deg"].update(s.get("angles_deg", {}))
    return st
