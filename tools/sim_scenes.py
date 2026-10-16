"""
运行时构建 MuJoCo 场景（不改动磁盘上的模型文件）：

  view_scene()   手掌立起来、朝前（手指朝上），没有方块、关掉接触，随便动
  grasp_scene()  手掌朝下，手可以整体上下移动（mocap），方块是能被抓起来的自由物体，
                 方块放在"你捏东西时拇指和食指之间"的位置

都基于 glove_mujoco/teleop_scene.xml（里面 include 了导出的 robot.xml）。
"""
import json
import math
import os
import sys

import numpy as np
import mujoco

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import glove_model as gm  # noqa: E402

ROOT_BODY = "part_1"
CONTACT = int(mujoco.mjtDisableBit.mjDSBL_CONTACT)


def _spec():
    return mujoco.MjSpec.from_file(gm.SCENE)


def view_scene(height=0.05):
    """手掌立起来朝前；返回 (model, 相机参数)"""
    import place_root
    spec = _spec()
    spec.delete(spec.body("cube"))
    pose = place_root.auto_pose(gm.MJCF, "palm_front", height)
    root = spec.body(ROOT_BODY)
    root.pos, root.quat = pose["pos"], pose["quat_wxyz"]
    m = spec.compile()
    m.opt.disableflags = int(m.opt.disableflags) | CONTACT
    # 手背朝 +X；相机放在 +X 一侧往回看，看到的就是"举起手、手心朝前"时你自己看到的样子
    cam = {"azimuth": 180.0, "elevation": -10.0, "distance": 0.45, "lookat": [0.0, 0.0, height + 0.12]}
    return m, cam


def _tip_low(m, d, finger):
    """远节网格上最低的那个点（手掌朝下时就是指尖）"""
    b = m.jnt_bodyid[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, gm.TIP_JOINT[finger])]
    pts = []
    for g in range(m.ngeom):
        if m.geom_bodyid[g] == b and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
            mid = m.geom_dataid[g]
            v = m.mesh_vert[m.mesh_vertadr[mid]: m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
            pts.append(v @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g])
    p = np.concatenate(pts)
    return p[p[:, 2].argmin()]


def _palm_low(m, d):
    """基座（手掌）网格最低点的高度"""
    zs = []
    for g in range(m.ngeom):
        if m.geom_bodyid[g] == 1 and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
            mid = m.geom_dataid[g]
            v = m.mesh_vert[m.mesh_vertadr[mid]: m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
            zs.append((v @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g])[:, 2].min())
    return float(min(zs))


def default_cube_xy(width=0.04):
    """
    积木默认位置：零位时拇指尖和食指尖连线的中点，如果那里会碰到手，就沿"拇指 -> 食指"方向往外挪，
    直到零位时不接触（每次 5 mm，最多 6 cm）。
    """
    m = mujoco.MjModel.from_xml_path(gm.SCENE)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    thumb, index = _tip_low(m, d, "thumb"), _tip_low(m, d, "index")
    p = (thumb + index) / 2
    step = (index - thumb)[:2]
    step = step / np.linalg.norm(step) * 0.005
    gm_, info = grasp_scene(float(p[0]), float(p[1]), width=width)
    gd = mujoco.MjData(gm_)
    x, y = float(p[0]), float(p[1])
    for _ in range(12):
        place_cube(gm_, gd, info, x, y, 0.0)
        mujoco.mj_forward(gm_, gd)
        if not contacts_on_cube(gm_, gd, info):
            break
        x, y = x + float(step[0]), y + float(step[1])
    return x, y


def pinch_from_angles(q, fingers=("index",)):
    """
    给定一组电机关节角（你做"捏着积木"手势时手套的读数），在手掌朝下的模型里算积木该放哪：
      center  拇指尖和食指尖的中点
      yaw     两指尖连线的水平方向（积木转到让两个侧面正对这条线，才夹得住）
      gap     两指尖距离
      palm_z  手掌底面高度（积木顶面要低于它）
    """
    m = mujoco.MjModel.from_xml_path(gm.SCENE)
    d = mujoco.MjData(m)
    m.opt.disableflags = int(m.opt.disableflags) | CONTACT
    for name, v in q.items():
        d.ctrl[m.actuator(name).id] = v
    for _ in range(1500):
        mujoco.mj_step(m, d)
    thumb = _tip_low(m, d, "thumb")
    other = np.mean([_tip_low(m, d, f) for f in fingers], axis=0)
    axis = other - thumb
    return {"center": (thumb + other) / 2, "yaw": float(math.atan2(axis[1], axis[0])),
            "gap": float(np.linalg.norm(axis)), "palm_z": _palm_low(m, d)}


def grasp_scene(x, y, yaw=0.0, width=0.04, lift=0.08, density=300, friction=1.0):
    """
    手保持原来的高度（手掌朝下）不动；width 见方的积木放在地上 (x, y)，绕竖直轴转 yaw（弧度）。
    另外加一个"抓取辅助"焊接约束（默认不生效），由 HandLift 在你捏住时打开、松手时关闭。返回 (model, info)。
    """
    spec = _spec()
    spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC      # 摩擦锥更准；impratio 保持默认 1（调大会让表面"粘"）
    root = spec.body(ROOT_BODY)
    root.mocap = True                                       # 手可以整体抬起来（不受重力）
    half = width / 2
    cube = spec.body("cube")
    cube.pos = [x, y, half + 0.0005]
    cube.quat = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
    cube.add_freejoint()
    g = cube.geoms[0]
    g.size = [half, half, half]
    g.friction = [friction, 0.02, 0.002]
    g.condim = 4
    g.density = density                                     # 4 cm、300 kg/m³ 约 19 g
    weld = spec.add_equality()
    weld.name, weld.type, weld.objtype = "grasp_assist", mujoco.mjtEq.mjEQ_WELD, mujoco.mjtObj.mjOBJ_BODY
    weld.name1, weld.name2, weld.active = "cube", ROOT_BODY, False
    m = spec.compile()
    cube_id = m.body("cube").id
    info = {"root_pos": np.array(m.body_pos[m.body(ROOT_BODY).id]), "base_offset": np.zeros(3), "lift": lift,
            "cube_body": cube_id, "cube_geom": m.geom("cube").id, "half": half, "size": (width, width),
            "weld": m.equality("grasp_assist").id, "hand_body": m.body(ROOT_BODY).id,
            "cube_qadr": m.jnt_qposadr[m.body_jntadr[cube_id]], "cube_vadr": m.jnt_dofadr[m.body_jntadr[cube_id]],
            "density": density}
    return m, info


def set_cube_size(m, d, info, width):
    """运行中改积木边长（米）：几何、碰撞包围盒、质量、惯量一起改，不用重新编译"""
    g, b, h = info["cube_geom"], info["cube_body"], width / 2
    m.geom_size[g] = [h, h, h]
    m.geom_rbound[g] = math.sqrt(3) * h
    m.geom_aabb[g] = [0, 0, 0, h, h, h]
    mass = info["density"] * width ** 3
    m.body_mass[b] = mass
    m.body_inertia[b] = [mass * width ** 2 / 6] * 3
    mujoco.mj_setConst(m, d)
    info["half"], info["size"] = h, (width, width)


def set_cube_friction(m, info, mu):
    m.geom_friction[info["cube_geom"], 0] = mu


def place_cube(m, d, info, x, y, yaw):
    """把积木直接放到地上 (x, y)、转角 yaw（弧度），速度清零"""
    a, v = info["cube_qadr"], info["cube_vadr"]
    d.qpos[a:a + 3] = [x, y, info["half"] + 0.0005]
    d.qpos[a + 3:a + 7] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
    d.qvel[v:v + 6] = 0


def finger_of_body(m):
    """{body id: 手指名}：每根手指第一个电机关节之后的所有 body 都算这根手指"""
    out = {}
    for f, joints in gm.FINGERS.items():
        first = m.jnt_bodyid[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, joints[0])]
        for bb in range(m.nbody):
            p = bb
            while p > 0:
                if p == first:
                    out[bb] = f
                    break
                p = m.body_parentid[p]
    return out


_FOB = {}


def finger_contacts(m, d, only_geom=None):
    """
    {手指名: 法向力 N}：手指碰到手以外的东西（积木、地面……）的接触力。
    only_geom 给定时只算和这个 geom 的接触。
    """
    fob = _FOB.setdefault(id(m), finger_of_body(m))
    out = {}
    f6 = np.zeros(6)
    for i in range(d.ncon):
        c = d.contact[i]
        b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
        hand1, hand2 = m.body_rootid[b1] == 1, m.body_rootid[b2] == 1
        if hand1 == hand2:                      # 手和手（自碰撞已关）或者两个外物之间
            continue
        hand_body, other_geom = (b1, c.geom2) if hand1 else (b2, c.geom1)
        if only_geom is not None and other_geom != only_geom:
            continue
        f = fob.get(hand_body)
        if f:
            mujoco.mj_contactForce(m, d, i, f6)
            out[f] = out.get(f, 0.0) + abs(f6[0])
    return out


def contacts_on_cube(m, d, info):
    """{手指名: 法向力 N}，只算和积木的接触"""
    return finger_contacts(m, d, info["cube_geom"])


class HandLift:
    """
    拇指 + 至少一根其他手指同时捏住积木 0.4 s：
      - 抓取辅助（assist=True）：把积木焊在手上（保持当前相对位置，不会跳）；
      - 手慢慢抬起 lift 高度。
    松手：手指张开超过 15°（和捏住时相比），或者所有手指离开积木 0.3 s：解除焊接、手放回原高度。
    手的高度另外加上 manual（滑块面板控制）。
    """

    OPEN_JOINTS = [j for f in ("thumb", "index", "mid") for j in gm.FINGERS[f] if "_abd_" not in j]

    def __init__(self, info, speed=0.04, force_on=0.5, hold=0.4, release=0.3, assist=True, open_angle=15.0, auto=True):
        self.info, self.speed, self.force_on, self.hold, self.release, self.assist = info, speed, force_on, hold, release, assist
        self.auto = auto              # 抓住后自动抬手
        self.open_angle = math.radians(open_angle)
        self.dirs = gm.directions()
        self.manual = 0.0
        self.height = 0.0
        self.t_grasp = 0.0
        self.t_open = 0.0
        self.attached = False
        self.ctrl_at_grasp = None

    def _opened(self, m, d):
        """和捏住那一刻相比，拇指/食指/中指的弯曲关节平均往张开方向退了多少（rad，正数 = 张开）"""
        if self.ctrl_at_grasp is None:
            return 0.0
        return -float(np.mean([self.dirs[j] * (d.ctrl[m.actuator(j).id] - self.ctrl_at_grasp[j]) for j in self.OPEN_JOINTS]))

    def key(self, keycode):
        if keycode == 266:      # PageUp
            self.manual += 0.01
        elif keycode == 267:    # PageDown
            self.manual -= 0.01

    def _attach(self, m, d):
        """焊接约束的相对位姿设成当前的，避免一打开就把积木拽过去"""
        e, b1, b2 = self.info["weld"], self.info["cube_body"], self.info["hand_body"]
        R1 = d.xmat[b1].reshape(3, 3)
        relpos = R1.T @ (d.xpos[b2] - d.xpos[b1])
        q1inv = np.zeros(4)
        mujoco.mju_negQuat(q1inv, d.xquat[b1])
        relq = np.zeros(4)
        mujoco.mju_mulQuat(relq, q1inv, d.xquat[b2])
        m.eq_data[e, 0:3] = 0
        m.eq_data[e, 3:6] = relpos
        m.eq_data[e, 6:10] = relq
        d.eq_active[e] = 1
        self.attached = True

    def _detach(self, d):
        d.eq_active[self.info["weld"]] = 0
        self.attached = False

    def drop(self, d):
        """外部移动积木时先松开"""
        self._detach(d)
        self.t_grasp, self.t_open, self.ctrl_at_grasp = 0.0, 0.0, None

    def update(self, m, d, dt):
        f = contacts_on_cube(m, d, self.info)
        pinching = f.get("thumb", 0) > self.force_on and any(v > self.force_on for k, v in f.items() if k != "thumb")
        self.t_grasp = self.t_grasp + dt if pinching else 0.0
        touching = sum(f.values()) > 0.05                     # 还有任何一根手指碰着积木就不算松手
        self.t_open = 0.0 if touching else self.t_open + dt
        if not self.attached and self.t_grasp > self.hold:
            if self.assist:
                self._attach(m, d)
            else:
                self.attached = True
            self.ctrl_at_grasp = {j: float(d.ctrl[m.actuator(j).id]) for j in self.OPEN_JOINTS}
        elif self.attached and (self.t_open > self.release or self._opened(m, d) > self.open_angle):
            if self.assist:
                self._detach(d)
            else:
                self.attached = False
        target = self.info["lift"] if (self.attached and self.auto) else 0.0
        step = self.speed * dt
        self.height += max(-step, min(step, target - self.height))
        d.mocap_pos[0] = self.info["root_pos"] + self.info["base_offset"] + [0, 0, self.height + self.manual]
        return self.attached, f
