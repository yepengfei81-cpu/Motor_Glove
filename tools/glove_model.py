"""
手套模型的公共信息：关节分组、电机 ID 约定、在 MuJoCo 模型里推算"握拳/向拇指侧摆"对应的电机正负号。
被 place_cube.py、calibrate.py、glove_live.py 共用。
"""
import json
import os

import numpy as np

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
MJCF = os.path.join(ROOT, "glove_mujoco", "robot.xml")
SCENE = os.path.join(ROOT, "glove_mujoco", "teleop_scene.xml")
DIRECTIONS_FILE = os.path.join(ROOT, "glove_mujoco", "joint_directions.json")

# 每根手指的电机关节，按机构上从基座往外的顺序（和 URDF 的树一致）：
# 四指是 MCP 弯曲（装在基座上）-> MCP 侧摆 -> PIP；拇指是 CMC 侧摆 -> CMC 弯曲 -> MCP 弯曲 -> IP
FINGERS = {
    "thumb": ["thumb_cmc_abd_motor", "thumb_cmc_flex_motor", "thumb_mcp_flex_motor", "thumb_ip_motor"],
    "index": ["index_mcp_flex_motor", "index_mcp_abd_motor", "index_pip_motor"],
    "mid": ["mid_mcp_motor", "mid_mcp_abd_motor", "mid_pip_motor"],
    "ring": ["ring_mcp_flex_motor", "ring_mcp_abd_motor", "ring_pip_motor"],
}
# 约定的电机 ID 按上面的顺序编号：拇指 1-4，食指 5-7，中指 8-10，无名指 11-13
DEFAULT_IDS = {j: i + 1 for i, j in enumerate(sum(FINGERS.values(), []))}
# 手背法线在基座(part_1)坐标系里的参考方向。来自 2026-10-07 用户确认过的"手掌朝前"姿态，
# 基座零件在 Onshape 里是固定的，所以这个方向不随零位、CAD 修改而变。只用它来定正负。
DORSAL_REF_BASE = np.array([0.181, -0.983, -0.001])

# 每根手指远端那节（用来看指尖往哪走）
TIP_JOINT = {"thumb": "thumb_ip", "index": "index_pip", "mid": "mid_pip", "ring": "ring_pip"}


def finger_of(joint):
    return next(f for f, js in FINGERS.items() if joint in js)


def physical_positive(joint):
    """标定时让用户往哪个方向动这个关节（这个方向定义为关节角的正方向）"""
    if joint == "thumb_cmc_abd_motor":
        return "拇指向食指/手心方向收（对掌）"
    if "_abd_" in joint:
        return "手指向拇指一侧摆"
    return "弯曲（握拳方向）"


def dorsal_normal(m, d):
    """
    手背法线（世界坐标）：三个 MCP 侧摆电机轴的平均（这三个电机固定在基座上），
    正负按 DORSAL_REF_BASE 定。place_root.py 用的是同一个函数。
    """
    import mujoco
    jid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
    abd = [jid(f"{f}_mcp_abd_motor") for f in ("index", "mid", "ring")]
    n = np.mean([d.xaxis[j] * np.sign(np.dot(d.xaxis[j], d.xaxis[abd[0]])) for j in abd], axis=0)
    n /= np.linalg.norm(n)
    ref = d.xmat[1].reshape(3, 3) @ DORSAL_REF_BASE
    return n if np.dot(n, ref) >= 0 else -n


def physical_negative(joint):
    if joint == "thumb_cmc_abd_motor":
        return "拇指向外张开（远离手心）"
    if "_abd_" in joint:
        return "手指向外侧摆（远离拇指）"
    return "伸直（往手背方向）"


def demo_hint(joint, model_dir):
    """模型里"正转"在真手上大概对应什么动作（拇指的方向以仿真演示为准）"""
    text = physical_positive(joint) if model_dir.get(joint, 1) > 0 else physical_negative(joint)
    return text + ("（拇指以仿真演示为准）" if joint.startswith("thumb") else "")


def _load(path=SCENE):
    import mujoco
    m = mujoco.MjModel.from_xml_path(path)
    return m, mujoco.MjData(m)


def compute_directions(path=SCENE, step=0.3):
    """
    对每个电机关节：在模型里让它转 +step，看对应指尖的位移，
    返回 {joint: +1/-1}，表示"电机关节角为正"是否对应 physical_positive() 描述的方向。
    """
    import mujoco
    m, d = _load(path)
    m.opt.disableflags = int(m.opt.disableflags) | int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    acts = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(m.nu)]
    jid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
    tipb = {f: m.jnt_bodyid[jid(j)] for f, j in TIP_JOINT.items()}

    def settle(ctrl):
        mujoco.mj_resetData(m, d)
        d.ctrl[:] = 0
        for a, v in ctrl.items():
            d.ctrl[acts.index(a)] = v
        for _ in range(2000):
            mujoco.mj_step(m, d)
        return {f: d.xipos[b].copy() for f, b in tipb.items()}

    zero = settle({})
    mujoco.mj_resetData(m, d)
    mujoco.mj_forward(m, d)
    n = dorsal_normal(m, d)
    # 机构在手背；手心方向 = -n。但指尖本身也在手背上方，所以用"指尖到手掌中心"的方向更稳
    palm_dir = -n
    thumb_c = zero["thumb"]
    out = {}
    for f, joints in FINGERS.items():
        for j in joints:
            moved = settle({j: step})[f] - zero[f]
            if "_abd_" in j and f != "thumb":
                ref = thumb_c - zero[f]
                ref -= np.dot(ref, n) * n
            elif j == "thumb_cmc_abd_motor":
                ref = zero["index"] - zero["thumb"]
            else:
                ref = palm_dir
            out[j] = 1 if np.dot(moved, ref) > 0 else -1
    return out


def directions(path=SCENE, refresh=False):
    """读缓存的方向表；模型更新过（robot.xml 比缓存新）就重算"""
    if not refresh and os.path.exists(DIRECTIONS_FILE) and os.path.getmtime(DIRECTIONS_FILE) > os.path.getmtime(MJCF):
        return json.load(open(DIRECTIONS_FILE))
    out = compute_directions(path)
    json.dump(out, open(DIRECTIONS_FILE, "w"), indent=1)
    return out


def joint_axes(path=SCENE):
    """
    每个电机关节在零位时的转轴方向，表示在基座(part_1)坐标系里。
    基座在 Onshape 里是固定的，所以只有关节轴真的反了（比如重新导出时配合方向变了）这个向量才会反向。
    标定时记下它，之后用它判断要不要把标定方向翻过来。
    """
    import mujoco
    m, d = _load(path)
    mujoco.mj_forward(m, d)
    R = d.xmat[1].reshape(3, 3)
    out = {}
    for joint in sum(FINGERS.values(), []):
        j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, joint)
        out[joint] = [float(v) for v in R.T @ d.xaxis[j]]
    return out
