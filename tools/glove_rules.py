"""
这个手套装配体特有的合并规则（给 check_mates.py 和 export_glove.py 共用）。

背景：Assembly 1 URDF 里每个电机被拆成了 16 个散件（CASE_M/F/B、HORN、螺丝、接插件……），
零件名带相同的 <编号>，但它们之间没有任何配合；电机壳和支架之间是用两个圆柱/旋转配合对齐螺丝孔，
而不是紧固。onshape-to-robot 只认紧固 / fix_ / Group，所以这里生成一些“虚拟的紧固配合”：

  规则 1  同一编号的电机散件全部并成一个刚体（含 HORN：法兰当作电机的一部分，被驱动的零件绕它转）
  规则 2  两个刚体之间若有 >=2 个旋转/圆柱配合且轴线不重合（= 两个螺丝孔），它们是固定在一起的
          （两者之间若已有 dof_/closing_ 则不合并，只给出警告）
  规则 4  装配体里被图钉固定（grounded）的实例焊到基座上（它们都不相对世界运动）
  规则 5  电机和零件之间只有 1 个圆柱/旋转 + 1 个平行配合，且平行约束的方向和圆柱轴不同（转不动）-> 固定
"""
import re
from collections import defaultdict

import numpy as np

MOTOR_PART = re.compile(
    r"^(DC15_A01_\w+_DUMMY|PHS_1_7X20_5_DC10(_\d+)?|BTS2_M2_6X8(_\d+)?|B3B-EH(_\d+)?) <(\d+)>$"
)
PREFIX = re.compile(r"^(dof|closing|frame)_")


def two_ends(fd):
    e = fd.get("matedEntities", [])
    if len(e) != 2 or not e[0]["matedOccurrence"] or not e[1]["matedOccurrence"]:
        return None
    return e[0]["matedOccurrence"][0], e[1]["matedOccurrence"][0]


def fastened(name, a, b):
    return {
        "featureType": "mate",
        "suppressed": False,
        "featureData": {
            "name": name,
            "mateType": "FASTENED",
            "matedEntities": [{"matedOccurrence": [a]}, {"matedOccurrence": [b]}],
        },
    }


def world_axis(root, ent):
    occ = next(o for o in root["occurrences"] if o["path"] == ent["matedOccurrence"])
    T = np.array(occ["transform"]).reshape(4, 4)
    cs = ent["matedCS"]
    z = T[:3, :3] @ np.array(cs["zAxis"])
    o = T[:3, :3] @ np.array(cs["origin"]) + T[:3, 3]
    return z, o


def coaxial(axes, tol_ang=1e-3, tol_dist=1e-4):
    z0, o0 = axes[0]
    for z, o in axes[1:]:
        if abs(abs(np.dot(z0, z)) - 1) > tol_ang or np.linalg.norm(np.cross(z0, o - o0)) > tol_dist:
            return False
    return True


def synthetic_mates(root):
    """返回 (要追加的虚拟紧固配合列表, 日志行列表)"""
    inst = {i["id"]: i for i in root["instances"] if not i.get("suppressed")}
    def name(i):
        m = MOTOR_PART.match(inst[i]["name"])
        return f"电机<{m.group(5)}>" if m else inst[i]["name"]
    log, out = [], []

    # ---- 规则 3：closing_ 用了圆柱配合的，按旋转闭合处理（轴向位置已由宽度配合固定） ----
    for f in root["features"]:
        fd = f.get("featureData", {})
        if f["featureType"] == "mate" and fd.get("name", "").startswith("closing_") and fd.get("mateType") == "CYLINDRICAL":
            fd["mateType"] = "REVOLUTE"
            log.append(f"规则3 {fd['name']}: 圆柱 -> 按旋转闭合")

    # ---- 规则 1：电机散件 ----
    motors = defaultdict(list)
    for i in inst.values():
        m = MOTOR_PART.match(i["name"])
        if m:
            motors[m.group(5)].append(i["id"])
    for n, ids in sorted(motors.items(), key=lambda kv: int(kv[0])):
        head = next((i for i in ids if "CASE_M" in name(i)), ids[0])
        for i in ids:
            if i != head:
                out.append(fastened(f"auto_motor{n}_{name(i).split(' ')[0]}", head, i))
        log.append(f"规则1 电机<{n}>: {len(ids)} 个散件并成一个刚体")

    # ---- 规则 2：两个螺丝孔 = 固定 ----
    parent = {i: i for i in inst}

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    mates = [f["featureData"] for f in root["features"] + out
             if f["featureType"] == "mate" and not f.get("suppressed")]
    for fd in mates:
        ab = two_ends(fd)
        if not ab or ab[0] not in parent or ab[1] not in parent:
            continue
        n = fd.get("name", "")
        if n.startswith("fix_") or (fd["mateType"] == "FASTENED" and not PREFIX.match(n)):
            parent[find(ab[0])] = find(ab[1])
    for f in root["features"]:
        if f["featureType"] == "mateGroup" and not f.get("suppressed"):
            occs = [o["occurrence"][0] for o in f["featureData"]["occurrences"]]
            for o in occs[1:]:
                if occs[0] in parent and o in parent:
                    parent[find(occs[0])] = find(o)

    changed = True
    while changed:
        changed = False
        pairs = defaultdict(list)
        named = set()
        for fd in mates:
            ab = two_ends(fd)
            if not ab or ab[0] not in parent or ab[1] not in parent:
                continue
            key = frozenset((find(ab[0]), find(ab[1])))
            if len(key) < 2:
                continue
            if PREFIX.match(fd.get("name", "")):
                named.add(key)
            elif fd["mateType"] in ("REVOLUTE", "CYLINDRICAL"):
                pairs[key].append(fd)
        for key, lst in pairs.items():
            if len(lst) < 2:
                continue
            axes = [world_axis(root, fd["matedEntities"][0]) for fd in lst]
            if coaxial(axes):
                continue
            a, b = tuple(key)
            if key in named:
                log.append(f"!! 规则2 跳过: {name(a)} <-> {name(b)} 有两个不同轴的配合 {[fd['name'] for fd in lst]}，但它们之间还有 dof_/closing_，请检查")
                continue
            out.append(fastened(f"auto_rigid_{len(out)}", a, b))
            mates.append(out[-1]["featureData"])
            parent[find(a)] = find(b)
            log.append(f"规则2 固定: {name(a)} <-> {name(b)}  ({', '.join(fd['name'] for fd in lst)})")
            changed = True

    # ---- 规则 4：装配体里被图钉固定（grounded）的实例 = 不相对世界运动 = 和基座焊在一起 ----
    order = [i["id"] for i in root["instances"] if not i.get("suppressed")]
    base = order[0] if order else None
    for occ in root.get("occurrences", []):
        if occ.get("fixed") and len(occ["path"]) == 1 and occ["path"][0] in parent and base is not None:
            i = occ["path"][0]
            if find(i) == find(base):
                continue
            out.append(fastened(f"auto_ground_{len(out)}", base, i))
            mates.append(out[-1]["featureData"])
            parent[find(i)] = find(base)
            log.append(f"规则4 固定: {name(i)} 在装配体里被图钉固定 -> 焊到基座 {name(base)}")

    # ---- 规则 5：电机和零件之间只有 1 个圆柱/旋转 + 1 个方向不同的平行配合（转不动）= 装死 ----
    motor_ids = {i for i in inst if MOTOR_PART.match(inst[i]["name"])}
    cyl, par, named = defaultdict(list), defaultdict(list), set()
    for fd in mates:
        ab = two_ends(fd)
        if not ab or ab[0] not in parent or ab[1] not in parent:
            continue
        key = frozenset((find(ab[0]), find(ab[1])))
        if len(key) < 2:
            continue
        if PREFIX.match(fd.get("name", "")):
            named.add(key)
        elif fd["mateType"] in ("REVOLUTE", "CYLINDRICAL"):
            cyl[key].append(fd)
        elif fd["mateType"] == "PARALLEL":
            par[key].append(fd)
    for key in cyl:
        if key in named or key not in par:
            continue
        a, b = tuple(key)
        if not any(find(m) in key for m in motor_ids):
            continue
        z_c, _ = world_axis(root, cyl[key][0]["matedEntities"][0])
        z_p, _ = world_axis(root, par[key][0]["matedEntities"][0])
        if abs(np.dot(z_c, z_p)) > 0.99:
            continue                                   # 平行约束和圆柱轴同向：还能转，不是装死
        out.append(fastened(f"auto_mount_{len(out)}", a, b))
        parent[find(a)] = find(b)
        log.append(f"规则5 固定: {name(a)} <-> {name(b)}  ({cyl[key][0]['name']} + {par[key][0]['name']}，平行约束和圆柱轴垂直，电机转不动)")
    return out, log
