#!/usr/bin/env python3
"""
导出前的配合检查：只调用一次 Onshape API（或读取已保存的 JSON），
按 onshape-to-robot 1.8 的规则检查 dof_ / closing_ / fix_ 命名是否能构成一棵树。

用法（在 ~/motor_glove 下，先 conda activate motor_glove）:
    python tools/check_mates.py glove                 # 从 Onshape 拉取并保存到 glove/assembly_raw.json
    python tools/check_mates.py glove --offline       # 只读本地 glove/assembly_raw.json，不消耗 API 配额
"""
import argparse
import json
import os
import re
import sys
from collections import defaultdict

from dotenv import find_dotenv, load_dotenv

EXPECTED_MOTORS = 13  # 2026-10-09 拇指加了 IP 电机
DOF_TYPES = {"REVOLUTE", "CYLINDRICAL", "SLIDER", "FASTENED", "BALL"}
CLOSING_TYPES = {"REVOLUTE", "FASTENED", "BALL"}


def fetch(robot_dir: str) -> dict:
    from onshape_to_robot.config import Config
    from onshape_to_robot.onshape_api.client import Client

    config = Config(robot_dir)
    client = Client(logging=False, creds=config.config_file)
    if not (config.workspace_id or config.version_id):
        doc = client.get_document(config.document_id)
        config.workspace_id = doc["defaultWorkspace"]["id"]
    return client.get_assembly(
        config.document_id,
        config.version_id if config.version_id else config.workspace_id,
        config.element_id,
        wmv="v" if config.version_id else "w",
        configuration=config.configuration,
    )


def joint_name(mate_name: str) -> str:
    parts = mate_name.split("_")[1:]
    if parts and parts[-1] in ("inv", "inverted"):
        parts = parts[:-1]
    return "_".join(parts)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("robot_dir")
    ap.add_argument("--offline", action="store_true", help="只读本地 assembly_raw.json")
    args = ap.parse_args()

    load_dotenv(find_dotenv(usecwd=True))
    raw_path = os.path.join(args.robot_dir, "assembly_raw.json")
    if args.offline:
        with open(raw_path, encoding="utf-8") as f:
            data = json.load(f)
    else:
        data = fetch(args.robot_dir)
        with open(raw_path, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=1)
        print(f"* 原始装配数据已保存到 {raw_path}（之后可用 --offline 重复检查）\n")

    root = data["rootAssembly"]
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from glove_rules import synthetic_mates, two_ends
    extra, log = synthetic_mates(root)
    root["features"] = root["features"] + extra
    print("* 自动合并规则（与 export_glove.py 一致）:")
    for line in log:
        print("   ", line)
    print()
    for feat in root["features"]:
        if feat["featureType"] == "mate" and not feat.get("suppressed"):
            fd = feat["featureData"]
            if two_ends(fd) is None and fd.get("mateType") in ("REVOLUTE", "CYLINDRICAL", "SLIDER"):
                print(f"!! 「{fd.get('name','')}」({fd['mateType']}) 只连了一个零件（另一端是原点？），工具会忽略它")
    instances = [i for i in root["instances"] if not i.get("suppressed")]
    inst_name = {i["id"]: i["name"] for i in root["instances"]}
    if not instances:
        sys.exit("装配体里没有实例")
    base_id = instances[0]["id"]
    print(f"* 顶层实例 {len(instances)} 个，base link = 第一个实例「{inst_name[base_id]}」\n")

    # ---- 收集配合 ----
    mates, groups = [], []
    for feat in root["features"]:
        if feat.get("suppressed"):
            if feat["featureType"] == "mate":
                n = feat["featureData"].get("name", "")
                if re.match(r"^(dof|closing|fix|frame)_", n):
                    print(f"!! 被抑制的配合「{n}」带有前缀，导出时会被跳过，请恢复它")
            continue
        fd = feat["featureData"]
        if feat["featureType"] == "mateGroup":
            groups.append([o["occurrence"][0] for o in fd["occurrences"]])
        elif feat["featureType"] == "mate":
            ents = fd.get("matedEntities", [])
            if len(ents) != 2 or not ents[0]["matedOccurrence"] or not ents[1]["matedOccurrence"]:
                continue
            mates.append(
                dict(
                    name=fd.get("name", ""),
                    type=fd.get("mateType", "?"),
                    a=ents[0]["matedOccurrence"][0],
                    b=ents[1]["matedOccurrence"][0],
                )
            )

    # ---- 合并刚性连接 (紧固 / fix_ / Group) → body ----
    parent = {i["id"]: i["id"] for i in instances}

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(x, y):
        parent[find(x)] = find(y)

    for m in mates:
        n = m["name"]
        if n.startswith("fix_") or (
            m["type"] == "FASTENED" and not re.match(r"^(dof|closing|frame)_", n)
        ):
            if m["a"] in parent and m["b"] in parent:
                union(m["a"], m["b"])
    for g in groups:
        for k in range(1, len(g)):
            if g[0] in parent and g[k] in parent:
                union(g[0], g[k])

    from glove_rules import MOTOR_PART

    def label(i):
        m = MOTOR_PART.match(inst_name[i])
        return f"电机<{m.group(5)}>" if m else inst_name[i]

    body_of = {i: find(i) for i in parent}
    body_members = defaultdict(list)
    for i, b in body_of.items():
        if label(i) not in body_members[b]:
            body_members[b].append(label(i))

    def bname(b):
        return " + ".join(body_members[b][:3]) + (" …" if len(body_members[b]) > 3 else "")

    # ---- 分类 ----
    dofs, closings, frames, unnamed_rot, problems = [], [], [], [], []
    for m in mates:
        n, t = m["name"], m["type"]
        if m["a"] not in body_of or m["b"] not in body_of:
            continue
        ba, bb = body_of[m["a"]], body_of[m["b"]]
        if n.startswith("dof_"):
            if t not in DOF_TYPES:
                problems.append(f"「{n}」是 {t} 类型，dof_ 只支持旋转/圆柱/滑块/紧固/球")
            if ba == bb:
                problems.append(f"「{n}」连接的两个零件已被合并成同一个刚体（被 fix_/紧固 合并了），这个关节会消失")
            dofs.append((n, t, ba, bb))
        elif n.startswith("closing_"):
            if t not in CLOSING_TYPES:
                problems.append(f"「{n}」是 {t} 类型，closing_ 必须是旋转（MuJoCo 才会生成闭环约束）")
            closings.append((n, t, ba, bb))
        elif n.startswith("frame_"):
            frames.append(n)
        elif t in ("REVOLUTE", "CYLINDRICAL") and ba != bb:
            unnamed_rot.append((n, t, ba, bb))

    # ---- 关节重名（导出后 URDF / MuJoCo 里关节名必须唯一） ----
    from collections import Counter as _C
    for jn, k in _C(joint_name(n) for n, *_ in dofs).items():
        if k > 1:
            problems.append(f"关节名「{jn}」出现了 {k} 次，导出后会重名，必须改成唯一的名字")

    # ---- 电机数量 ----
    motors = [n for n, *_ in dofs if joint_name(n).endswith("_motor")]
    print(f"* 以 _motor 结尾的关节 {len(motors)} 个（期望 {EXPECTED_MOTORS}）")
    for n in sorted(motors):
        print(f"    {joint_name(n)}")
    if len(motors) != EXPECTED_MOTORS:
        problems.append(f"_motor 关节数量是 {len(motors)}，不是 {EXPECTED_MOTORS}")
    print()

    # ---- 树检查：dof_ 之间不能形成回路，所有刚体要从 base 可达 ----
    tparent = {b: b for b in set(body_of.values())}

    def tfind(x):
        while tparent[x] != x:
            tparent[x] = tparent[tparent[x]]
            x = tparent[x]
        return x

    pair_count = defaultdict(list)
    for n, t, ba, bb in dofs:
        pair_count[frozenset((ba, bb))].append(n)
        if ba == bb:
            continue
        if tfind(ba) == tfind(bb):
            problems.append(f"「{n}」让 dof_ 形成了回路；每个回路要有一个 closing_，否则导出会报 'The DOF graph is not a tree'")
        else:
            tparent[tfind(ba)] = tfind(bb)
    for pair, names in pair_count.items():
        if len(names) > 1:
            problems.append(f"同一对零件之间有 {len(names)} 个 dof_：{names}，只能留一个")

    base_body = body_of[base_id]
    visited = {b for b in tparent if tfind(b) == tfind(base_body)}
    reached = visited
    all_bodies = set(body_of.values())
    orphans = all_bodies - reached
    print(f"* 刚体（合并后）{len(all_bodies)} 个，其中从 base 沿 dof_ 能到达 {len(reached)} 个")
    if orphans:
        print(f"!! 下面 {len(orphans)} 个刚体没有被任何 dof_ 接进来，导出时会被固定到基座上：")
        for b in sorted(orphans, key=bname):
            print(f"    - {bname(b)}")
    print()

    # ---- 没有前缀的旋转/圆柱 ----
    forgotten = [(n, t, ba, bb) for n, t, ba, bb in unnamed_rot if ba in orphans or bb in orphans]
    if unnamed_rot:
        print(f"* 没有前缀的旋转/圆柱配合 {len(unnamed_rot)} 个（同轴的另一侧支撑属于正常情况）")
        for n, t, ba, bb in unnamed_rot:
            flag = "  <-- 连着一个孤立刚体，可能漏标了" if (n, t, ba, bb) in forgotten else ""
            print(f"    {n:14s} {t:12s} {bname(ba)}  <->  {bname(bb)}{flag}")
        print()

    # ---- 回路数核对 ----
    print(f"* dof_ {len(dofs)} 个，closing_ {len(closings)} 个，frame_ {len(frames)} 个")
    for n, t, ba, bb in closings:
        print(f"    {n:28s} {t:10s} {bname(ba)}  <->  {bname(bb)}")
    print()

    if problems:
        print("========== 需要处理的问题 ==========")
        for p in problems:
            print(" -", p)
        sys.exit(1)
    print("========== 没有发现问题，可以导出 ==========")


if __name__ == "__main__":
    main()
