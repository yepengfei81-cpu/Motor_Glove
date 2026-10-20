#!/usr/bin/env python3
"""
带自动合并规则的导出：等价于 `onshape-to-robot <目录>`，但在读取装配体之后、分析配合之前，
把 glove_rules.synthetic_mates() 生成的虚拟紧固配合追加进去（电机散件、两个螺丝孔的固定连接）。

用法:  python tools/export_glove.py glove          # URDF
       python tools/export_glove.py glove_mujoco   # MuJoCo
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from glove_rules import synthetic_mates  # noqa: E402

from onshape_to_robot import assembly as asm  # noqa: E402
from onshape_to_robot import export  # noqa: E402

_orig = asm.Assembly.retrieve_assembly


def retrieve_with_rules(self):
    _orig(self)
    root = self.assembly_data["rootAssembly"]
    raw_path = os.path.join(self.config.output_directory, "assembly_raw.json")
    with open(raw_path, "w", encoding="utf-8") as f:
        json.dump(self.assembly_data, f, ensure_ascii=False, indent=1)
    extra, log = synthetic_mates(root)
    root["features"] = root["features"] + extra
    print(f"* 自动合并规则追加了 {len(extra)} 个虚拟紧固配合:")
    for line in log:
        print("   ", line)


asm.Assembly.retrieve_assembly = retrieve_with_rules

if __name__ == "__main__":
    robot_dir = sys.argv[1]
    sys.argv = ["onshape-to-robot"] + sys.argv[1:]
    export.main()
    import subprocess
    subprocess.run([sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)), "fill_inertia.py"), robot_dir], check=True)
    tools = os.path.dirname(os.path.abspath(__file__))
    subprocess.run([sys.executable, os.path.join(tools, "place_root.py"), robot_dir], check=True)
    if os.path.exists(os.path.join(robot_dir, "robot.xml")):
        subprocess.run([sys.executable, os.path.join(tools, "tune_mjcf.py"), robot_dir], check=True)
    if os.path.exists(os.path.join(robot_dir, "robot.xml")):
        # URDF 目录的摆放姿态是从 MuJoCo 模型算的，MuJoCo 模型更新后顺便重算；测试方块也跟着重新摆
        base_name = os.path.basename(os.path.abspath(robot_dir))
        sibling = os.path.join(os.path.dirname(os.path.abspath(robot_dir)), base_name[:-len("_mujoco")] if base_name.endswith("_mujoco") else base_name)
        if os.path.exists(os.path.join(sibling, "robot.urdf")):
            subprocess.run([sys.executable, os.path.join(tools, "place_root.py"), sibling], check=True)
        if os.path.exists(os.path.join(robot_dir, "teleop_scene.xml")):
            subprocess.run([sys.executable, os.path.join(tools, "place_cube.py")], check=True)
