#!/usr/bin/env python3
"""
骨架图交互界面：MuJoCo 窗口里实时看，旁边一个面板调：
  13 个关节角度（正 = 握拳 / 向拇指侧）、骨头粗细、关节球大小、转轴长度、CAD 透明度，
  显示开关（CAD / 骨架 / 转轴 / 标签），预设姿态，保存高清 PNG（用当前窗口视角）。

  python tools/skeleton_gui.py
MuJoCo 窗口里鼠标左键拖动旋转、右键平移、滚轮缩放；保存时用的就是这个视角。
"""
import json
import math
import os
import subprocess
import sys
import time
import tkinter as tk

import mujoco
import mujoco.viewer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import glove_model as gm  # noqa: E402
import skeleton_lib as sk  # noqa: E402

FIGS = os.path.join(gm.ROOT, "docs", "figs")
SHORT = {"thumb": "拇指", "index": "食指", "mid": "中指", "ring": "无名指"}
JOINT_CN = {"cmc_abd": "CMC 对掌", "cmc_flex": "CMC 弯曲", "mcp_flex": "MCP 弯曲", "mcp": "MCP 弯曲", "mcp_abd": "MCP 侧摆", "pip": "PIP", "ip": "IP"}


def joint_label(j):
    f = gm.finger_of(j)
    key = j.replace(f + "_", "").replace("_motor", "")
    return f"ID {gm.DEFAULT_IDS[j]:2d}  {SHORT[f]} {JOINT_CN.get(key, key)}"


class Panel:
    def __init__(self, state):
        self.state = state
        self.root = tk.Tk()
        self.root.title("骨架图 控制面板")
        self.root.geometry("+20+40")
        self.closed = False
        self.save_request = None
        self.root.protocol("WM_DELETE_WINDOW", self._close)
        font = ("Noto Sans CJK TC", 10)
        self.vars = {}
        left = tk.Frame(self.root); left.pack(side=tk.LEFT, padx=8, pady=6, anchor="n")
        right = tk.Frame(self.root); right.pack(side=tk.LEFT, padx=8, pady=6, anchor="n")
        tk.Label(left, text="关节角度 (°)，正 = 握拳 / 向拇指侧", font=font).pack(anchor="w")
        for js in gm.FINGERS.values():
            for j in js:
                abd = "_abd_" in j
                v = tk.DoubleVar(value=state["angles_deg"][j])
                tk.Scale(left, label=joint_label(j), from_=-35 if abd else -25, to=35 if abd else 110, resolution=1,
                         orient=tk.HORIZONTAL, length=300, variable=v, font=font).pack()
                self.vars[j] = v
        row = tk.Frame(left); row.pack(pady=4)
        for txt, amp in (("零位", 0), ("半握", 35), ("握拳", 70)):
            tk.Button(row, text=txt, font=font, command=lambda a=amp: self._preset(a)).pack(side=tk.LEFT, padx=3)
        tk.Label(right, text="外观", font=font).pack(anchor="w")

        def slider(key, text, frm, to, res):
            v = tk.DoubleVar(value=state[key])
            tk.Scale(right, label=text, from_=frm, to=to, resolution=res, orient=tk.HORIZONTAL, length=300, variable=v, font=font).pack()
            self.vars[key] = v
        slider("bone_mm", "骨头粗细 (mm)", 1, 12, 0.5)
        slider("joint_mm", "关节球半径 (mm)", 0, 14, 0.5)
        slider("axis_mm", "转轴箭头长度 (mm)", 0, 90, 5)
        slider("mesh_alpha", "CAD 透明度 (0 = 不显示)", 0, 1, 0.02)
        for key, text in (("show_mesh", "显示 CAD"), ("show_bones", "显示骨架"), ("show_axes", "显示关节转轴"),
                          ("show_labels", "显示电机 ID（保存的图里）")):
            v = tk.BooleanVar(value=state[key])
            tk.Checkbutton(right, text=text, variable=v, font=font).pack(anchor="w")
            self.vars[key] = v
        tk.Label(right, text="保存文件名（docs/figs/ 下）", font=font).pack(anchor="w", pady=(10, 0))
        self.name = tk.Entry(right, font=font, width=28); self.name.insert(0, "skeleton_custom"); self.name.pack(anchor="w")
        tk.Button(right, text="保存高清 PNG（当前窗口视角）", font=font, command=self._save).pack(anchor="w", pady=6)
        self.status = tk.StringVar(value="")
        tk.Label(right, textvariable=self.status, font=font, wraplength=300, justify=tk.LEFT).pack(anchor="w")

    def _preset(self, amp):
        for j, v in self.vars.items():
            if j in self.state["angles_deg"]:
                v.set(amp * 0.4 if "_abd_" in j else amp)

    def _save(self):
        self.save_request = self.name.get().strip() or "skeleton_custom"

    def _close(self):
        self.closed = True
        self.root.destroy()

    def read(self):
        for j in self.state["angles_deg"]:
            self.state["angles_deg"][j] = float(self.vars[j].get())
        for key in ("bone_mm", "joint_mm", "axis_mm", "mesh_alpha"):
            self.state[key] = float(self.vars[key].get())
        for key in ("show_mesh", "show_bones", "show_axes", "show_labels"):
            self.state[key] = bool(self.vars[key].get())

    def update(self):
        if not self.closed:
            self.root.update()


def main():
    md = gm.directions()
    state = json.loads(json.dumps(sk.DEFAULT_STATE))
    m = sk.build_model()
    d = mujoco.MjData(m)
    sk.apply_mesh_alpha(m, state["mesh_alpha"])
    sk.set_pose(m, d, state["angles_deg"], md)
    anatomy = sk.Anatomy(m, md)
    center, dist = sk.fit_camera(m, d)
    panel = Panel(state)
    viewer = mujoco.viewer.launch_passive(m, d, show_left_ui=False, show_right_ui=False)
    viewer.cam.azimuth, viewer.cam.elevation, viewer.cam.distance = state["cam"]["azimuth"], state["cam"]["elevation"], dist
    viewer.cam.lookat[:] = center
    viewer.opt.label = mujoco.mjtLabel.mjLABEL_NONE
    t_sim = time.time()
    try:
        while viewer.is_running() and not panel.closed:
            panel.read()
            panel.update()
            for j, v in state["angles_deg"].items():
                d.ctrl[m.actuator(j).id] = md[j] * math.radians(v)
            now = time.time()
            steps = 0
            while t_sim < now and steps < 40:
                mujoco.mj_step(m, d)
                t_sim += m.opt.timestep
                steps += 1
            t_sim = max(t_sim, now - 0.05)
            sk.apply_mesh_alpha(m, state["mesh_alpha"] if state["show_mesh"] else 0.0)
            with viewer.lock():
                viewer.user_scn.ngeom = 0
                sk.draw_skeleton(viewer.user_scn, m, d, state, md, anatomy)
            viewer.sync()
            if panel.save_request:
                name = panel.save_request; panel.save_request = None
                state["cam"] = dict(azimuth=float(viewer.cam.azimuth), elevation=float(viewer.cam.elevation),
                                    distance=float(viewer.cam.distance), lookat=[float(x) for x in viewer.cam.lookat])
                os.makedirs(FIGS, exist_ok=True)
                js = os.path.join(FIGS, name + ".json")
                json.dump(state, open(js, "w"), indent=1, ensure_ascii=False)
                panel.status.set("渲染中 ...")
                panel.update()
                env = dict(os.environ, MUJOCO_GL="egl")
                r = subprocess.run([sys.executable, os.path.join(gm.ROOT, "tools", "fig_skeleton.py"), "--state", js,
                                    "--out", os.path.join(FIGS, name + ".png")], env=env, capture_output=True, text=True)
                panel.status.set(("已保存 docs/figs/" + name + ".png（设置在同名 .json）") if r.returncode == 0 else ("保存失败: " + r.stderr[-300:]))
            time.sleep(1 / 60)
    except KeyboardInterrupt:
        pass
    finally:
        if not panel.closed:
            panel._close()
        viewer.close()
        time.sleep(0.2)


if __name__ == "__main__":
    main()
