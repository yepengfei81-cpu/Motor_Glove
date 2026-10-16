#!/usr/bin/env python3
"""
给 PPT 用的一组图（白底、2000 px 宽），输出到 docs/figs/ppt/：
  cad_<view>.png            干净的 CAD 渲染（手掌朝前 / 手背 / 拇指侧 / 俯视）
  modules_<view>.png        按手指上色：拇指 / 食指 / 中指 / 无名指 / 基座
  rcm_mid_mcp.png           中指 MCP 双平行四边形：0° / 30° / 60° 三个姿态叠影 + 虚拟转轴
  rom_overlay_<view>.png    全手 零位 / 半握 / 握拳 叠影
  sim_grasp_open.png / sim_grasp_hold.png   MuJoCo 里抓积木（张开 / 抓起）
  python tools/fig_ppt.py [--size 2000 1500]
"""
import argparse
import math
import os
import sys

import numpy as np
import mujoco
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import glove_model as gm  # noqa: E402
import skeleton_lib as sk  # noqa: E402
import sim_scenes  # noqa: E402
from figtext import T  # noqa: E402

OUT = os.path.join(gm.ROOT, "docs", "figs", "ppt")
VIEWS = {"palm_front": (180, -8), "dorsal": (0, -8), "thumb_side": (250, -5), "top": (180, -78), "palm_oblique": (215, -25)}
FINGER_RGBA = {"thumb": (0.90, 0.45, 0.10, 1), "index": (0.20, 0.55, 0.90, 1), "mid": (0.25, 0.70, 0.35, 1), "ring": (0.75, 0.30, 0.75, 1), None: (0.55, 0.58, 0.64, 1)}
FINGER_CN = {"thumb": T("thumb"), "index": T("index"), "mid": T("mid"), "ring": T("ring"), None: T("base")}


def camera(m, d, az, el, dist_scale=1.0):
    center, dist = sk.fit_camera(m, d)
    cam = mujoco.MjvCamera()
    cam.azimuth, cam.elevation, cam.distance = az, el, dist * dist_scale
    cam.lookat[:] = center
    return cam


def render(m, d, cam, W, H, extra=None):
    r = mujoco.Renderer(m, H, W)
    r.update_scene(d, cam, mujoco.MjvOption())
    if extra:
        extra(r.scene)
    img = Image.fromarray(r.render())
    r.close()
    return img


def multiply(images):
    acc = np.ones(np.asarray(images[0]).shape[:2] + (3,), dtype=np.float32)
    for im in images:
        acc *= np.asarray(im.convert("RGB"), dtype=np.float32) / 255
    return Image.fromarray((acc * 255).astype(np.uint8))


def blend(images):
    """加权平均：每张图在白底上渲染，叠起来像重影，不会像相乘那样变黑"""
    acc = np.zeros(np.asarray(images[0]).shape[:2] + (3,), dtype=np.float32)
    for im in images:
        acc += np.asarray(im.convert("RGB"), dtype=np.float32) / 255
    return Image.fromarray((acc / len(images) * 255).astype(np.uint8))


POSE_TINT = [(0.55, 0.58, 0.64, 1), (0.25, 0.55, 0.90, 1), (0.90, 0.45, 0.10, 1)]   # 零位 灰 / 半握 蓝 / 握拳 橙


def legend(im, items, font):
    dr = ImageDraw.Draw(im)
    W, H = im.size
    lx, ly = int(W * 0.02), int(H * 0.86)
    for col, txt in items:
        dr.rectangle([lx, ly, lx + 36, ly + 16], fill=tuple(int(c * 255) for c in col[:3]))
        dr.text((lx + 50, ly - 12), txt, fill=(30, 30, 30), font=font)
        ly += int(H * 0.03)
    return im


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--size", type=int, nargs=2, default=[2000, 1500])
    args = ap.parse_args()
    W, H = args.size
    os.makedirs(OUT, exist_ok=True)
    font = ImageFont.truetype(sk.FONT, int(H * 0.021))
    md = gm.directions()
    m = sk.build_model(W, H)
    d = mujoco.MjData(m)
    sk.set_pose(m, d, sk.DEFAULT_STATE["angles_deg"], md)
    meshes = [g for g in range(m.ngeom) if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH and m.geom_group[g] != 3]

    # 1) 干净的 CAD 渲染
    for g in meshes:
        m.geom_rgba[g] = [0.62, 0.66, 0.74, 1]
    for name, (az, el) in VIEWS.items():
        render(m, d, camera(m, d, az, el), W, H).save(os.path.join(OUT, f"cad_{name}.png"))
    print("* cad_*.png")

    # 2) 按手指上色
    fob = sim_scenes.finger_of_body(m)
    for g in meshes:
        m.geom_rgba[g] = FINGER_RGBA[fob.get(int(m.geom_bodyid[g]))]
    for name in ("palm_front", "dorsal", "palm_oblique"):
        az, el = VIEWS[name]
        im = render(m, d, camera(m, d, az, el), W, H)
        legend(im, [(FINGER_RGBA[f], FINGER_CN[f]) for f in ("thumb", "index", "mid", "ring", None)], font)
        im.save(os.path.join(OUT, f"modules_{name}.png"))
    print("* modules_*.png")

    # 3) 中指 MCP 双平行四边形：三个姿态叠影 + 虚拟转轴（其他手指隐藏，基座淡化，中指按姿态换色）
    an = sk.Anatomy(m, md)
    axis = d.xaxis[m.joint("mid_mcp_motor").id] * md["mid_mcp_motor"] * an.rcm_sign
    az = math.degrees(math.atan2(-axis[1], -axis[0])); el = math.degrees(math.asin(-axis[2]))   # 沿转轴看过去
    sk.set_pose(m, d, sk.DEFAULT_STATE["angles_deg"], md)
    cam = camera(m, d, az, max(-85, min(85, el)), 0.7)
    imgs = []
    for k, ang in enumerate((0, 30, 60)):
        for g in meshes:
            f = fob.get(int(m.geom_bodyid[g]))
            m.geom_rgba[g] = POSE_TINT[k] if f == "mid" else ([0.80, 0.82, 0.86, 0.25] if f is None else [0, 0, 0, 0])
        st = dict(sk.DEFAULT_STATE["angles_deg"]); st["mid_mcp_motor"] = ang
        sk.set_pose(m, d, st, md)
        c = an.fingers(m, d)["mid"]["chain"][0][1]

        def extra(scn, c=c):
            sk._line(scn, c - axis * 0.035, c + axis * 0.035, 0.005, sk.RED, mujoco.mjtGeom.mjGEOM_ARROW)
            sk._geom(scn, mujoco.mjtGeom.mjGEOM_SPHERE, [0.007, 0, 0], c, sk.RED)
        imgs.append(render(m, d, cam, W, H, extra))
    im = blend(imgs)
    dr = ImageDraw.Draw(im)
    dr.text((int(W * 0.02), int(H * 0.04)), T("rcm_title"), fill=(30, 30, 30), font=font)
    legend(im, [(POSE_TINT[0], "MCP 0°"), (POSE_TINT[1], "MCP 30°"), (POSE_TINT[2], "MCP 60°"), (sk.RED, T("rcm_axis"))], font)
    im.save(os.path.join(OUT, "rcm_mid_mcp.png"))
    print("* rcm_mid_mcp.png")

    # 4) 全手活动范围叠影：三个姿态三种颜色，加权平均
    for name in ("palm_oblique", "thumb_side", "palm_front"):
        az, el = VIEWS[name]
        sk.set_pose(m, d, sk.DEFAULT_STATE["angles_deg"], md)
        for g in meshes:
            m.geom_rgba[g] = [0.62, 0.66, 0.74, 1]
        cam = camera(m, d, az, el, 1.05)
        imgs = []
        for k, amp in enumerate((0, 35, 70)):
            for g in meshes:
                m.geom_rgba[g] = [0.80, 0.82, 0.86, 1] if fob.get(int(m.geom_bodyid[g])) is None else POSE_TINT[k]
            st = {j: (amp * 0.4 if "_abd_" in j else amp) for j in sk.DEFAULT_STATE["angles_deg"]}
            sk.set_pose(m, d, st, md)
            imgs.append(render(m, d, cam, W, H))
        im = blend(imgs)
        ImageDraw.Draw(im).text((int(W * 0.02), int(H * 0.04)), T("rom_title"), fill=(30, 30, 30), font=font)
        legend(im, [(POSE_TINT[0], T("pose0")), (POSE_TINT[1], T("pose1")), (POSE_TINT[2], T("pose2"))], font)
        im.save(os.path.join(OUT, f"rom_overlay_{name}.png"))
    print("* rom_overlay_*.png")

    # 5) 仿真抓积木（张开 / 抓起）
    x0, y0 = sim_scenes.default_cube_xy()
    mg, info = sim_scenes.grasp_scene(x0, y0)
    mg.vis.global_.offwidth, mg.vis.global_.offheight = W, H
    dg = mujoco.MjData(mg)
    lift = sim_scenes.HandLift(info)
    for i in range(mg.nu):
        n_ = mujoco.mj_id2name(mg, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        t = 0.215 if n_.endswith("pip_motor") or n_ == "thumb_ip_motor" else 0.52
        mg.actuator_forcelimited[i] = 1; mg.actuator_forcerange[i] = [-t, t]
    lift.update(mg, dg, 0); mujoco.mj_forward(mg, dg)
    cam = mujoco.MjvCamera(); cam.lookat[:] = [0.02, 0, 0.09]; cam.distance = 0.42; cam.azimuth = 200; cam.elevation = -18
    r = mujoco.Renderer(mg, H, W)
    r.update_scene(dg, cam); Image.fromarray(r.render()).save(os.path.join(OUT, "sim_grasp_open.png"))
    best = None
    for thumb_abd, thumb_flex, flex in ((35, 50, 65), (25, 40, 60), (40, 60, 70), (20, 30, 50), (30, 45, 75)):
        mujoco.mj_resetData(mg, dg); lift = sim_scenes.HandLift(info); lift.update(mg, dg, 0)
        pinch = {"thumb_cmc_abd_motor": thumb_abd, "thumb_cmc_flex_motor": thumb_flex, "thumb_mcp_flex_motor": thumb_flex * 0.6, "thumb_ip_motor": 20}
        for f in ("index", "mid"):
            for j in gm.FINGERS[f]:
                if "_abd_" not in j:
                    pinch[j] = flex * (1.0 if "mcp" in j else 0.8)
        for k in range(int(7.0 / mg.opt.timestep)):
            s_ = min(1.0, max(0.0, (k * mg.opt.timestep - 0.3) / 1.5))
            for n_, v in pinch.items():
                dg.ctrl[mg.actuator(n_).id] = md[n_] * math.radians(v) * s_
            lift.update(mg, dg, mg.opt.timestep); mujoco.mj_step(mg, dg)
        h = dg.xpos[info["cube_body"]][2] - info["half"]
        if best is None or h > best[0]:
            best = (h, dg.qpos.copy(), dg.ctrl.copy())
        if h > 0.05:
            break
    dg.qpos[:] = best[1]; dg.ctrl[:] = best[2]; mujoco.mj_forward(mg, dg)
    r.update_scene(dg, cam); Image.fromarray(r.render()).save(os.path.join(OUT, "sim_grasp_hold.png"))
    r.close()
    print("* sim_grasp_open.png / sim_grasp_hold.png  （积木离地 %.1f cm）" % (best[0] * 100))


if __name__ == "__main__":
    main()
