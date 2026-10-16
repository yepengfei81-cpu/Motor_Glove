#!/usr/bin/env python3
"""
骨架图（命令行版）。要交互调关节角度、粗细、透明度，用 skeleton_gui.py。

  python tools/fig_skeleton.py                        # 零位，4 个视角，输出到 docs/figs/
  python tools/fig_skeleton.py --no-mesh --bone-mm 6  # 纯骨架、骨头粗一点
  python tools/fig_skeleton.py --state docs/figs/my.json --out docs/figs/my.png   # 界面里保存的状态
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import glove_model as gm  # noqa: E402
import skeleton_lib as sk  # noqa: E402

VIEWS = {"palm_front": (180, -8), "dorsal": (0, -8), "thumb_side": (250, -5), "palm_oblique": (215, -25)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--state", help="skeleton_gui.py 保存的 json；给了就只渲染这一张")
    ap.add_argument("--out", default=None)
    ap.add_argument("--alpha", type=float, default=0.22)
    ap.add_argument("--no-mesh", action="store_true")
    ap.add_argument("--bone-mm", type=float, default=4.5)
    ap.add_argument("--axis-mm", type=float, default=45.0)
    ap.add_argument("--pose", choices=["zero", "fist", "half"], default="zero")
    ap.add_argument("--size", type=int, nargs=2, default=[2000, 1500])
    args = ap.parse_args()
    W, H = args.size
    figs = os.path.join(gm.ROOT, "docs", "figs")
    if args.state:
        st = sk.load_state(args.state)
        out = args.out or os.path.splitext(args.state)[0] + ".png"
        print("*", sk.render_png(st, out, W, H))
        return
    st = json.loads(json.dumps(sk.DEFAULT_STATE))
    st.update(mesh_alpha=args.alpha, show_mesh=not args.no_mesh, bone_mm=args.bone_mm, axis_mm=args.axis_mm)
    amp = {"zero": 0.0, "fist": 60.0, "half": 30.0}[args.pose]
    st["angles_deg"] = {j: (amp * 0.5 if "_abd_" in j else amp) for j in st["angles_deg"]}
    for name, (az, el) in VIEWS.items():
        st["cam"] = dict(azimuth=az, elevation=el, distance=None, lookat=None)
        out = args.out or os.path.join(figs, f"skeleton_{args.pose}_{name}{'_nomesh' if args.no_mesh else ''}.png")
        print("*", sk.render_png(st, out, W, H))
        if args.out:
            break


if __name__ == "__main__":
    main()
