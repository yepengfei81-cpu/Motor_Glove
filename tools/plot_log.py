#!/usr/bin/env python3
"""
把 bilateral.py --log 记录的 CSV 画成图，并估计接触时的等效刚度。

  python tools/plot_log.py data/grasp_soft.csv                    # 一次记录：时间序列 + 力矩-位置差散点
  python tools/plot_log.py data/grasp_soft.csv data/grasp_firm.csv  # 两次对比（轻捏 vs 用力捏）
  python tools/plot_log.py data/*.csv --joint ring_pip_motor      # 只看一个关节

每个关节输出：
  - 从手力矩 τ_F（从手电流 × Kt）随时间的变化：自由运动时 ≈ 0，碰到东西后上升 = 抓握力曲线
  - 接触阶段 τ_F 对位置差 e = q_主 − q_从 的散点和线性拟合。斜率 = 这次采集时从手的有效刚度
    （由从手位置环增益决定，用 --follower-kp 改；电流到上限后曲线变平）。
    所以：轻捏 / 用力捏 的区别体现在力矩的大小和时间曲线上；软 / 硬 的区别体现在斜率上。
"""
import argparse
import csv
import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dxl_bus import KT, MODEL_NAME  # noqa: E402
from figtext import T, LANG  # noqa: E402

NAME_TO_MODEL = {v: k for k, v in MODEL_NAME.items()}
plt.rcParams["font.family"] = ["DejaVu Sans", "Noto Sans CJK JP"] if LANG == "en" else ["Noto Sans CJK JP", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
CONTACT_MA = 25     # 从手电流超过这个值算接触


def load(path):
    with open(path) as f:
        rows = list(csv.reader(f))
    header, models_row = rows[0], rows[1]
    settings = {}
    data = rows[2:]
    if data and data[0] and str(data[0][0]).startswith("# settings"):
        settings = dict(x.split(":") for x in data[0][1:]); data = data[1:]
    joints = list(dict.fromkeys(h.split(":")[0] for h in header[1:]))   # 按出现顺序去重
    models = {m.split(":")[0]: m.split(":")[1] for m in models_row[1:]}
    arr = np.array([[float(x) for x in r] for r in data])
    col = {h: k for k, h in enumerate(header)}
    out = {"t": arr[:, 0], "joints": joints, "models": models, "settings": settings}
    for j in joints:
        g = lambda k: arr[:, col[f"{j}:{k}"]]
        kt = KT[NAME_TO_MODEL.get(models[j].replace("(12V)", ""), 1200)]
        out[j] = dict(qA=g("qA"), qB=g("qB"), iA=g("iA_meas"), iAcmd=g("iA_cmd"), iB=g("iB_meas"),
                      tauB=g("iB_meas") / 1000 * kt, tauA=g("iA_cmd") / 1000 * kt,
                      kpB=g("kpB") if f"{j}:kpB" in col else None)
    return out


def stiffness(d, j):
    """接触阶段：τ_F 对 e = qA − qB 的最小二乘斜率"""
    e = d[j]["qA"] - d[j]["qB"]
    tau = np.abs(d[j]["tauB"])
    mask = np.abs(d[j]["iB"]) > CONTACT_MA
    if mask.sum() < 20:
        return None, mask
    x, y = np.abs(e[mask]), tau[mask]
    A = np.vstack([x, np.ones_like(x)]).T
    k, b = np.linalg.lstsq(A, y, rcond=None)[0]
    return float(k), mask


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--joint", action="append", help="只画这些关节（可重复）")
    ap.add_argument("--out", default=None, help="输出图片路径（默认和第一个 CSV 同名 .png）")
    ap.add_argument("--panels", action="store_true", help="再把每个面板单独存成 <out>_p1.png, _p2.png …（放 PPT 用）")
    args = ap.parse_args()
    runs = [(os.path.splitext(os.path.basename(p))[0], load(p)) for p in args.logs]
    joints = args.joint or [j for j in runs[0][1]["joints"] if any(np.abs(d[j]["iB"]).max() > CONTACT_MA for _, d in runs)]
    if not joints:
        joints = runs[0][1]["joints"][:3]
        print("没有关节检测到接触（从手电流都低于 %d mA），画前三个关节" % CONTACT_MA)
    has_kp = any(d[j].get("kpB") is not None and np.ptp(d[j]["kpB"]) > 1 for _, d in runs for j in joints)
    ncol = 4 if has_kp else 3
    fig, axes = plt.subplots(len(joints), ncol, figsize=(5 * ncol, 3.2 * len(joints)), squeeze=False)
    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]
    summary = []
    for r, j in enumerate(joints):
        for c, (name, d) in enumerate(runs):
            t = d["t"]; col = colors[c % len(colors)]
            ax = axes[r, 0]
            ax.plot(t, np.degrees(d[j]["qA"]), color=col, lw=1.2, label=f"{name} {T('pl_leader')}")
            ax.plot(t, np.degrees(d[j]["qB"]), color=col, lw=1.2, ls="--", label=f"{name} {T('pl_follower')}")
            ax.set_ylabel(T("pl_angle")); ax.set_title(T("pl_t1", j=j), fontsize=10)
            ax = axes[r, 1]
            ax.plot(t, d[j]["tauB"] * 1000, color=col, lw=1.2, label=f"{name} {T('pl_tauB')}")
            ax.plot(t, -d[j]["tauA"] * 1000, color=col, lw=0.8, ls=":", label=f"{name} {T('pl_tauA')}")
            ax.set_ylabel(T("pl_torque")); ax.set_title(T("pl_t2"), fontsize=10)
            k, mask = stiffness(d, j)
            ax = axes[r, 2]
            e = np.degrees(np.abs(d[j]["qA"] - d[j]["qB"]))
            tag = f"{name}" + (f" (Kp={d['settings']['follower_kp']})" if d["settings"].get("follower_kp") else "")
            ax.scatter(e[mask], np.abs(d[j]["tauB"][mask]) * 1000, s=6, color=col, alpha=0.5,
                       label=tag + (f"  {T('pl_slope')} {k:.2f} N·m/rad" if k else T("pl_nocontact")))
            if k:
                xs = np.linspace(0, e[mask].max(), 10)
                ax.plot(xs, k * np.radians(xs) * 1000, color=col, lw=1.5)
                summary.append((name, j, k, float(np.abs(d[j]["tauB"][mask]).max())))
            ax.set_xlabel(T("pl_x3")); ax.set_ylabel(T("pl_y3")); ax.set_title(T("pl_t3"), fontsize=10)
            if has_kp and d[j].get("kpB") is not None:
                ax = axes[r, 3]
                ax.plot(t, d[j]["kpB"], color=col, lw=1.5, label=f"{name} {T('pl_kp')}")
                ax2 = ax.twinx() if c == 0 else ax._twin
                ax._twin = ax2
                ax2.plot(t, np.abs(d[j]["iB"]), color=col, lw=0.8, ls=":", label=f"{name} {T('pl_iB')}")
                ax.set_ylabel(T("pl_kp")); ax2.set_ylabel(T("pl_iB")); ax.set_xlabel(T("pl_time")); ax.set_title(T("pl_t4"), fontsize=10)
                h1, l1 = ax.get_legend_handles_labels(); h2, l2 = ax2.get_legend_handles_labels()
                ax.legend(h1 + h2, l1 + l2, fontsize=8)
        for c in range(3):
            axes[r, c].grid(alpha=0.3); axes[r, c].legend(fontsize=8)
        if has_kp:
            axes[r, 3].grid(alpha=0.3)
        axes[r, 0].set_xlabel(T("pl_time")); axes[r, 1].set_xlabel(T("pl_time"))
    fig.tight_layout()
    out = args.out or os.path.splitext(args.logs[0])[0] + ".png"
    fig.savefig(out, dpi=150)
    print("* 图:", out)
    if args.panels:
        fig.canvas.draw()
        base = os.path.splitext(out)[0]
        for r in range(len(joints)):
            for c in range(ncol):
                ax = axes[r, c]
                bb = ax.get_tightbbox(fig.canvas.get_renderer())
                if hasattr(ax, "_twin"):
                    bb = bb.union([bb, ax._twin.get_tightbbox(fig.canvas.get_renderer())])
                bb = bb.transformed(fig.dpi_scale_trans.inverted()).expanded(1.03, 1.05)
                name = f"{base}_p{c + 1}.png" if len(joints) == 1 else f"{base}_{joints[r]}_p{c + 1}.png"
                fig.savefig(name, dpi=220, bbox_inches=bb)
                print("  面板:", name)
    for name, j, k, tmax in summary:
        print(f"  {name:16s} {j:22s} 力矩-位置差斜率 {k:.2f} N·m/rad，最大接触力矩 {tmax*1000:.0f} mN·m")


if __name__ == "__main__":
    main()
