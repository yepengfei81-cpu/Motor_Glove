#!/usr/bin/env python3
"""
PPT 用的示意图（matplotlib，白底，输出到 docs/figs/ppt/）：
  system_diagram.png    系统框图：主手套 → OpenRB → PC（MuJoCo 孪生 / 策略）→ 从手套，位置正向、力反向
  control_law.png       力反馈控制律：力矩 vs 位置差（死区、刚度、饱和、单向）
  data_record.png       每个周期记录什么（位置 / 力 / 阻抗标签怎么来）
"""
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import glove_model as gm  # noqa: E402
from figtext import T, LANG  # noqa: E402

OUT = os.path.join(gm.ROOT, "docs", "figs", "ppt")
plt.rcParams["font.family"] = ["DejaVu Sans", "Noto Sans CJK JP"] if LANG == "en" else ["Noto Sans CJK JP", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
BLUE, RED, GREY, GREEN = "#2b6cb0", "#c53030", "#4a5568", "#2f855a"


def box(ax, x, y, w, h, title, lines, color=GREY):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.3", fc="white", ec=color, lw=2.2))
    ax.text(x + w / 2, y + h - 0.45, title, ha="center", va="center", fontsize=13.5 if LANG == "en" else 15, fontweight="bold", color=color)
    for i, t in enumerate(lines):
        ax.text(x + w / 2, y + h - 1.05 - 0.55 * i, t, ha="center", va="center", fontsize=10.5 if LANG == "en" else 11.5, color="#2d3748")


def arrow(ax, p, q, text, color, above=True, rad=0.0):
    ax.add_patch(FancyArrowPatch(p, q, arrowstyle="-|>", mutation_scale=22, lw=2.4, color=color, connectionstyle=f"arc3,rad={rad}"))
    mx, my = (p[0] + q[0]) / 2, (p[1] + q[1]) / 2 + (0.42 if above else -0.42)
    ax.text(mx, my, text, ha="center", va="center", fontsize=11.5, color=color, fontweight="bold")


def system_diagram():
    fig, ax = plt.subplots(figsize=(18, 7.2))
    ax.set_xlim(0, 18); ax.set_ylim(0, 7.2); ax.axis("off")
    box(ax, 0.3, 2.2, 3.7, 3.0, T("sys_leader"), T("sys_leader_l"), BLUE)
    box(ax, 5.3, 2.2, 2.7, 3.0, T("sys_board"), T("sys_board_l"), GREY)
    box(ax, 9.4, 2.2, 3.6, 3.0, T("sys_pc"), T("sys_pc_l"), GREEN)
    box(ax, 14.4, 2.2, 3.4, 3.0, T("sys_follower"), T("sys_follower_l"), RED)
    arrow(ax, (4.0, 4.3), (5.3, 4.3), "", BLUE); arrow(ax, (8.0, 4.3), (9.4, 4.3), T("sys_q"), BLUE)
    arrow(ax, (13.0, 4.3), (14.4, 4.3), T("sys_qstar"), BLUE)
    arrow(ax, (14.4, 3.0), (13.0, 3.0), T("sys_back"), RED, above=False); arrow(ax, (9.4, 3.0), (8.0, 3.0), T("sys_i"), RED, above=False); arrow(ax, (5.3, 3.0), (4.0, 3.0), "", RED, above=False)
    ax.text(9, 6.6, T("sys_title"), ha="center", fontsize=16, fontweight="bold")
    ax.text(9, 1.4, T("sys_note1"), ha="center", fontsize=12.5, color="#2d3748")
    ax.text(9, 0.7, T("sys_note2"), ha="center", fontsize=12, color="#718096")
    fig.savefig(os.path.join(OUT, "system_diagram.png"), dpi=180, bbox_inches="tight")
    plt.close(fig)


def control_law():
    import numpy as np
    kp, db, tmax = 0.4, 2.0, 0.06
    e = np.linspace(-30, 30, 601)
    tau = np.where(np.abs(e) > db, -kp * (np.abs(e) - db) * np.sign(e) * math.pi / 180, 0)
    tau = np.clip(tau, -tmax, tmax)
    fig, ax = plt.subplots(figsize=(9, 5.4))
    ax.plot(e, tau * 1000, color=RED, lw=3, label=T("law_line"))
    ax.axvspan(-db, db, color="#edf2f7", label=T("law_db"))
    ax.axhline(tmax * 1000, color=GREY, ls="--", lw=1.2); ax.axhline(-tmax * 1000, color=GREY, ls="--", lw=1.2)
    ax.text(-29, -tmax * 1000 + 4, T("law_lim"), ha="left", fontsize=11, color=GREY)
    ax.annotate(T("law_slope"), xy=(9, -0.4 * (9 - 2) * math.pi / 180 * 1000), xytext=(13, -12), fontsize=12, arrowprops=dict(arrowstyle="->", color=GREY))
    ax.set_xlabel(T("law_x"), fontsize=13)
    ax.set_ylabel(T("law_y"), fontsize=13)
    ax.set_title(T("law_title"), fontsize=14, fontweight="bold")
    ax.set_ylim(-75, 75); ax.grid(alpha=0.3); ax.legend(loc="center left", fontsize=11)
    fig.savefig(os.path.join(OUT, "control_law.png"), dpi=180, bbox_inches="tight")
    plt.close(fig)


def data_record():
    fig, ax = plt.subplots(figsize=(18, 6.6))
    ax.set_xlim(0, 18); ax.set_ylim(0, 6.6); ax.axis("off")
    ax.text(9, 6.1, T("rec_title"), ha="center", fontsize=16, fontweight="bold")
    box(ax, 0.3, 2.0, 3.9, 3.3, T("rec_log"), T("rec_log_l"), BLUE)
    box(ax, 4.9, 3.4, 4.6, 1.95, T("rec_1"), T("rec_1_l"), GREEN)
    box(ax, 4.9, 1.15, 4.6, 1.95, T("rec_2"), T("rec_2_l"), GREEN)
    box(ax, 10.2, 2.0, 4.2, 3.3, T("rec_3"), T("rec_3_l"), RED)
    box(ax, 15.1, 2.0, 2.7, 3.3, T("rec_dep"), T("rec_dep_l"), GREY)
    for y in (4.4, 2.1):
        arrow(ax, (4.2, y), (4.9, y), "", GREY)
    arrow(ax, (9.5, 3.3), (10.2, 3.3), "", GREY); arrow(ax, (14.4, 3.65), (15.1, 3.65), "", GREY)
    ax.text(9, 0.5, T("rec_refs"), ha="center", fontsize=11.5, color="#718096")
    fig.savefig(os.path.join(OUT, "data_record.png"), dpi=180, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    system_diagram(); control_law(); data_record()
    print("* system_diagram.png, control_law.png, data_record.png")
