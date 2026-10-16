#!/usr/bin/env python3
"""
PPT 用的打分卡和对比表（matplotlib，白底，输出到 docs/figs/ppt/）。文字语言跟 figtext 一样由 FIG_LANG 控制（默认英文）。
  scorecard.png            4 条标准 × 6 个系统 的总打分卡
  scorecard_<system>.png   每个系统的小卡片（放在对应那页的右上角）
  comparison_table.png     6 列对比表
"""
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Circle, Wedge

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import glove_model as gm  # noqa: E402
from figtext import LANG  # noqa: E402

OUT = os.path.join(gm.ROOT, "docs", "figs", "ppt")
plt.rcParams["font.family"] = ["DejaVu Sans", "Noto Sans CJK JP"] if LANG == "en" else ["Noto Sans CJK JP", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
GREEN, AMBER, RED, GREY, INK, LIGHT = "#2f855a", "#d69e2e", "#c53030", "#718096", "#1a202c", "#edf2f7"
BLUE = "#2b6cb0"

E = LANG == "en"
SYSTEMS = ["UMI", "DexUMI", "ART-Glove", "DITTO", "DITTO-X", "Ours" if E else "我们"]
KEYS = ["umi", "dexumi", "artglove", "ditto", "dittox", "ours"]
CRITERIA = [
    ("Dexterous", "enough independent DoF") if E else ("灵巧", "足够多的独立自由度"),
    ("No embodiment gap", "human motion runs 1:1 on the robot") if E else ("无具身差距", "人的动作机器人能原样执行"),
    ("Operator feels contact", "real or rendered force") if E else ("操作者感到接触", "真实或渲染的力"),
    ("Robot in the loop", "takeover, DAgger, actuator dynamics") if E else ("机器人在环", "可接管、DAgger、执行器动力学"),
]
# 分数: 1 = ✓, 0.5 = 半, 0 = ✗ ；备注是每格下面的小字
SCORES = {
    "umi":      [(0, "parallel gripper"), (1, "same gripper"), (1, "real contact"), (0, "")],
    "dexumi":   [(0.5, "Inspire 6 / XHand 12 act."), (0.5, "fingertips only"), (1, "real contact"), (0, "")],
    "artglove": [(1, "22 DoF measured"), (0, "retargeting needed"), (1, "real contact"), (0, "measurement only")],
    "ditto":    [(0.5, "2 fingers, 7 DoF"), (1, "1:1 joints"), (1, "rendered joint force"), (1, "")],
    "dittox":   [(0.5, "3 fingers, ~9 motors"), (0.5, "thumb retargeted"), (1, "joint force + vibration"), (1, "reverse teleop, DAgger")],
    "ours":     [(1, "4 fingers, 13 DoF"), (1, "all 13 joints 1:1"), (1, "rendered joint force"), (0.5, "sim / glove #2 today")],
}
if not E:
    SCORES = {
        "umi":      [(0, "平行夹爪"), (1, "同一套夹爪"), (1, "真实接触"), (0, "")],
        "dexumi":   [(0.5, "Inspire 6 / XHand 12 主动"), (0.5, "只匹配指尖"), (1, "真实接触"), (0, "")],
        "artglove": [(1, "测 22 自由度"), (0, "需要重定向"), (1, "真实接触"), (0, "只测量")],
        "ditto":    [(0.5, "2 指 7 自由度"), (1, "关节 1:1"), (1, "渲染关节力"), (1, "")],
        "dittox":   [(0.5, "3 指约 9 电机"), (0.5, "拇指重定向"), (1, "关节力 + 振动"), (1, "反向遥操作、DAgger")],
        "ours":     [(1, "4 指 13 自由度"), (1, "13 关节全 1:1"), (1, "渲染关节力"), (0.5, "目前仿真 / 第二只手套")],
    }


def chip(ax, x, y, score, r=0.22):
    """圆形打分标记：满 = 绿勾，半 = 琥珀半圆，零 = 红叉。"""
    if score >= 1:
        ax.add_patch(Circle((x, y), r, fc=GREEN, ec="none"))
        ax.text(x, y - 0.01, "✓", ha="center", va="center", fontsize=r * 70, color="white", fontweight="bold")
    elif score > 0:
        ax.add_patch(Circle((x, y), r, fc="white", ec=AMBER, lw=2.5))
        ax.add_patch(Wedge((x, y), r, 90, 270, fc=AMBER, ec="none"))
    else:
        ax.add_patch(Circle((x, y), r, fc=RED, ec="none"))
        ax.text(x, y - 0.01, "✗", ha="center", va="center", fontsize=r * 70, color="white", fontweight="bold")


def scorecard():
    nc = len(SYSTEMS)
    cw, rh, x0, y0 = 2.3, 1.25, 4.6, 0.6
    W, H = x0 + cw * nc + 0.3, y0 + rh * 4 + 1.3
    fig, ax = plt.subplots(figsize=(W, H)); ax.set_xlim(0, W); ax.set_ylim(0, H); ax.axis("off")
    ax.text(0.3, H - 0.55, "Four requirements for good dexterous-manipulation data" if E else "好的灵巧操作数据要满足的四个要求",
            fontsize=17, fontweight="bold", color=INK, va="center")
    for j, s in enumerate(SYSTEMS):
        cx = x0 + cw * (j + 0.5)
        bold = KEYS[j] == "ours"
        ax.add_patch(FancyBboxPatch((cx - cw / 2 + 0.06, y0 + rh * 4 - 0.02), cw - 0.12, 0.6, boxstyle="round,pad=0.01,rounding_size=0.12",
                                    fc=BLUE if bold else LIGHT, ec="none"))
        ax.text(cx, y0 + rh * 4 + 0.28, s, ha="center", va="center", fontsize=13.5, fontweight="bold", color="white" if bold else INK)
    for i, (name, desc) in enumerate(CRITERIA):
        cy = y0 + rh * (3 - i) + rh / 2
        if i % 2 == 0:
            ax.add_patch(FancyBboxPatch((0.2, cy - rh / 2 + 0.03), W - 0.4, rh - 0.06, boxstyle="round,pad=0.01,rounding_size=0.1", fc="#f7fafc", ec="none"))
        ax.text(0.4, cy + 0.17, f"{'①②③④'[i]} {name}", fontsize=13, fontweight="bold", color=INK, va="center")
        ax.text(0.4, cy - 0.22, desc, fontsize=10.5, color=GREY, va="center")
        for j, k in enumerate(KEYS):
            cx = x0 + cw * (j + 0.5)
            sc, note = SCORES[k][i]
            chip(ax, cx, cy + 0.14, sc)
            ax.text(cx, cy - 0.33, note, ha="center", va="center", fontsize=8.6, color=GREY)
    fig.savefig(os.path.join(OUT, "scorecard.png"), dpi=200, bbox_inches="tight"); plt.close(fig)


def mini_cards():
    """每页右上角用的 2×2 小卡：标题 + 四个标准的勾/半/叉。"""
    short = [c[0] for c in CRITERIA]
    for s, k in zip(SYSTEMS, KEYS):
        W, H = 5.8, 2.9
        fig, ax = plt.subplots(figsize=(W, H)); ax.set_xlim(0, W); ax.set_ylim(0, H); ax.axis("off")
        ax.add_patch(FancyBboxPatch((0.08, 0.08), W - 0.16, H - 0.16, boxstyle="round,pad=0.02,rounding_size=0.2", fc="white", ec=BLUE if k == "ours" else GREY, lw=2.2))
        ax.text(W / 2, H - 0.42, s, ha="center", va="center", fontsize=14, fontweight="bold", color=BLUE if k == "ours" else INK)
        for i in range(4):
            col, row = i % 2, i // 2
            cx, cy = 0.5 + col * 2.9, H - 1.05 - row * 0.9
            chip(ax, cx, cy, SCORES[k][i][0], r=0.2)
            ax.text(cx + 0.32, cy, f"{'①②③④'[i]} {short[i]}", va="center", fontsize=9.6, color=INK)
        fig.savefig(os.path.join(OUT, f"scorecard_{k}.png"), dpi=200, bbox_inches="tight"); plt.close(fig)


def comparison_table():
    rows = [
        ("Dexterity", ["parallel gripper", "Inspire 6 active\n/ XHand 12", "22 DoF measured", "2 fingers, 7 DoF", "3 fingers\n(~9 motors)", "4 fingers, 13 DoF"]),
        ("Correspondence\nto the robot", ["identical gripper", "fingertips only", "none\n(retargeting)", "1:1", "index, middle 1:1;\nthumb retargeted", "all 13 joints 1:1"]),
        ("What the\noperator feels", ["real contact", "real contact", "real contact", "rendered joint force", "joint force +\nfingertip vibration", "rendered joint force"]),
        ("Robot in the loop", ["no", "no", "no", "yes", "yes, reverse teleop", "yes (sim / glove #2;\nrobot hand next)"]),
        ("Policy results", ["yes", "yes, 86% avg.", "none", "yes", "yes, with DAgger", "not yet"]),
    ] if E else [
        ("灵巧度", ["平行夹爪", "Inspire 6 主动\n/ XHand 12", "测 22 自由度", "2 指 7 自由度", "3 指\n（约 9 电机）", "4 指 13 自由度"]),
        ("和机器人的对应", ["夹爪相同", "只匹配指尖", "无（要重定向）", "1:1", "食指中指 1:1\n拇指重定向", "13 关节全 1:1"]),
        ("操作者感到的力", ["真实接触", "真实接触", "真实接触", "渲染的关节力", "关节力 +\n指尖振动", "渲染的关节力"]),
        ("机器人在环", ["否", "否", "否", "是", "是，有反向遥操作", "是（仿真 / 第二只手套；\n机械手是下一步）"]),
        ("策略结果", ["有", "有，平均 86%", "无", "有", "有，含 DAgger", "还没有"]),
    ]
    cw, rh, x0, y0 = 2.55, 1.05, 2.6, 0.3
    W, H = x0 + cw * 6 + 0.2, y0 + rh * 5 + 0.75
    fig, ax = plt.subplots(figsize=(W, H)); ax.set_xlim(0, W); ax.set_ylim(0, H); ax.axis("off")
    for j, s in enumerate(SYSTEMS):
        cx = x0 + cw * (j + 0.5); bold = KEYS[j] == "ours"
        ax.add_patch(FancyBboxPatch((cx - cw / 2 + 0.05, y0 + rh * 5 + 0.05), cw - 0.1, 0.55, boxstyle="round,pad=0.01,rounding_size=0.1", fc=BLUE if bold else LIGHT, ec="none"))
        ax.text(cx, y0 + rh * 5 + 0.33, s, ha="center", va="center", fontsize=13, fontweight="bold", color="white" if bold else INK)
    ax.add_patch(FancyBboxPatch((x0 + cw * 5 + 0.03, y0 + 0.02), cw - 0.06, rh * 5 - 0.04, boxstyle="round,pad=0.01,rounding_size=0.1", fc="#ebf4ff", ec="none"))
    for i, (name, cells) in enumerate(rows):
        cy = y0 + rh * (4 - i) + rh / 2
        if i % 2 == 0:
            ax.add_patch(FancyBboxPatch((0.15, cy - rh / 2 + 0.02), x0 + cw * 5 - 0.2, rh - 0.04, boxstyle="round,pad=0.01,rounding_size=0.08", fc="#f7fafc", ec="none"))
        ax.text(0.3, cy, name, fontsize=12, fontweight="bold", color=INK, va="center")
        for j, c in enumerate(cells):
            ax.text(x0 + cw * (j + 0.5), cy, c, ha="center", va="center", fontsize=10.2, color=INK, linespacing=1.25)
    fig.savefig(os.path.join(OUT, "comparison_table.png"), dpi=200, bbox_inches="tight"); plt.close(fig)


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    scorecard(); mini_cards(); comparison_table()
    print("* scorecard.png, scorecard_<system>.png ×6, comparison_table.png")
