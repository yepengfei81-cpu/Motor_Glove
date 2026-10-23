#!/usr/bin/env python3
"""
对比两份（或多份）bilateral.py 的日志：同样的参数、同样的动作，只改控制频率（或别的什么），用数据说话。

  python tools/compare_logs.py data/bench_hz100.csv data/bench_hz800.csv
  python tools/compare_logs.py a.csv b.csv --joint thumb_cmc_abd_motor --out data/compare.png

每份日志算：
  环周期：均值 / p99 / 最大（实际跑到的频率）
  自由运动跟踪：门控关着且主手在动时，从手落后主手的 RMS（°）和互相关估的延迟（ms）
  接触：门控打开的时间占比、门控开关次数（每分钟）、接触段反馈力矩的高频噪声（mN·m）、主手电流指令噪声（mA）、
        接触段主手速度抖动（rad/s）、反馈力矩的频谱（哪一段频率在抖）
图：每份日志一列（角度 / 位置差+门控 / 反馈力矩+电流 / 环周期），最后一行是接触段力矩频谱叠在一起。
"""
import argparse
import csv
import os
import sys

import numpy as np

SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]        # dataviz 参考调色板，按固定顺序
INK, INK2, GRID = "#0b0b0b", "#52514e", "#e6e5e1"


def load(path, joint=None):
    rows = list(csv.reader(open(path)))
    head = rows[0]
    data = [r for r in rows[1:] if r and not r[0].startswith("#")]
    t = np.array([float(r[0]) for r in data])
    cols = {}
    for k, name in enumerate(head[1:], 1):
        j, key = name.split(":")
        cols.setdefault(j, {})[key] = np.array([float(r[k]) for r in data])
    if joint is None:
        joint = next(iter(cols))
    if joint not in cols:
        sys.exit(f"{path} 里没有关节 {joint}，有: {list(cols)}")
    return t, cols[joint], joint


def hp_noise(x, t, win_s=0.05):
    """高频噪声：减掉 win_s 的滑动平均后的标准差"""
    if len(x) < 10:
        return float("nan")
    n = max(3, int(win_s / np.median(np.diff(t))))
    k = np.ones(n) / n
    base = np.convolve(x, k, mode="same")
    return float(np.std(x - base))


def lag_ms(t, qa, qb):
    """互相关估从手落后主手的延迟（去均值，限制在 0–200 ms）"""
    dt = np.median(np.diff(t))
    a, b = qa - qa.mean(), qb - qb.mean()
    if np.std(a) < 1e-3 or np.std(b) < 1e-3:
        return float("nan")
    maxlag = int(0.2 / dt)
    best, bl = -1e9, 0
    for L in range(0, maxlag):
        c = np.dot(a[: len(a) - L], b[L:]) if L else np.dot(a, b)
        if c > best:
            best, bl = c, L
    return bl * dt * 1000


def analyze(t, d):
    dt = np.diff(t) * 1000
    gate = d["gate"]; err = d["err"]; dqA = d["dqA"]
    free = (gate < 0.5) & (np.abs(dqA) > 0.3)
    contact = gate > 0.5
    trans = int(np.sum(np.abs(np.diff(contact.astype(int)))))
    # 等效延迟：自由运动里 位置差 ≈ 延迟 × 主手速度，回归斜率就是延迟，不受你动得快慢影响
    v = dqA[free]; e = err[free]
    eq_delay = 1000 * float(np.dot(e, v) / np.dot(v, v)) if free.sum() > 10 and np.dot(v, v) > 0 else float("nan")
    tau_c = d["tau_fb"][contact]
    noise = hp_noise(tau_c, t[contact])
    mean_tau = float(np.mean(np.abs(tau_c))) if contact.sum() > 10 else float("nan")
    # 稳健的延迟估计：|v| > 1 rad/s 时 |err|/|v| 的中位数，不受个别大跳变影响
    big = free & (np.abs(dqA) > 1.0)
    robust_delay = 1000 * float(np.median(np.abs(err[big]) / np.abs(dqA[big]))) if big.sum() > 10 else float("nan")
    # 接触段的振荡：极限环会把门控打得一开一关，所以把接触段前后各扩 0.5 s 当"接触窗口"，
    # 在窗口里看位置差去掉 0.5 s 滑动平均后的标准差（慢慢用力推进去不算，来回摆才算）
    dts = np.median(np.diff(t)); k = max(1, int(0.5 / dts))
    win = np.convolve(contact.astype(float), np.ones(2 * k + 1), mode="same") > 0
    base = np.convolve(err, np.ones(k) / k, mode="same")
    osc_std = np.degrees(float(np.std((err - base)[win]))) if win.sum() > 10 else float("nan")
    f_, p_ = spectrum(t, err, win)
    band = 100 * float(p_[(f_ >= 1) & (f_ <= 10)].sum() / p_[f_ >= 1].sum()) if f_ is not None and p_[f_ >= 1].sum() > 0 else float("nan")
    tau_w = d["tau_fb"][win]
    osc_tau = 1000 * float(np.std((d["tau_fb"] - np.convolve(d["tau_fb"], np.ones(k) / k, mode="same"))[win])) if win.sum() > 10 else float("nan")
    out = {
        "周期均值 ms": dt.mean(), "周期 p99 ms": np.percentile(dt, 99), "周期最大 ms": dt.max(), "实际 Hz": 1000 / dt.mean(),
        "时长 s": t[-1] - t[0],
        "自由运动跟踪 RMS °": np.degrees(np.sqrt(np.mean(err[free] ** 2))) if free.sum() > 10 else float("nan"),
        "自由运动主手速度 RMS rad/s": float(np.sqrt(np.mean(v ** 2))) if free.sum() > 10 else float("nan"),
        "等效延迟（位置差/速度）ms": eq_delay,
        "等效延迟（中位数，稳健）ms": robust_delay,
        "接触窗口位置差振荡 std °": osc_std,
        "接触窗口力矩振荡 std mN·m": osc_tau,
        "接触窗口位置差 1–10 Hz 功率占比 %": band,
        "接触时间占比 %": 100 * contact.mean(), "门控开关次/分钟": trans / ((t[-1] - t[0]) / 60),
        "接触段力矩噪声 mN·m": 1000 * noise,
        "接触段力矩噪声/均值 %": 100 * noise / mean_tau if mean_tau and mean_tau > 0 else float("nan"),
        "接触段电流指令噪声 mA": hp_noise(d["iA_cmd"][contact], t[contact]),
        "接触段主手速度抖动 rad/s": float(np.std(dqA[contact])) if contact.sum() > 10 else float("nan"),
        "接触段力矩均值 mN·m": 1000 * float(np.mean(np.abs(d["tau_fb"][contact]))) if contact.sum() > 10 else float("nan"),
    }
    return out, free, contact


def spectrum(t, x, mask):
    """接触段反馈力矩的功率谱（均匀重采样到中位周期）"""
    if mask.sum() < 64:
        return None, None
    dt = np.median(np.diff(t))
    tt = t[mask]; xx = x[mask]
    tu = np.arange(tt[0], tt[-1], dt); xu = np.interp(tu, tt, xx)
    xu = xu - xu.mean()
    f = np.fft.rfftfreq(len(xu), dt)
    p = np.abs(np.fft.rfft(xu * np.hanning(len(xu)))) ** 2 / len(xu)
    return f, p


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--joint", default=None)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.family"] = ["Noto Sans CJK JP", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
    runs = []
    for p in args.logs:
        t, d, joint = load(p, args.joint)
        stats, free, contact = analyze(t, d)
        runs.append((os.path.basename(p).replace(".csv", ""), t, d, stats, free, contact))
    # ---- 表 ----
    keys = list(runs[0][3])
    w = max(len(k) for k in keys) + 2
    print(f"关节 {joint}\n" + " " * w + "".join(f"{r[0]:>18s}" for r in runs))
    for k in keys:
        print(f"{k:{w}s}" + "".join(f"{r[3][k]:18.2f}" for r in runs))
    # ---- 图 ----
    n = len(runs)
    fig, axes = plt.subplots(5, n, figsize=(6.2 * n, 13), squeeze=False, gridspec_kw={"height_ratios": [1, 1, 1, 0.8, 1]})
    for ax in axes.flat:
        ax.set_facecolor("#fcfcfb"); ax.grid(True, color=GRID, lw=0.8); ax.tick_params(colors=INK2, labelsize=9)
        for s in ax.spines.values(): s.set_color(GRID)
    for c, (name, t, d, st, free, contact) in enumerate(runs):
        col = SERIES[c % len(SERIES)]
        t0 = t - t[0]
        ax = axes[0][c]
        ax.plot(t0, np.degrees(d["qA"]), color=col, lw=1.4, label="主手")
        ax.plot(t0, np.degrees(d["qB"]), color=INK2, lw=1.0, ls="--", label="从手")
        ax.set_title(f"{name}  （{st['实际 Hz']:.0f} Hz）", color=INK, fontsize=11, loc="left")
        ax.set_ylabel("角度 °", color=INK2); ax.legend(frameon=False, fontsize=9)
        ax = axes[1][c]
        ax.plot(t0, np.degrees(d["err"]), color=col, lw=1.0, label="主手 − 从手")
        ax.fill_between(t0, 0, d["gate"] * np.nanmax(np.abs(np.degrees(d["err"]))) if np.nanmax(np.abs(d["err"])) > 0 else 0,
                        color=INK2, alpha=0.12, lw=0, label="接触门控")
        ax.set_ylabel("位置差 °", color=INK2); ax.legend(frameon=False, fontsize=9)
        ax = axes[2][c]
        ax.plot(t0, 1000 * d["tau_fb"], color=col, lw=1.0, label="反馈力矩 mN·m")
        ax.set_ylabel("反馈力矩 mN·m", color=INK2); ax.legend(frameon=False, fontsize=9)
        ax = axes[3][c]
        ax.plot(t0[1:], np.diff(t) * 1000, color=col, lw=0.8)
        ax.set_ylabel("环周期 ms", color=INK2); ax.set_xlabel("时间 s", color=INK2)
        ax.set_ylim(0, max(3, np.percentile(np.diff(t) * 1000, 99.5) * 1.3))
    axs = axes[4][0]
    for c, (name, t, d, st, free, contact) in enumerate(runs):
        f, p = spectrum(t, d["tau_fb"], contact)
        if f is not None:
            m = (f > 0.5) & (f < 150)
            axs.semilogy(f[m], p[m] * 1e6, color=SERIES[c % len(SERIES)], lw=1.2, label=name)
    axs.set_xlabel("频率 Hz", color=INK2); axs.set_ylabel("接触段力矩功率谱", color=INK2)
    axs.set_title("接触段反馈力矩频谱（曲线越低越平稳）", color=INK, fontsize=11, loc="left"); axs.legend(frameon=False, fontsize=9)
    for c in range(1, n):
        axes[4][c].axis("off")
    fig.tight_layout()
    out = args.out or os.path.join(os.path.dirname(args.logs[0]) or ".", "compare_" + "_vs_".join(r[0] for r in runs) + ".png")
    fig.savefig(out, dpi=150, facecolor="#fcfcfb")
    print(f"\n图已存 {out}")


if __name__ == "__main__":
    main()
