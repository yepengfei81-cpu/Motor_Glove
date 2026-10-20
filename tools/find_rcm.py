#!/usr/bin/env python3
"""
求中指 MCP 双平行四边形的虚拟转轴（远程转动中心 RCM），用来在去掉连杆的机械手上放一个真实转轴并保持同构。

  python tools/find_rcm.py                   # 默认扫 -10° … 80°
  python tools/find_rcm.py --range -20 90    # 换扫描范围

做法：在 MuJoCo 模型（glove_mujoco/robot.xml，闭链由等式约束保证）里把 mid_mcp_motor 从小到大扫一遍，
记录近节（平行四边形的输出连杆，也就是 mid_mcp_abd 电机装的那块）在每个角度的位姿，
最小二乘找"在所有角度下位置都不变"的那条线 = 虚拟转轴。输出：
  1. 漂移：轴上的点在扫描过程中最多动了多少 mm（双平行四边形理想情况下是 0）
  2. 传动比：近节转角 / 电机转角（双平行四边形应为 1.000，等于 1 时机械手关节角 = 手套电机角，零点也相同）
  3. 位置：相对电机轴（mid_mcp_motor 配合的连接器：原点在电机轴上，Z 沿电机轴）的偏移，mm
     —— 同时给出直观分解：往指尖方向多少、往掌侧多少、沿轴向多少
  4. 在近节零件坐标系里的位置（转轴从近节零件的哪里穿过）
"""
import argparse
import math
import os
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import glove_model as gm  # noqa: E402

MOTOR, ABD, PIP = "mid_mcp_motor", "mid_mcp_abd_motor", "mid_pip_motor"


def rotvec(R):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, R.ravel())
    v = np.zeros(3)
    mujoco.mju_quat2Vel(v, q, 1.0)
    return v


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--range", type=float, nargs=2, default=[-10.0, 60.0], metavar=("FROM", "TO"),
                    help="扫描范围（°）。模型里 65° 以后连杆接近奇异位置，瞬心会大幅移动，先确认真手套能不能弯到那里")
    ap.add_argument("--step", type=float, default=5.0)
    args = ap.parse_args()

    m = mujoco.MjModel.from_xml_path(gm.MJCF)
    m.opt.disableflags = int(m.opt.disableflags) | int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    m.opt.gravity[:] = 0
    d = mujoco.MjData(m)
    md = gm.directions()
    jm, ja = m.joint(MOTOR).id, m.joint(ABD).id
    link = int(m.body_parentid[m.jnt_bodyid[ja]])          # 近节 = 平行四边形输出连杆
    motor_body = int(m.jnt_bodyid[jm])                     # 电机输出（曲柄）所在 body，坐标系 = 电机配合的连接器
    base = m.body("part_1").id
    act = m.actuator(MOTOR).id
    # 只看中指 MCP 这几个闭链（别的手指的闭链和这里无关）
    mid_eq = {e for e in range(m.neq) if "closing_mid_mcp" in (mujoco.mj_id2name(m, m.eq_objtype[e], m.eq_obj1id[e]) or "")}

    def settle():
        for _ in range(1500):
            mujoco.mj_step(m, d)
        mujoco.mj_forward(m, d)

    def to_base(p=None, v=None):
        """世界坐标 -> 基座 part_1 坐标：点要减原点再转，方向只转"""
        Rb, pb = d.xmat[base].reshape(3, 3), d.xpos[base]
        return Rb.T @ (p - pb) if p is not None else Rb.T @ v

    def link_pose(bid):
        Rb = d.xmat[base].reshape(3, 3)
        return Rb.T @ d.xmat[bid].reshape(3, 3), to_base(p=d.xpos[bid].copy())

    # 从 0° 往正方向扫，再回到 0° 往负方向扫，连杆每一步都是连续跟过去的
    pos = list(np.arange(0.0, args.range[1] + 1e-9, args.step))
    neg = list(np.arange(-args.step, args.range[0] - 1e-9, -args.step))
    Rs, ps, th, viol, ang_list = [], [], [], [], []
    zero = None
    for a in pos + neg:
        if neg and a == neg[0]:
            d.qpos[:] = zero["qpos"]; d.qvel[:] = 0
        d.ctrl[act] = md[MOTOR] * math.radians(a)
        settle()
        R, p = link_pose(link)
        Rs.append(R); ps.append(p); th.append(d.qpos[m.jnt_qposadr[jm]] * md[MOTOR]); ang_list.append(a)
        # 闭链两点在平面内的距离（沿铰链轴的偏差是 CAD 里连接器放在不同面上，不影响平面运动，不算）
        gaps = []
        for e in mid_eq:
            s1, s2 = m.eq_obj1id[e], m.eq_obj2id[e]
            g = d.site_xpos[s1] - d.site_xpos[s2]
            z = d.site_xmat[s1].reshape(3, 3)[:, 2]
            gaps.append(np.linalg.norm(g - z * np.dot(g, z)))
        viol.append(max(gaps) if gaps else 0.0)
        if a == 0.0:
            zero = dict(qpos=d.qpos.copy(), R=R, p=p, Rm=link_pose(motor_body)[0],
                        o=to_base(p=d.xanchor[jm].copy()), am=to_base(v=d.xaxis[jm].copy()),
                        pip=to_base(p=d.xanchor[m.joint(PIP).id].copy()), dors=to_base(v=gm.dorsal_normal(m, d)))
    Rs, ps, th = np.array(Rs), np.array(ps), np.array(th)
    n = len(Rs)

    # 1) 最小二乘：近节上的点 x（近节坐标）在所有姿态下世界位置都是 c：R_t x + p_t = c
    A = np.zeros((3 * n, 6)); b = np.zeros(3 * n)
    for t in range(n):
        A[3 * t:3 * t + 3, :3] = Rs[t]
        A[3 * t:3 * t + 3, 3:] = -np.eye(3)
        b[3 * t:3 * t + 3] = -ps[t]
    c = np.linalg.lstsq(A, b, rcond=None)[0][3:]
    R0, am, o = zero["R"], zero["am"], zero["o"]
    k = int(np.argmax([np.linalg.norm(rotvec(R @ R0.T)) for R in Rs]))
    av = rotvec(Rs[k] @ R0.T); av /= np.linalg.norm(av)
    if np.dot(av, am) < 0:
        av = -av
    c = c + av * np.dot(o - c, av)                          # 轴上离电机轴原点最近的点
    x_link = R0.T @ (c - zero["p"])
    drift = max(np.linalg.norm(Rs[t] @ x_link + ps[t] - c) for t in range(n)) * 1000

    # 2) 传动比：近节绕虚拟轴转的角度 vs 电机角度
    phi = np.array([np.dot(rotvec(R @ R0.T), av) for R in Rs])
    th0 = th - th[0]
    ratio = abs(np.polyfit(th0, phi, 1)[0])

    # 3) 相对电机轴的偏移
    v = c - o
    f = zero["pip"] - c; f -= av * np.dot(f, av); f /= np.linalg.norm(f)          # 指尖方向（垂直于轴）
    palm = np.cross(av, f)
    if np.dot(palm, zero["dors"]) > 0:
        palm = -palm                                                               # 掌侧 = 背离手背
    Rm = zero["Rm"]
    v_m, av_m = Rm.T @ v, Rm.T @ av
    ang = math.degrees(math.acos(min(1.0, abs(float(np.dot(av, am))))))
    perp = np.linalg.norm(v - am * np.dot(v, am)) * 1000

    name = lambda bid: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, bid)
    print(f"扫描 {args.range[0]:.0f}° … {args.range[1]:.0f}°，{n} 个姿态")
    print(f"近节（输出连杆）body: {name(link)}    电机曲柄 body: {name(motor_body)}")
    print()
    print(f"1) 漂移：虚拟转轴上的点在整个扫描中最多移动 {drift:.2f} mm" + ("（是一个真正的固定转轴）" if drift < 0.3 else "（不是严格的固定转轴，见下面说明）"))
    print(f"2) 传动比：近节转角 / 电机转角 = {ratio:.4f}" + ("（1:1，机械手关节角 = 手套电机角）" if abs(ratio - 1) < 0.01 else "  <- 不是 1:1，同构时要按这个比例换算"))
    print(f"   虚拟转轴和电机轴的夹角 {ang:.2f}°" + ("（平行）" if ang < 0.5 else ""))
    print()
    print(f"3) 虚拟转轴相对电机轴的位置（零位）：两轴相距 {perp:.2f} mm")
    print(f"   直观分解：往指尖方向 {np.dot(v, f) * 1000:+.2f} mm，往掌侧 {np.dot(v, palm) * 1000:+.2f} mm，沿轴向 {np.dot(v, av) * 1000:+.2f} mm")
    print(f"   在电机配合的连接器坐标系里（原点在电机轴上，Z 沿电机轴）：")
    print(f"     点  X {v_m[0] * 1000:+8.2f}  Y {v_m[1] * 1000:+8.2f}  Z {v_m[2] * 1000:+8.2f}  mm")
    print(f"     轴  ({av_m[0]:+.4f}, {av_m[1]:+.4f}, {av_m[2]:+.4f})")
    print(f"   在基座 part_1（URDF 根连杆）坐标系里：点 ({c[0] * 1000:+.2f}, {c[1] * 1000:+.2f}, {c[2] * 1000:+.2f}) mm，"
          f"轴 ({av[0]:+.4f}, {av[1]:+.4f}, {av[2]:+.4f})")
    print()
    print(f"4) 在近节 body {name(link)} 的坐标系里，转轴穿过 ({x_link[0] * 1000:+.2f}, {x_link[1] * 1000:+.2f}, {x_link[2] * 1000:+.2f}) mm")
    print()
    vmax = max(viol) * 1000
    print(f"5) 中指 MCP 闭链在平面内的残差最大 {vmax:.3f} mm" + ("（远小于漂移：漂移来自连杆几何本身）" if vmax < drift / 5 else
          "（和漂移同一个量级：残差大的那些角度里连杆可能已经到了奇异 / 限位附近，结果不可信，缩小 --range 再算）"))
    # 逐段的瞬时转动中心（相邻两个姿态之间），在电机连接器坐标系里
    order = np.argsort(ang_list)
    print("   瞬时转动中心随角度的移动（电机连接器坐标系，mm；相对上面拟合的固定轴）：")
    for a_i, b_i in zip(order[:-1], order[1:]):
        Ra, pa, Rb2, pb2 = Rs[a_i], ps[a_i], Rs[b_i], ps[b_i]
        xi = np.linalg.lstsq(Ra - Rb2, pb2 - pa, rcond=None)[0]
        ci = Ra @ xi + pa
        ci = ci + av * np.dot(c - ci, av)
        dv = Rm.T @ (ci - c)
        print(f"     {ang_list[a_i]:+5.0f}° → {ang_list[b_i]:+5.0f}°   偏 X {dv[0] * 1000:+6.2f}  Y {dv[1] * 1000:+6.2f}"
              f"     平面内残差 {viol[b_i] * 1000:.3f} mm")
    # 近节转角 φ 对电机角 θ 的关系
    pf = np.polyfit(np.degrees(th0), np.degrees(phi), 2)
    print(f"6) 近节转角 φ(°) ≈ {pf[0]:+.5f}·θ² {pf[1]:+.4f}·θ {pf[2]:+.3f}，θ = 手套电机角（°）")
    for a_i in order[::3]:
        print(f"     电机 {math.degrees(th0[a_i]):+6.1f}°  ->  近节 {math.degrees(phi[a_i]):+6.1f}°")
    print("   机械手的 MCP 电机装在固定轴上时，它的关节角应该给 φ(θ)，而不是 θ；固定轴本身的位置误差 = 上面的漂移")


if __name__ == "__main__":
    main()
