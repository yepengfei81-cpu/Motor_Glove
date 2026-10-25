#!/usr/bin/env python3
"""
双手套主从遥操作 + 力反馈：手套 A（戴在手上，主）-> 手套 B（放在桌上，从）。

  python tools/bilateral.py --dry-run                 # 只读主手，算出来但不写任何电机
  python tools/bilateral.py                           # 正式：主手电流模式，从手跟随；从手被挡住时主手感到力
  python tools/bilateral.py index mid ring            # 只用几根手指
  python tools/bilateral.py --leader-port /dev/ttyACM0 --follower-port /dev/ttyUSB0
  python tools/bilateral.py --log data/grasp_soft.csv  # 记录每个周期：主/从位置、速度、电流 -> 之后用 plot_log.py 画图
  python tools/bilateral.py --follower-kp 150 --log data/kp150.csv   # 从手设成"软"的（P 增益 150，默认 400）
  python tools/bilateral.py --adaptive-kp 600 100 --log data/adaptive.csv
        # 方法 3 演示：从手刚度随接触电流变化 K_t = f(τ_t)。空中 P=600（硬、跟得准），接触电流滑动平均
        # 达到 --adapt-ref（默认 150 mA）时降到 P=100（软）。每周期的 P 增益记在 CSV 的 kpB 列，plot_log.py 会画出来

前提：两只手套各在一条总线上（例如 A 接 OpenRB，B 接 U2D2 + 另一块 OpenRB 供电），
     ID 都是 1–13（同一张表），B 用  calibrate.py <手指> --glove B --port <B 的串口>  标定过。

控制：
  从手：模式 5（电流限制的位置控制），目标位置 = 主手关节角换算到从手的编码器；电流上限 = 抓握力上限。
  主手：电流模式。反馈力矩 τ = -kp·(|e|-死区)·sign(e)，e = q_主 - q_从：从手被挡住，e 变大，主手被往回推。
       另外可以叠加从手电流：τ += -kc·(I_从 - I_th)（DITTO 的做法），默认 kc = 0。
  自由运动时从手跟得上，e 在死区内，主手不出力。
安全：电流上限写进电机；启动 1.5 s 渐进；总线看门狗 100 ms；退出或出错时两边都清零电流、关扭矩。
"""
import argparse
import math
import os
import signal
import sys
import threading
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import glove_model as gm  # noqa: E402
from dxl_bus import (Bus, HW_ERRORS, KT, MODEL_NAME, TICKS_PER_REV, effective_sign, load_calibration, load_registry,  # noqa: E402
                     ticks_to_rad, wrap_ticks, default_port)


def go_realtime(cpu):
    """SCHED_FIFO 50 + 绑核。cpu < 0 时挑最近一秒最空闲的核（避开 ROS2 正在用的）。后面创建的线程会继承"""
    def busy():
        out = {}
        for line in open("/proc/stat"):
            if line.startswith("cpu") and line[3].isdigit():
                v = list(map(int, line.split()[1:8]))
                out[int(line[3:].split()[0])] = (sum(v) - v[3], sum(v))
        return out
    if cpu is None or cpu < 0:
        a = busy(); time.sleep(0.3); b = busy()
        load = {c: (b[c][0] - a[c][0]) / max(1, b[c][1] - a[c][1]) for c in a if c >= 2}
        cpu = min(load, key=load.get) if load else 0
    try:
        os.sched_setaffinity(0, {cpu})
    except Exception as e:  # noqa: BLE001
        print(f"!! 绑核失败: {e}")
    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(50))
        print(f"* 实时优先级 SCHED_FIFO 50，绑在 CPU {cpu}")
    except PermissionError:
        print(f"!! 没有实时优先级权限（ulimit -r 是 0）。一次性放开：sudo 编辑 /etc/security/limits.conf 加一行\n"
              f"     {os.environ.get('USER', 'ypf')} - rtprio 99\n   重新登录后再跑。临时办法：另开终端 sudo chrt -f -p 50 {os.getpid()}。已绑在 CPU {cpu}，继续普通优先级跑")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("fingers", nargs="*", help="thumb / index / mid / ring，默认全部")
    ap.add_argument("--leader-port", default=None, help="主手串口（默认 /dev/ttyACM0）")
    ap.add_argument("--follower-port", default=None, help="从手串口（默认 /dev/ttyUSB0；写 none 表示没有从手，只测主手）")
    ap.add_argument("--leader-glove", default="A")
    ap.add_argument("--follower-glove", default="B")
    ap.add_argument("--dry-run", action="store_true", help="不开扭矩、不写任何电机")
    ap.add_argument("--kp", type=float, default=0.5, help="位置误差反馈刚度 N·m/rad（默认 0.5）")
    ap.add_argument("--kc", type=float, default=0.0, help="从手电流反馈增益（主手力矩 / 从手力矩，默认 0 = 关）")
    ap.add_argument("--deadband", type=float, default=2.0, help="位置误差死区 度（默认 2）")
    ap.add_argument("--skip", nargs="*", default=[], metavar="JOINT", help="临时不同步的关节名，例如 --skip thumb_cmc_abd_motor")
    ap.add_argument("--map", default=None, help="fit_map.py 拟合的主->从关节映射 json：从手目标 = a·q主 + b，补偿指尖几何差异")
    ap.add_argument("--follower-model", choices=["glove", "hand"], default="glove", help="从手是手套还是直驱机械手（默认 glove）")
    ap.add_argument("--g2h", default=None, help="手套->机械手关节映射 json（默认 config/glove_to_hand.json，从手是 hand 时自动用）")
    ap.add_argument("--scale", action="append", default=[], metavar="NAME=K",
                    help="主手某个关节角先乘 K 再映射（经验修正，如 --scale thumb_mcp_flex_motor=1.72）")
    ap.add_argument("--follower-range", type=float, default=110, help="从手目标角度限幅 ±度（默认 110，保护机械手不撞限位）")
    ap.add_argument("--contact-ma", type=float, default=100, help="接触门控：从手电流到这个值（mA）时门控全开，一半时开始开；0 = 关掉门控（默认 100）")
    ap.add_argument("--contact-vel", type=float, default=0.8, help="接触门控：从手速度超过这个值（rad/s）视为自由运动不出力，低于 1/4 时视为被挡住（默认 0.8）")
    ap.add_argument("--contact", action="append", default=[], metavar="NAME=MA",
                    help="某个关节单独的接触电流门限 mA（机构有摩擦、空载电流就高的关节调大，如 --contact index_pip_motor=250）")
    ap.add_argument("--contact-lag", type=float, default=0.0,
                    help="接触门控再加一条：从手落后目标超过这个角度（度）才算被挡住，0 = 关（默认）。摩擦大的关节只会让电流高、不会落后很多")
    ap.add_argument("--no-fb", nargs="*", default=[], metavar="JOINT", help="这些关节只跟随、不给力反馈，如 --no-fb index_pip_motor")
    ap.add_argument("--fb-scale", action="append", default=[], metavar="NAME=K",
                    help="某个关节的反馈力乘 K（发抖的关节调小，如 --fb-scale thumb_cmc_abd_motor=0.5）")
    ap.add_argument("--two-sided", action="store_true",
                    help="允许从手在主手前面时把主手往前拉（默认不允许：只在主手往从手被挡住的方向推时才出力，"
                         "避免从手关节卡住落在后面时把你的手指往前带）")
    ap.add_argument("--model", default=None, metavar="NAME",
                    help="用 motor_id.py 辨识出来的参数：NAME = config/motor_params/NAME.json 用于所有关节（台架 T1/T3）；"
                         "auto = 按每个电机总线报的型号选 XL330-M288.json / XL330-M077.json（手套、机械手用这个）。"
                         "打开后：主手用观测速度加阻尼（--zeta）和摩擦前馈（--comp-gain），从手用模型法接触检测（--gate model），墙模式电流上限按推人/顶人分开")
    ap.add_argument("--gate", choices=["current", "model"], default=None,
                    help="接触检测：current = 从手电流过门限（旧）；model = 从手实测电流减去模型算的空载电流（需要 --model）。默认跟 --model 走")
    ap.add_argument("--r-load", type=float, default=0.025, help="负载摩擦换算用的力臂 m：手推杆的位置 / 四连杆在舵盘上的半径（默认 0.025）")
    ap.add_argument("--r-load-map", action="append", default=[], metavar="NAME=M", help="按关节给力臂 m（舵盘半径），覆盖 --r-load；可重复")
    ap.add_argument("--comp-gain", type=float, default=0.5, help="空载库仑摩擦前馈比例（默认 0.5）")
    ap.add_argument("--v-dead", type=float, default=0.3, help="摩擦前馈的速度死区 rad/s（默认 0.3；慢动作也想补就 0.15，太小零速附近会抖）")
    ap.add_argument("--v-ramp", type=float, default=0.5, help="摩擦前馈从死区到全额的过渡宽度 rad/s（默认 0.5）")
    ap.add_argument("--b-free", type=float, default=0.0,
                    help="空载粘滞阻力 N·m·s/rad（默认 0），封顶在补掉的库仑摩擦，最坏等于关扭矩的手感。杆松手滑太久就给 0.001")
    ap.add_argument("--zeta", type=float, default=0.7, help="主手接触时的阻尼比，B = 2ζ√(kp·J)（默认 0.7）")
    ap.add_argument("--v0", type=float, default=0.5, help="摩擦前馈和推人/顶人判断的速度过渡宽度 rad/s（默认 0.5）")
    ap.add_argument("--kc-lpf", type=float, default=3.0, help="--coupling 时观测接触力反射前的低通 Hz（默认 3）")
    ap.add_argument("--coupling", action="store_true",
                    help="虚拟耦合（需要 --model）：主手力矩 = −kp·死区(e) − B·ė + kc·F̂，F̂ = 从手实测电流 − 模型空载电流。"
                         "没有门控：空载时三项自然为零，接触时阻尼和力反射连续生效；B = 2ζ√(kp·J)")
    ap.add_argument("--mode", choices=["current", "pwm"], default="current",
                    help="主手驱动方式：current 电机电流模式（默认）；pwm 电压模式——力矩自己换算成占空比 (τ/G + k·Ke·v̂)/V，"
                         "电机的电磁阻尼 Kt·Ke/R 是物理的、零延迟的：空载按 --pwm-cancel 抵消掉（和电流模式一样轻），接触和松手时保留")
    ap.add_argument("--pwm-limit", type=float, default=90, help="pwm 模式的 PWM 上限 %%（默认 90）")
    ap.add_argument("--pwm-g", type=float, default=None, help="pwm 模式：堵转力矩/电压 N·m/V = Kt/R（默认取 motor_params 的 pwm.G，没有则 0.044）")
    ap.add_argument("--pwm-ke", type=float, default=None, help="pwm 模式：反电动势常数 V·s/rad（默认取 motor_params 的 pwm.Ke，没有则 0.12）")
    ap.add_argument("--pwm-cancel", type=float, default=0.8, help="pwm 模式：按 Ke·v̂ 前馈抵消电磁阻尼的比例 0–1（默认 0.8；剩下交给电流环。估高了就是负阻尼、停不下来，M288 的电磁阻尼 21 mN·m·s/rad 不抵消就是空载最大的黏劲）")
    ap.add_argument("--pwm-iloop", type=float, default=5.0, help="pwm 模式：软件电流环带宽 Hz（默认 5）。用电机实测电流做积分修正，低频像电流模式一样轻，高频（接触、松手）保留电磁阻尼；0 关掉")
    ap.add_argument("--pwm-r", type=float, default=None, help="pwm 模式：电压/实测电流 的表观电阻 Ω（默认取 motor_params 的 pwm.R_sensor_ohm，没有则 4.1），定电流环增益和限流")
    ap.add_argument("--wall", action="store_true",
                    help="虚拟墙模式（力位混控）：主手用模式 5（电流限制的位置控制），目标位置 = 从手当前位置、电流上限 = 门控 × max-torque。"
                         "墙由电机内部 2 kHz 位置环渲染，没有虚位；空中电流上限 0，手感等于关扭矩。kp/deadband 在这个模式里不起作用")
    ap.add_argument("--wall-kp", type=int, default=800, help="--wall 时主手位置环 P 增益（默认 800）")
    ap.add_argument("--wall-kd", type=int, default=0, help="--wall 时主手位置环 D 增益（默认 0；松手回弹振荡就加，试 20–60）")
    ap.add_argument("--wall-press", type=float, default=5.0, help="--wall 开着时从手目标停在墙里几度（默认 5：保持稳定的接触电流，又不把物体压扁）")
    ap.add_argument("--gate-tau-off", type=float, default=None,
                    help="接触门控【关闭】的时间常数 s（默认 = gate-tau 的 1/3：开慢一点不误触发，关快一点松手就松）")
    ap.add_argument("--max-torque", type=float, default=0.08, help="主手每个电机输出力矩上限 N·m（默认 0.08 ≈ 230 mA）")
    ap.add_argument("--friction-ma", type=float, default=0.0, help="主手摩擦补偿：顺着运动方向给这么多 mA 抵消减速箱反拖阻力（默认 0 = 关；试 20–30）")
    ap.add_argument("--visc-ma", type=float, default=0.0, help="主手粘滞补偿：每 rad/s 顺着运动方向给这么多 mA，抵消电流模式下的电磁阻尼（默认 0；试 10–20）")
    ap.add_argument("--damp", type=float, default=0.0, help="接触时主手的阻尼 N·m·s/rad，逆着运动方向，压抖动（默认 0；试 0.01–0.03）")
    ap.add_argument("--gate-tau", type=float, default=0.05, help="接触门控的平滑时间常数 s（默认 0.05；抖就调大到 0.15）")
    ap.add_argument("--leader-limit", type=int, default=500, help="主手电流上限 mA")
    ap.add_argument("--follower-limit", type=int, default=300, help="从手电流上限 mA（= 抓握力上限）")
    ap.add_argument("--budget", type=float, default=2500, help="每条总线电流总和上限 mA")
    ap.add_argument("--lpf", type=float, default=25, help="主手反馈力的低通 Hz（默认 25）")
    ap.add_argument("--hz", type=float, default=500, help="控制环目标频率（默认 500；3 Mbps + Fast Sync Read 下 13 关节可到 ~800）")
    ap.add_argument("--no-parallel", action="store_true", help="不并行读两条总线（默认并行：主手和从手的状态同时读，控制频率高一倍左右）")
    ap.add_argument("--rt", type=int, nargs="?", const=-1, default=None, metavar="CPU",
                    help="实时优先级（SCHED_FIFO 50）并绑到一个 CPU 核（默认自动挑一个空闲的）。不用关 ROS2，实时优先级会抢在它们前面。"
                         "需要一次性放开 rtprio 限制：/etc/security/limits.conf 加一行 `ypf - rtprio 99`，重新登录")
    ap.add_argument("--lock", action="store_true",
                    help="测试力反馈本身：不用从手（--follower-port none），按回车把当前姿态当成从手被挡住的位置（虚拟墙），"
                         "再按回车解锁。戴着用力推，看每个关节能被推开多少、出多少电流")
    ap.add_argument("--log", default=None, help="把每个周期的数据存成 CSV")
    ap.add_argument("--follower-kp", type=int, default=None, help="从手位置环 P 增益（电机默认 400；低 = 软，高 = 硬），会记进 CSV")
    ap.add_argument("--follower-kd", type=int, default=None, help="从手位置环 D 增益（默认 0）")
    ap.add_argument("--follower-ki", type=int, default=None, help="从手位置环 I 增益（默认 0）。能顶过静摩擦消掉小误差，太大会在目标附近来回蹭")
    ap.add_argument("--follower-mode", choices=["position", "current"], default="position",
                    help="从手怎么驱动：position = 电机自己的位置环（默认）；current = 电流模式，我们自己做 PD + 模型前馈（惯量、粘滞、库仑摩擦都按辨识参数沿参考轨迹补），"
                         "从手不再粘滑，接触力也能直接算出来（指令力矩 − 模型），需要 --model")
    ap.add_argument("--follower-bw", type=float, default=6.0, help="--follower-mode current 时从手位置环带宽 Hz（默认 6）：K = J·(2πf)²，按每个电机的惯量自动算")
    ap.add_argument("--follower-bw-map", action="append", default=[], metavar="KEY=HZ",
                    help="按关节或型号给从手环带宽，覆盖 --follower-bw；KEY 是关节名前缀（index_pip）或型号（M077、M288）。例：--follower-bw-map M077=12")
    ap.add_argument("--follower-zeta", type=float, default=0.8, help="--follower-mode current 时从手位置环阻尼比（默认 0.8）")
    ap.add_argument("--follower-comp", type=float, default=0.9, help="--follower-mode current 时从手库仑摩擦前馈比例（默认 0.9），沿参考轨迹的速度方向")
    ap.add_argument("--fb-force", type=float, default=0.0,
                    help="不经门控的力反射增益（默认 0 = 关）：主手力矩 −= 增益 × 从手接触力估计。从手是 current 模式时用 指令力矩 − 模型 算，position 模式时用 实测电流 − 模型。"
                         "软物体、纹理靠它才摸得到；位置差弹簧只能摸到硬挡")
    ap.add_argument("--fb-force-lpf", type=float, default=6.0,
                    help="力反射观测器的统一带宽 Hz（默认 6）。指令力矩、速度项、加速度项全部过同一个低通再相减，否则主手一加速，瞬时的指令力矩减去滞后的模型就是一个假的接触力，反射回去等于负惯量，两边一起抖")
    ap.add_argument("--fb-force-th", type=float, default=5.0, help="力反射的死区 mN·m（默认 5），模型误差以内不反射")
    ap.add_argument("--obs-hz", type=float, default=15.0, help="主手速度观测器低通 Hz（默认 15）。M288 惯量大、电磁阻尼大，可以开到 25，换向时前馈翻得更快")
    ap.add_argument("--gain", action="append", default=[], metavar="NAME=KP[,KD[,KI]]",
                    help="单独给某些关节设从手 P/D 增益，NAME 是关节名或前缀，例如 --gain thumb=600,60 --gain index_pip_motor=900")
    ap.add_argument("--cmd-lpf", type=float, default=0.0,
                    help="从手目标角度的低通 Hz：滤掉主手编码器量化和手抖，减小从手抖动（0 = 关；试 10–15）")
    ap.add_argument("--adaptive-kp", type=int, nargs=2, metavar=("KP_MAX", "KP_MIN"), default=None,
                    help="方法 3：从手 P 增益随接触电流变化。空中 = KP_MAX，接触电流(滑动平均)达到 --adapt-ref 时降到 KP_MIN，例如 600 100")
    ap.add_argument("--adapt-ref", type=float, default=150, help="接触电流参考值 mA（滑动平均到这个值时刚度降到 KP_MIN，默认 150）")
    ap.add_argument("--adapt-tau", type=float, default=0.3, help="接触电流滑动平均的时间常数 s（默认 0.3；ACP 用 1 s）")
    args = ap.parse_args()
    fingers = args.fingers or list(gm.FINGERS)
    bad = [f for f in fingers if f not in gm.FINGERS]
    if bad:
        sys.exit(f"不认识的手指 {bad}")
    if args.rt is not None:
        go_realtime(args.rt)
    lport = args.leader_port or "/dev/ttyACM0"
    fport = None if (args.follower_port or "").lower() == "none" or args.lock else (args.follower_port or "/dev/ttyUSB0")

    reg = load_registry()
    calA, calB = load_calibration(args.leader_glove), load_calibration(args.follower_glove)
    joints = [j for f in fingers for j in gm.FINGERS[f] if j in calA["joints"] and (fport is None or j in calB["joints"]) and j not in args.skip]
    if args.skip:
        print(f"* 按要求跳过: {args.skip}")
    if not joints:
        sys.exit("没有两只手套都标定过的关节；从手用  python tools/calibrate.py <手指> --glove B --port <串口>  标定")
    skipped = [j for f in fingers for j in gm.FINGERS[f] if j not in joints]
    if skipped:
        print(f"* 跳过（从手没标定）: {skipped}")
    axes = gm.joint_axes()                                       # 主手永远是手套
    axes_f = gm.joint_axes(gm.scene_of(args.follower_model))     # 从手按它自己的模型取关节轴
    ids = {j: reg[j] for j in joints}
    sA = {j: effective_sign(calA["joints"][j], j, axes) for j in joints}
    zA = {j: calA["joints"][j]["zero_raw"] for j in joints}      # 开主手总线后会换算到"当前所在的那一圈"（见 zA_turn）
    db = math.radians(args.deadband)
    kp0 = args.adaptive_kp[0] if args.adaptive_kp else (args.follower_kp if args.follower_kp is not None else 400)
    kp_cmd = {j: float(kp0) for j in joints}       # 每个关节当前的从手 P 增益（固定或随电流变化）
    kp_written = dict(kp_cmd)
    iB_avg = {j: 0.0 for j in joints}

    leader = Bus(ids.values(), port=lport)
    leader.torque_off()
    leader.ensure_return_delay("主手")
    cleared, still = leader.clear_hardware_errors()
    if still:
        leader.close()
        sys.exit(f"主手有硬件错误没清掉: {still}")
    models = {j: leader.model(ids[j]) for j in joints}
    # 主手角度不再按 ±180° 折返：零点换算到上电时所在的那一圈，之后用多圈位置连续计数。
    # 手套关节到不了 ±180°，无影响；台架的杆转过 180° 时从手目标不会突然跳 360°
    rawA0 = leader.read_raw()
    for j in joints:
        zA[j] = zA[j] + round((rawA0[ids[j]] - zA[j]) / TICKS_PER_REV) * TICKS_PER_REV
    follower = None
    if fport is not None:
        follower = Bus(ids.values(), port=fport)
        follower.torque_off()
        follower.ensure_return_delay("从手")
        cleared, still = follower.clear_hardware_errors()
        if still:
            leader.close(); follower.close()
            sys.exit(f"从手有硬件错误没清掉: {still}")
        sB = {j: effective_sign(calB["joints"][j], j, axes_f) for j in joints}
        # 从手零点换算到"当前所在的那一圈"，避免上电位置跨过 0/4095 时转一整圈
        raw0 = follower.read_raw()
        zB = {}
        for j in joints:
            z = calB["joints"][j]["zero_raw"]
            zB[j] = z + round((raw0[ids[j]] - z) / TICKS_PER_REV) * TICKS_PER_REV
        fmodels = {j: follower.model(ids[j]) for j in joints}

    from retarget import JointMap
    jmap = JointMap(joints, scale=args.scale, map_path=args.map, to_hand=(args.follower_model == "hand"),
                    g2h_path=args.g2h, range_deg=args.follower_range)
    to_follower = jmap
    def kt_lead_guess(j):
        return (MPJ[j]["kt"] if MPJ else KT[models[j]])

    def load_mp(path):
        import json as _json
        raw = _json.load(open(path))
        kt_em = raw.get("kt_em_estimate") or raw.get("kt_drive_net") or 0.146
        J_true = raw.get("J_chirp", 2.6e-6) * kt_em / raw.get("kt_used", 0.146)   # chirp 的 J 是按 fit 当时用的 Kt 拟的，换算到这里用的 Kt
        return {"kt": kt_em, "J": J_true, "b": raw.get("b_over_J", 1.3) * J_true, "tc": raw.get("tc_over_J", 110) * J_true,
                "k_load": raw.get("load_friction_model", {}).get("friction_mNm_per_N_radial", 11.0) / 1000,
                "pwm": raw.get("pwm", {}), "path": os.path.relpath(path)}
    rmap = {}
    for item in args.r_load_map:
        n_, v_ = item.split("="); rmap[n_] = float(v_)
    r_load = {j: rmap.get(j, args.r_load) for j in joints}
    MP, MPJ, MPF = None, None, None                        # MP 只当"有没有模型"的开关用；参数按关节放在 MPJ[j]（主手）/ MPF[j]（从手）
    fbw = {j: args.follower_bw for j in joints}            # 从手软件环带宽，按关节/型号可覆盖
    for item in args.follower_bw_map:
        key, v_ = item.split("="); v_ = float(v_)
        for j in joints:
            fm_name = MODEL_NAME.get(fmodels[j], "") if follower is not None else ""
            if j.startswith(key) or (key and key in fm_name):
                fbw[j] = v_
    B_contact = {j: 0.0 for j in joints}
    phi_eff = {j: 0.0 for j in joints}
    if args.model:
        mp_dir = os.path.join(gm.ROOT, "config", "motor_params")
        cache, MPJ = {}, {}
        for j in joints:
            # --model auto：按总线报的型号选 config/motor_params/<型号>.json（XL330-M288 / XL330-M077）；否则所有关节用同一份
            name = MODEL_NAME.get(models[j], str(models[j])) if args.model == "auto" else args.model
            path = os.path.join(mp_dir, f"{name}.json")
            if not os.path.exists(path):
                sys.exit(f"没有 {os.path.relpath(path)}（--model auto 需要 config/motor_params/<型号>.json，比如 XL330-M288.json）")
            MPJ[j] = cache.setdefault(path, load_mp(path))
        MP = True
        MPF = dict(MPJ)                                        # 从手接触检测用的模型：默认和主手同一份
        if args.model == "auto" and follower is not None:
            for j in joints:
                name = MODEL_NAME.get(fmodels[j], str(fmodels[j]))
                path = os.path.join(mp_dir, f"{name}.json")
                if os.path.exists(path):
                    MPF[j] = cache.setdefault(path, load_mp(path))
        B_contact = {j: 2 * args.zeta * math.sqrt(max(args.kp, 1e-3) * MPJ[j]["J"]) for j in joints}
        phi_eff = {j: min(0.5, MPJ[j]["k_load"] / r_load[j]) for j in joints}   # 负载摩擦 / 传递力矩，封顶 0.5
        for path, m in cache.items():
            js = [j for j in joints if MPJ[j] is m]
            print(f"* 电机模型 {m['path']}（{len(js)} 个主手关节{'，从手同款' if all(MPF[j] is m for j in js) else '，从手另选'}）：Kt_em {m['kt']:.3f} N·m/A，J {m['J'] * 1e6:.0f} ×10⁻⁶ kg·m²，b {m['b'] * 1e3:.3f} mN·m·s/rad，"
                  f"tc {m['tc'] * 1e3:.1f} mN·m，负载摩擦 {m['k_load'] * 1e3:.1f} mN·m/N；"
                  f"接触阻尼 B = 2·{args.zeta}·√(kp·J) = {B_contact[js[0]] * 1e3:.1f} mN·m·s/rad；空载摩擦前馈 {m['tc'] * 1e3:.1f} mN·m × {args.comp_gain:.0%}")
        rl = sorted(set(round(v, 4) for v in r_load.values()))
        print(f"* 力臂 r_load {', '.join(f'{v * 1000:.0f}' for v in rl)} mm → 推人效率补偿 ×(1 + φ·drive)，φ = {', '.join(f'{v:.2f}' for v in sorted(set(round(x, 2) for x in phi_eff.values())))}；"
              f"接触检测 {'模型法' if (args.gate or 'model') == 'model' else '电流门限'}；|v| < {args.v_dead} rad/s 不补摩擦")
    if args.mode == "pwm":
        if args.wall:
            sys.exit("--mode pwm 和 --wall 不能同时用")
        pwm_g = {j: args.pwm_g or (MPJ and MPJ[j]["pwm"].get("G_Nm_per_V")) or 0.044 for j in joints}
        pwm_ke = {j: args.pwm_ke or (MPJ and MPJ[j]["pwm"].get("Ke_V_s_per_rad")) or 0.12 for j in joints}
        pwm_r = {j: args.pwm_r or (MPJ and MPJ[j]["pwm"].get("R_sensor_ohm")) or 4.1 for j in joints}
        pwm_ui = {j: 0.0 for j in joints}                                              # 电流环积分项（V），只允许表示反电动势
        # 堵转 占空比→电流 查表（断续导通：低占空比下表观电阻 14 Ω，到 88% 才是 4.1 Ω）。按 6.4 V 测的，别的电压按比例缩放占空比
        DEFAULT_MAP = {"duty_pct": [0, 3, 5, 7, 9, 11.5, 14.5, 18, 22.5, 27.5, 35, 45, 57, 88], "mA": [0, 18, 25, 32, 43, 56, 74, 104, 126, 199, 263, 396, 552, 1475], "V_bus": 6.4}
        pwm_map = {}
        for j in joints:
            mp_ = (MPJ[j]["pwm"].get("stall_map") if MPJ else None) or DEFAULT_MAP
            pwm_map[j] = (np.array(mp_["mA"], float) / 1000, np.array(mp_["duty_pct"], float) / 100, float(mp_.get("V_bus", 6.4)))

        def duty_for_current(j, i_a, v_bus):
            """堵转时要这么多电流（A，传感器单位）需要的占空比（0–1），超出表的部分按 R 线性外推"""
            ia, dd, v0 = pwm_map[j]
            a = abs(i_a)
            if a <= ia[-1]:
                d_ = float(np.interp(a, ia, dd))
            else:
                d_ = float(dd[-1] + (a - ia[-1]) * pwm_r[j] / v0)
            return math.copysign(d_ * v0 / v_bus, i_a)

        def r_eff(j, i_a, v_bus):
            """工作点附近的表观电阻 V/A（电流环增益用）"""
            a = max(abs(i_a), 0.01)
            return abs(duty_for_current(j, a, v_bus)) * v_bus / a
        seen = set()
        for j in joints:
            key = (pwm_g[j], pwm_ke[j], pwm_r[j])
            if key in seen:
                continue
            seen.add(key)
            print(f"* 电压(PWM)模式{'（' + MPJ[j]['path'] + '）' if MPJ else ''}：Kt {kt_lead_guess(j):.3f} N·m/A，Ke {pwm_ke[j]:.3f} V·s/rad，堵转 R {pwm_r[j]:.1f} Ω；"
                  f"占空比按堵转查表：10 mA → {100 * duty_for_current(j, 0.01, 6.4):.1f}%，100 mA → {100 * duty_for_current(j, 0.1, 6.4):.0f}%，800 mA → {100 * duty_for_current(j, 0.8, 6.4):.0f}%")
        print(f"* 空载反电动势前馈抵消 {args.pwm_cancel:.0%} + 电流环 {args.pwm_iloop:g} Hz（增益按工作点表观电阻），接触门控开了两者都停；PWM 上限 {args.pwm_limit:.0f}%，限流 {args.leader_limit:.0f} mA")
    if args.gate is None:
        args.gate = "model" if MP else "current"
    if args.gate == "model" and MP is None:
        sys.exit("--gate model 需要 --model")
    kt_lead = {j: (MPJ[j]["kt"] if MPJ else KT[models[j]]) for j in joints}
    obs = {j: {"qL": None, "vL": 0.0, "qB": None, "vB": 0.0, "vB_raw_prev": 0.0, "aB": 0.0, "iBf": 0.0, "e": None, "ed": 0.0} for j in joints}
    if args.coupling and MP is None:
        sys.exit("--coupling 需要 --model")
    i_free_log = {j: 0.0 for j in joints}
    wall_free_dir = {j: 0.0 for j in joints}
    wall_on = {j: False for j in joints}
    wall_hold = {j: (0.0, 0.0) for j in joints}
    wall_qB = {j: 0.0 for j in joints}
    wall_dir = {j: 1.0 for j in joints}
    dirn = {j: jmap.direction(j) for j in joints}          # 从手空间的位置差/力矩换回主手方向
    cma = {j: args.contact_ma for j in joints}              # 每个关节的接触电流门限
    for item in args.contact:
        name, _, v = item.partition("=")
        if name in cma:
            cma[name] = float(v)
        else:
            print(f"!! --contact {item}: 没有这个关节（或没在同步）")
    if args.contact:
        print("* 单独的接触门限：" + "，".join(f"{j.replace('_motor', '')} {cma[j]:.0f} mA" for j in joints if cma[j] != args.contact_ma))
    no_fb = [j for j in joints if j in args.no_fb]
    if no_fb:
        print(f"* 只跟随、不给力反馈: {', '.join(j.replace('_motor', '') for j in no_fb)}")
    lag_rad = math.radians(args.contact_lag)
    fb_scale = {j: 1.0 for j in joints}
    for item in args.fb_scale:
        name, _, v = item.partition("=")
        if name in fb_scale:
            fb_scale[name] = float(v)
        else:
            print(f"!! --fb-scale {item}: 没有这个关节（或没在同步）")
    if args.fb_scale:
        print("* 反馈力比例：" + "，".join(f"{j.replace('_motor', '')} ×{fb_scale[j]:.2f}" for j in joints if fb_scale[j] != 1.0))
    move_dir = {j: 0.0 for j in joints}                    # 主手目标最近在往哪个方向走（+1/-1），单向反馈用
    qA_prev = None
    flipped = [j for j in joints if dirn[j] < 0]
    if flipped:
        print(f"* 这些关节从手方向和主手相反，力反馈已按方向换算: {', '.join(j.replace('_motor', '') for j in flipped)}")

    def leader_angles(st):
        """主手关节角换算成从手的目标（力反馈的位置差也用换算后的值）"""
        return to_follower({j: sA[j] * ticks_to_rad(st[ids[j]][2] - zA[j]) for j in joints})

    def follower_goal_ticks(q):
        return {ids[j]: zB[j] + sB[j] * q[j] * TICKS_PER_REV / (2 * math.pi) for j in joints}

    print(f"* 主手 {lport}（{args.leader_glove}），从手 {fport or '无'}（{args.follower_glove}）；{len(joints)} 个关节；pid {os.getpid()}"
          + ("" if args.rt is not None else "（要实时优先级：加 --rt，或另开终端 sudo chrt -f -p 50 " + str(os.getpid()) + "）"))
    print(f"* kp {args.kp} N·m/rad，kc {args.kc}，死区 {args.deadband}°，接触门控 {args.contact_ma:.0f} mA / {args.contact_vel} rad/s，"
          f"主手力矩上限 {args.max_torque} N·m，摩擦补偿 {args.friction_ma:.0f} mA，从手电流上限 {args.follower_limit} mA")
    if args.adaptive_kp:
        print(f"* 从手刚度随接触电流变化：P {args.adaptive_kp[0]} (空中) → {args.adaptive_kp[1]} (电流均值 ≥ {args.adapt_ref:.0f} mA)，时间常数 {args.adapt_tau} s")
    elif args.follower_mode == "current" and MPF is not None:
        for j in joints:
            m_ = MPF[j]; K_s = m_["J"] * (2 * math.pi * fbw[j]) ** 2
            print(f"* 从手 {j.replace('_motor', '')}：电流模式软件位置环，带宽 {fbw[j]:g} Hz → K {K_s:.2f} N·m/rad，B {2 * args.follower_zeta * math.sqrt(K_s * m_['J']) * 1e3:.1f} mN·m·s/rad，"
                  f"前馈 J {m_['J'] * 1e6:.0f}e-6·a + b·v + {args.follower_comp:.0%}·{m_['tc'] * 1e3:.1f} mN·m·sgn(v)，上限 {args.follower_limit} mA")
        if args.fb_force > 0:
            print(f"* 力反射（不经门控）：主手 −= {args.fb_force} × (从手指令力矩 − 模型)，低通 {args.fb_force_lpf:g} Hz，死区 {args.fb_force_th:g} mN·m")
    else:
        print(f"* 从手 P 增益固定 {kp0}" + (f"；力反射 {args.fb_force} ×（实测电流 − 模型），低通 {args.fb_force_lpf:g} Hz" if args.fb_force > 0 else ""))
    try:
        if not args.dry_run:
            if follower is not None and args.follower_mode == "current":
                if MPF is None:
                    sys.exit("--follower-mode current 需要 --model（从手的惯量和摩擦参数）")
                if args.adaptive_kp or args.wall:
                    sys.exit("--follower-mode current 不能和 --adaptive-kp / --wall 一起用")
                follower.enter_current_mode(args.follower_limit)
            elif follower is not None:
                follower.enter_position_mode(args.follower_limit)
                follower.set_position_gains(kp0, args.follower_kd, args.follower_ki)
                for g in args.gain:
                    name, _, val = g.partition("=")
                    parts = (val.split(",") + ["", "", ""])[:3]
                    kp_s, kd_s, ki_s = parts
                    hit = [j for j in joints if j.startswith(name)]
                    if not hit:
                        print(f"!! --gain {g}: 没有关节名以 {name} 开头")
                    for j in hit:
                        if kp_s:
                            follower.pkt.write2ByteTxRx(follower.port, ids[j], 84, int(kp_s))
                            kp_cmd[j] = kp_written[j] = float(kp_s)
                        if kd_s:
                            follower.pkt.write2ByteTxRx(follower.port, ids[j], 80, int(kd_s))
                        if ki_s:
                            follower.pkt.write2ByteTxRx(follower.port, ids[j], 82, int(ki_s))   # 位置环 I：顶掉摩擦造成的稳态误差
                    if hit:
                        print(f"* 从手增益 {', '.join(hit)}: P {kp_s or '不变'}，D {kd_s or '不变'}，I {ki_s or '不变'}")
                # 从手用 1.5 s 从自己现在的姿态滑到主手当前姿态：两只手套开始时不需要摆成一样，也不需要在零位
                qA0 = leader_angles(leader.read_state())
                raw = follower.read_raw()
                qB0 = {j: sB[j] * ticks_to_rad(raw[ids[j]] - zB[j]) for j in joints}
                print("* 从手正在滑到主手的姿态 ...", flush=True)
                t_s = time.time()
                while True:
                    s_ = min(1.0, (time.time() - t_s) / 1.5)
                    follower.write_positions(follower_goal_ticks({j: qB0[j] + (qA0[j] - qB0[j]) * s_ for j in joints}))
                    if s_ >= 1.0:
                        break
                    time.sleep(0.02)
            if args.wall:
                leader.enter_position_mode(args.leader_limit)       # 模式 5：目标电流寄存器 = 力的上限，先清零（空中无力）
                leader.set_position_gains(args.wall_kp, args.wall_kd)
                leader.write_currents({ids[j]: 0 for j in joints})
                print(f"* 虚拟墙模式：主手模式 5，P {args.wall_kp} D {args.wall_kd}，电流上限随门控 0 → 反馈力")
            elif args.mode == "pwm":
                leader.enter_pwm_mode(args.pwm_limit)
            else:
                leader.enter_current_mode(args.leader_limit)
            leader.check_torque("主手")
            if follower is not None:
                follower.check_torque("从手")
            print("* 扭矩已开。Ctrl+C 退出，退出时两边都关扭矩")
        alerts = []                                    # 最近的保护报警，固定显示在表格下面，不被刷新盖掉
        duty_prev = {}                                 # pwm 模式：上一拍的占空比（饱和时电流环不积分）
        n_lines = len(joints) + 6
        print("\n" * n_lines, end="")
        period = 1.0 / args.hz
        t0 = time.time(); t_prev, t_report, t_wd, n_loop = 0.0, t0, t0, 0
        t_gain = t0
        hk_step = 0
        vA = leader.voltage()
        vB = follower.voltage() if follower is not None else float("nan")
        q_cmd = None                                   # 低通后的从手目标
        ref = {j: {"q": None, "v": 0.0, "a": 0.0} for j in joints}     # 从手参考轨迹的速度/加速度（--follower-mode current 的前馈用）
        fhat = {j: 0.0 for j in joints}                 # 从手接触力估计（从手空间，N·m），--fb-force 用
        rfo = {j: {"tau": 0.0, "mdl": 0.0, "a": 0.0} for j in joints}   # 反作用力观测器里三个同带宽低通的状态
        tauB_cmd = {j: 0.0 for j in joints}             # 从手指令力矩（current 模式）
        tau_f = {j: 0.0 for j in joints}
        gate = {j: 0.0 for j in joints}                # 连续门控 0–1：从手电流大且几乎不动 = 被挡住
        contact = {j: False for j in joints}
        errors = 0
        log = []
        if follower is not None and not args.no_parallel:
            class _Reader(threading.Thread):
                """常驻线程：主线程 set(go) 后读一次从手状态，读完 set(done)"""
                def __init__(self, bus):
                    super().__init__(daemon=True)
                    self.bus, self.go, self.done, self.st, self.err = bus, threading.Event(), threading.Event(), None, None
                def run(self):
                    while True:
                        self.go.wait(); self.go.clear()
                        try:
                            self.st = self.bus.read_state()
                        except IOError as ex:
                            self.err = ex
                        self.done.set()
            reader = _Reader(follower); reader.start()
        ff_log = {j: 0.0 for j in joints}
        stat_gate, stat_n = {}, 0
        lock = {"on": False, "q": None, "n": 0}
        if args.lock:
            def _lock_thread():
                while True:
                    input()
                    lock["on"] = not lock["on"]
                    lock["n"] += 1
            threading.Thread(target=_lock_thread, daemon=True).start()
            print("* 锁定测试：按回车 = 把现在的姿态锁成虚拟墙，再按回车解锁")
        while True:
            t_loop = time.time()
            try:
                if follower is not None and not args.no_parallel:
                    reader.go.set()                                # 常驻线程读从手，主线程同时读主手（不再每周期新建线程）
                    stA = leader.read_state()
                    reader.done.wait()
                    reader.done.clear()
                    if reader.err is not None:
                        ex, reader.err = reader.err, None
                        raise ex
                    stB = reader.st
                else:
                    stA = leader.read_state()
                    stB = follower.read_state() if follower is not None else None
                errors = 0
            except IOError as e:
                errors += 1
                for b in (leader, follower):                       # SDK 一次收发失败后可能把串口标成"占用"，手动复位
                    if b is not None:
                        b.port.is_using = False
                if errors >= 5:
                    raise IOError(f"连续 5 次读取失败：{e}")
                continue
            qA = leader_angles(stA)
            if stB is not None:
                qB = {j: sB[j] * ticks_to_rad(stB[ids[j]][2] - zB[j]) for j in joints}
                if not args.dry_run:
                    # 墙开着的关节：从手目标冻结在墙的位置。墙是静止的障碍，从手不该追着主手的过冲跑，否则松手时两边一起飞
                    qA_cmd = {j: (wall_qB[j] + wall_dir[j] * math.radians(args.wall_press) if (args.wall and wall_on[j]) else qA[j]) for j in joints}
                    if args.cmd_lpf > 0:
                        dt_c = (t_loop - t_prev) if t_prev else period
                        a_c = dt_c / (dt_c + 1 / (2 * math.pi * args.cmd_lpf))
                        q_cmd = dict(qA_cmd) if q_cmd is None else {j: q_cmd[j] + a_c * (qA_cmd[j] - q_cmd[j]) for j in joints}
                        q_tgt = q_cmd
                    else:
                        q_tgt = qA_cmd
                    if args.follower_mode == "current":
                        # 从手自己的位置环 + 模型前馈：τ = K(q_ref−q) + B(v_ref−v̂) + J·a_ref + b·v_ref + c·tc·sgn(v_ref)
                        # 刚度按每个电机的惯量定带宽：K = J·(2πf)²，B = 2ζ√(KJ)。摩擦沿参考速度方向补，从手不再一格一格跳
                        dt_r = max((t_loop - t_prev) if t_prev else period, 1e-4)     # dt 在这段后面才算
                        a_rv = dt_r / (dt_r + 1 / (2 * math.pi * args.obs_hz)); a_ra = dt_r / (dt_r + 1 / (2 * math.pi * 6))
                        icmd = {}
                        for j in joints:
                            r = ref[j]; m_ = MPF[j]
                            if r["q"] is not None and abs(q_tgt[j] - r["q"]) < 1.0:
                                v_new = r["v"] + a_rv * ((q_tgt[j] - r["q"]) / dt_r - r["v"])
                                r["a"] += a_ra * ((v_new - r["v"]) / dt_r - r["a"]); r["v"] = v_new
                            r["q"] = q_tgt[j]
                            K_s = m_["J"] * (2 * math.pi * fbw[j]) ** 2
                            B_s = 2 * args.follower_zeta * math.sqrt(K_s * m_["J"])
                            ramp_r = min(1.0, max(0.0, (abs(r["v"]) - 0.05) / 0.2)) * math.copysign(1.0, r["v"])
                            tau = (K_s * (q_tgt[j] - qB[j]) + B_s * (r["v"] - obs[j]["vB"]) + m_["J"] * r["a"] + m_["b"] * r["v"]
                                   + args.follower_comp * m_["tc"] * ramp_r)
                            lim = args.follower_limit / 1000 * m_["kt"]
                            tau = float(np.clip(tau * min(1.0, (t_loop - t0) / 1.5), -lim, lim))      # 前 1.5 s 软启动（ramp 在后面才算）
                            tauB_cmd[j] = tau
                            icmd[ids[j]] = sB[j] * tau / m_["kt"] * 1000
                        follower.write_currents(icmd)
                    else:
                        follower.write_positions(follower_goal_ticks(q_tgt))
            elif args.lock:
                if lock["on"] and lock["q"] is None:
                    lock["q"] = dict(qA)                    # 锁定那一刻的姿态就是"从手被挡住的位置"
                    alerts.append(f"{time.strftime('%H:%M:%S')} 已锁定第 {(lock['n'] + 1) // 2} 个姿态，用力推")
                    del alerts[:-3]
                elif not lock["on"]:
                    lock["q"] = None
                qB = dict(lock["q"]) if lock["q"] is not None else dict(qA)
            else:
                qB = dict(qA)
            ramp = min(1.0, (t_loop - t0) / 1.5)
            dt = t_loop - t_prev if t_prev else period
            t_prev = t_loop
            alpha = 1.0 if args.lpf <= 0 else dt / (dt + 1 / (2 * math.pi * args.lpf))
            if stB is not None and args.adaptive_kp is not None:
                # 方法 3：K_t = f(τ_t)。接触电流（滑动平均）越大，从手刚度越低；空中回到最高
                kmax, kmin = args.adaptive_kp
                beta = dt / (dt + args.adapt_tau)
                for j in joints:
                    iB_avg[j] += beta * (abs(stB[ids[j]][0]) - iB_avg[j])
                    kp_cmd[j] = kmax - (kmax - kmin) * min(1.0, iB_avg[j] / args.adapt_ref)
                if not args.dry_run and t_loop - t_gain > 0.02:     # 最多 50 Hz 写增益，变化 ≥ 5 才写
                    changed = {ids[j]: kp_cmd[j] for j in joints if abs(kp_cmd[j] - kp_written[j]) >= 5}
                    if changed:
                        follower.write_p_gains(changed)
                        for j in joints:
                            if ids[j] in changed:
                                kp_written[j] = kp_cmd[j]
                    t_gain = t_loop
            # 速度观测：位置差分 + 40 Hz 低通（电机报的速度滞后 22 ms，不能用来做阻尼）
            dt_o = max(dt, 1e-4)                                   # dt 在上面已经算好（t_prev 此时已更新，不能再减）
            # 编码器 1 tick = 0.088°，750 Hz 下差分一跳就是 1 rad/s，速度低通 15 Hz、加速度再低通 6 Hz 才压得住
            a_v = dt_o / (dt_o + 1 / (2 * math.pi * args.obs_hz))
            a_a = dt_o / (dt_o + 1 / (2 * math.pi * 6))
            for j in joints:
                o = obs[j]
                qL = sA[j] * ticks_to_rad(stA[ids[j]][2] - zA[j])                   # 主手自己坐标系的角度（多圈连续）
                if o["qL"] is not None and abs(qL - o["qL"]) < 1.0:               # 一拍跳 1 rad 以上是 ±180° 折返，不算速度
                    o["vL"] += a_v * ((qL - o["qL"]) / dt_o - o["vL"])
                o["qL"] = qL
                if stB is not None:
                    if o["qB"] is not None and abs(qB[j] - o["qB"]) < 1.0:
                        vb_prev = o["vB"]
                        o["vB"] += a_v * ((qB[j] - o["qB"]) / dt_o - o["vB"])
                        o["aB"] += a_a * ((o["vB"] - vb_prev) / dt_o - o["aB"])
                    o["qB"] = qB[j]
                    o["iBf"] += a_a * (stB[ids[j]][0] - o["iBf"])           # 实测电流同样 6 Hz 低通，和模型算的空载电流同一带宽
                    e_now = qA[j] - qB[j]
                    if o["e"] is not None and abs(e_now - o["e"]) < 1.0:
                        o["ed"] += a_v * ((e_now - o["e"]) / dt_o - o["ed"])   # 位置差的导数（接触时 = 主手速度，跟随时 ≈ 0）
                    o["e"] = e_now
            if qA_prev is not None:
                for j in joints:
                    dq = qA[j] - qA_prev[j]
                    if abs(dq) > math.radians(0.3):                    # 目标明显动了才更新方向，静止时保持上一次的方向
                        move_dir[j] = math.copysign(1.0, dq)
            qA_prev = dict(qA)
            err, cur = {}, {}
            for j in joints:
                e = qA[j] - qB[j]
                err[j] = e
                # 单向反馈：只有主手目标在往从手被挡住的那一侧走（或停在那一侧）时才出力。
                # 从手卡住落在后面、而你在往回走时，e 的符号和运动方向相反，不出力，手指不会被往前带
                allow = True
                if not args.two_sided and not args.lock and move_dir[j] != 0.0 and math.copysign(1.0, e) != move_dir[j]:
                    e = 0.0; allow = False
                if j in no_fb:
                    g = 0.0
                elif args.lock:
                    g = 1.0 if lock["q"] is not None else 0.0        # 锁定 = 被挡住，门控全开；解锁 = 空中
                elif stB is None or cma[j] <= 0:
                    g = 1.0
                elif args.gate == "model":
                    # 模型法：从手空载时该有的电流（电机坐标）= (J·a + b·v + tc·tanh(v/v0)) / Kt；实测减去它就是接触力
                    o = obs[j]
                    v_m, a_m = sB[j] * o["vB"], sB[j] * o["aB"]
                    mj = MPF[j]                                                      # 从手电机自己的模型
                    i_free = (mj["J"] * a_m + mj["b"] * v_m + mj["tc"] * math.tanh(v_m / args.v0)) / mj["kt"] * 1000
                    i_free_log[j] = i_free
                    resid = abs(o["iBf"] - i_free)
                    g_i = min(1.0, max(0.0, (resid - 0.5 * cma[j]) / (0.5 * cma[j])))
                    velB = abs(o["vB"])
                    g_v = min(1.0, max(0.0, (args.contact_vel - velB) / (0.75 * args.contact_vel)))      # 从手还在快动就不是被挡住
                    g = g_i * g_v
                else:
                    iB = abs(stB[ids[j]][0]); velB = abs(stB[ids[j]][1])
                    g_i = min(1.0, max(0.0, (iB - 0.5 * cma[j]) / (0.5 * cma[j])))                          # 电流：半值开始，全值全开
                    g_v = min(1.0, max(0.0, (args.contact_vel - velB) / (0.75 * args.contact_vel)))          # 速度：不动全开，快动关闭
                    g = g_i * g_v
                if lag_rad > 0 and stB is not None and not args.lock and j not in no_fb:                  # 落后：到门限一半开始开，到门限全开
                    g *= min(1.0, max(0.0, (abs(e) - 0.5 * lag_rad) / (0.5 * lag_rad)))
                tau_g = args.gate_tau if g > gate[j] else (args.gate_tau_off if args.gate_tau_off is not None else args.gate_tau / 3)
                a_g = dt / (dt + tau_g)                                                                     # 门控低通：开按 gate-tau，关按 gate-tau-off
                gate[j] += a_g * (g - gate[j])
                contact[j] = gate[j] > 0.5
                # e 和 tauB 都在从手空间；乘 dirn[j] 换回主手方向（映射符号相反的关节不乘会变成正反馈，手指被猛拉）
                if args.coupling:
                    o = obs[j]
                    e_raw = qA[j] - qB[j]
                    e_dead = (abs(e_raw) - db) * math.copysign(1, e_raw) if abs(e_raw) > db else 0.0
                    # 从手接触力估计（从手力矩，从手空间符号）：实测电流 − 模型空载电流，都是 6 Hz 带宽
                    i_free = i_free_log[j] if args.gate == "model" else 0.0
                    f_raw = sB[j] * (o["iBf"] - i_free) / 1000 * KT[fmodels[j]] if stB is not None else 0.0
                    # 观测力再过一个很低的低通（--kc-lpf）：6 Hz 以上的 F̂ 是从手追踪加速电流漏进来的假信号，
                    # 直接以 kc=1 反射会和主手形成 10 Hz 的正反馈；反射只取低频"持续顶着"的分量，快的部分交给 K、B
                    a_f = dt / (dt + 1 / (2 * math.pi * max(args.kc_lpf, 0.1)))
                    o["f_hat"] = o.get("f_hat", 0.0) + a_f * (f_raw - o.get("f_hat", 0.0))
                    f_hat = o["f_hat"] if abs(o["f_hat"]) > 0.003 else 0.0           # 3 mN·m 以下当噪声
                    f_hat = float(np.clip(f_hat, -args.max_torque, args.max_torque))
                    t = (-args.kp * e_dead - B_contact[j] * o["ed"] - args.kc * f_hat) * dirn[j]
                    contact[j] = abs(f_hat) > 0.01
                    gate[j] = 1.0 if contact[j] else 0.0                             # 只用于显示和日志
                else:
                    t = -args.kp * (abs(e) - db) * math.copysign(1, e) * gate[j] * dirn[j] if abs(e) > db else 0.0
                if stB is not None and args.kc > 0 and not args.coupling:
                    # 从手电流（带符号，从手坐标）-> 从手力矩 -> 主手反向；也经过接触门控，空中跟踪的电流不反射
                    tauB = sB[j] * stB[ids[j]][0] / 1000 * KT[fmodels[j]]
                    th = 0.03
                    if abs(tauB) > th:
                        t -= args.kc * (abs(tauB) - th) * math.copysign(1, tauB) * gate[j] * dirn[j]
                if args.fb_force > 0 and stB is not None and MPF is not None:
                    # 从手接触力估计（反作用力观测器）：从手出的力矩 − 让它自己这么动需要的力矩。
                    # 三项都过同一个低通（--fb-force-lpf）再相减：指令力矩是瞬时的，模型里的速度、加速度是滞后的，不配平滤波的话
                    # 主手一加速，差值就是一个和加速度同相的假接触力，反射回去就是负惯量。摩擦项用和前馈同样的形状，免得前馈本身被当成接触力
                    o = obs[j]; m_ = MPF[j]; rf = rfo[j]
                    v_m = sB[j] * o["vB"]
                    a_f = dt / (dt + 1 / (2 * math.pi * max(args.fb_force_lpf, 0.1)))
                    tau_out = tauB_cmd[j] if args.follower_mode == "current" else sB[j] * stB[ids[j]][0] / 1000 * m_["kt"]
                    fr_shape = min(1.0, max(0.0, (abs(v_m) - 0.05) / 0.2)) * math.copysign(1.0, v_m)
                    rf["tau"] += a_f * (tau_out - rf["tau"])
                    rf["mdl"] += a_f * ((m_["b"] * v_m + (args.follower_comp if args.follower_mode == "current" else 1.0) * m_["tc"] * fr_shape) - rf["mdl"])
                    rf["a"] += a_f * (sB[j] * o["aB"] - rf["a"])                        # aB 本身已是 6 Hz，这里再同步一次相位
                    fhat[j] = rf["tau"] - m_["J"] * rf["a"] - rf["mdl"]
                    th_f = args.fb_force_th / 1000
                    if abs(fhat[j]) > th_f:
                        t -= args.fb_force * (abs(fhat[j]) - th_f) * math.copysign(1.0, fhat[j]) * dirn[j]
                if MP and not args.wall and not args.coupling:
                    vL = obs[j]["vL"]
                    t -= B_contact[j] * vL * gate[j]                         # 接触时按阻尼比给阻尼，用观测速度
                    # 推人时的效率补偿：电机出力方向和运动方向一致 = 电机在推人，摩擦从出力里扣，按 (1+φ) 放大；只在接触时
                    if gate[j] > 0 and abs(t) > 1e-6:
                        drive = max(0.0, math.tanh(vL * math.copysign(1.0, t) / args.v0))
                        t *= 1.0 + phi_eff[j] * drive * gate[j]
                t = float(np.clip(t * ramp * fb_scale[j], -args.max_torque, args.max_torque))
                tau_f[j] += alpha * (t - tau_f[j])
                ff = 0.0
                velA = sA[j] * stA[ids[j]][1]
                if MP and args.comp_gain > 0:
                    # 空载库仑摩擦前馈（墙模式里由饱和位置环实现）：|v| < 0.3 rad/s 不补（零速附近有斜率就是负阻尼），0.3–0.8 线性过渡；只补空载那几 mN·m
                    vL = obs[j]["vL"]
                    ramp_v = min(1.0, max(0.0, (abs(vL) - args.v_dead) / args.v_ramp)) * math.copysign(1.0, vL)
                    ff += args.comp_gain * MPJ[j]["tc"] * ramp_v / MPJ[j]["kt"] * 1000 * ramp
                    if args.b_free > 0:
                        # 空载粘滞让它停下来，但封顶在"补掉的那部分库仑摩擦"：再快也不会比关扭矩更重，慢速照样轻
                        visc = min(args.b_free * abs(vL), args.comp_gain * MPJ[j]["tc"]) * math.copysign(1.0, vL)
                        ff -= visc * (1.0 - gate[j]) / MPJ[j]["kt"] * 1000 * ramp
                if args.friction_ma > 0:                                   # 摩擦补偿：顺着主手运动方向；|v| < 0.15 rad/s 不补，免得静止时抖
                    v0 = 0.15
                    ff = args.friction_ma * math.tanh((abs(velA) - v0) / 0.3) * math.copysign(1, velA) * ramp if abs(velA) > v0 else 0.0
                if args.visc_ma > 0:                                       # 粘滞补偿：和速度成正比，空中抵消电流模式的电磁阻尼
                    ff += args.visc_ma * velA * ramp * (1.0 - gate[j])
                if args.damp > 0 and gate[j] > 0:                          # 接触时的阻尼：逆着速度，压抖动（换成电流加到 ff 里）
                    ff -= args.damp * velA * gate[j] / KT[models[j]] * 1000
                ff_log[j] = ff                                             # 墙模式下 ff 也用：自由段靠"目标放在前方 + 电流上限 = |ff|"实现前馈
                if args.wall:
                    # 墙模式：主手位置环顶在从手被挡住的位置上，这里只给"最多能出多大力"：门控 × 力矩上限（+ 从手力矩反射）
                    tau_cap = args.max_torque
                    if stB is not None and args.kc > 0:
                        tauB = abs(sB[j] * stB[ids[j]][0] / 1000 * KT[fmodels[j]])
                        tau_cap += args.kc * max(0.0, tauB - 0.03)
                    if MP:
                        # 推人（主手正被墙推回去）时摩擦从电机出力里扣，上限加上摩擦；顶人（你在往墙里推）时摩擦帮电机，上限减掉
                        q_wall_j = jmap.inverse(qB)[j] if (stB is not None or args.lock) else obs[j]["qL"]
                        toward = math.copysign(1.0, q_wall_j - obs[j]["qL"]) if abs(q_wall_j - obs[j]["qL"]) > 1e-4 else 0.0
                        drive = math.tanh(obs[j]["vL"] * toward / args.v0)             # +1 往墙的位置走 = 电机在推人
                        phi = MPJ[j]["k_load"] / r_load[j]                               # 负载摩擦 / 力矩
                        tau_cap *= (1.0 + phi * drive)
                    i_max = tau_cap / kt_lead[j] * 1000
                    # 门控只用来【开】墙。开着期间墙是静止的障碍：从手目标停在墙里 wall-press 度（保持一个稳定的接触电流），
                    # 主手被按在墙面上、上限全额；什么时候关墙看几何：主手退回到墙面外侧就关，不看门控（看门控会抖：
                    # 从手一停下来电流就掉、门控就关、从手又追上去顶、门控又开……6 Hz 的开关，开关间隙里主手就穿过去了）
                    if not wall_on[j] and gate[j] > 0.5 and allow:
                        wall_on[j] = True
                        wall_hold[j] = (obs[j]["qL"], t_loop)                 # 开墙瞬间先把主手按在原地，再在 100 ms 内滑到墙
                        wall_qB[j] = qB[j]                                    # 墙 = 开墙瞬间从手的位置
                        wall_dir[j] = math.copysign(1.0, e) if abs(e) > 1e-4 else move_dir[j]   # 主手从哪一侧压进来（从手空间）
                    elif wall_on[j]:
                        depth = (qA[j] - wall_qB[j]) * wall_dir[j]           # 主手目标在墙里多深（从手空间）
                        if depth < -math.radians(1.0):                        # 退到墙面外 1° 以上：关墙，恢复跟随
                            wall_on[j] = False
                    cap_contact = i_max * fb_scale[j] * ramp if wall_on[j] else 0.0
                    tau_f[j] += alpha * (cap_contact * kt_lead[j] / 1000 - tau_f[j])  # 低通：开墙从 0 渐入，记录用
                    cap_contact = float(np.clip(tau_f[j] / kt_lead[j] * 1000, 0, args.leader_limit))
                    # 自由段的摩擦前馈：模式 5 里没法直接给电流，但把目标位置放到运动方向前方 0.5 rad、电流上限设成 |ff|，
                    # 位置环立刻饱和在上限上，效果就是沿运动方向的恒流 —— 用饱和的位置环当电流模式
                    cap_free = min(abs(ff), args.leader_limit)
                    wall_free_dir[j] = math.copysign(1.0, ff) if abs(ff) > 0.5 else 0.0
                    cur[ids[j]] = cap_contact if wall_on[j] else cap_free
                    if not wall_on[j]:
                        tau_f[j] = 0.0
                else:
                    cur[ids[j]] = float(np.clip(sA[j] * (tau_f[j] / kt_lead[j] * 1000 + ff), -args.leader_limit, args.leader_limit))
            if args.wall:
                # 墙模式里 cur 是电流上限不是实际电流：按实测总电流限制，超过预算才把上限按比例压下来
                meas = sum(abs(stA[i][0]) for i in cur)
                if meas > args.budget:
                    cur = {i: v * args.budget / meas for i, v in cur.items()}
            else:
                total = sum(abs(v) for v in cur.values())
                if total > args.budget:
                    cur = {i: v * args.budget / total for i, v in cur.items()}
            if args.log is not None:
                row = [t_loop - t0]
                for j in joints:
                    i = ids[j]
                    row += [qA[j], sA[j] * stA[i][1], stA[i][0], cur[i], qB[j],
                            (sB[j] * stB[i][1]) if stB else 0.0, (stB[i][0] if stB else 0), round(kp_cmd[j]),
                            round(gate[j], 3), err[j], round(tau_f[j], 4), round(ff_log[j], 1), move_dir[j],
                            round(obs[j]["vL"], 3), round(i_free_log[j], 1)]
                log.append(row)
                stat_gate[j] = stat_gate.get(j, 0) + (1 if gate[j] > 0.5 else 0)
                stat_n += 1
            if not args.dry_run:
                if args.wall and (stB is not None or args.lock):
                    # 墙的位置 = 从手现在的位置（锁定测试里是锁定姿态）换回主手空间；力的大小 = cur（门控 × 上限），方向由位置环自己决定
                    q_wall = jmap.inverse({j: (wall_qB[j] if wall_on[j] else qB[j]) for j in joints})
                    goal = {}
                    for j in joints:
                        if not wall_on[j]:
                            q_goal = obs[j]["qL"] + wall_free_dir[j] * 0.5      # 自由段：目标在前方 0.5 rad，电流上限 = |ff|（ff=0 时上限 0，目标无所谓）
                        else:
                            q0_, t0_ = wall_hold[j]
                            s_ = min(1.0, (t_loop - t0_) / 0.1)
                            q_goal = q0_ + (q_wall[j] - q0_) * s_               # 刚开墙：先顶在主手当前位置（不再穿透），100 ms 内滑到真正的墙
                        tgt = zA[j] + sA[j] * q_goal * TICKS_PER_REV / (2 * math.pi)
                        pos = stA[ids[j]][2]
                        goal[ids[j]] = pos + wrap_ticks(round(tgt - pos))      # 就近折返：零点在 0/4095 边缘的关节不会转一整圈
                    leader.write_positions(goal)
                    leader.write_currents({i: abs(v) for i, v in cur.items()})
                elif args.mode == "pwm":
                    # 力矩(电流当量 mA × Kt) → 电压：τ/G 是堵转所需电压，k·Ke·v̂ 抵消反电动势（k = 空载抵消比例 × (1 − 门控)）。
                    # 门控开着（接触、松手那一瞬间）k→0，电机自己的电磁阻尼全额作用在主手上，不经过任何延迟
                    v_bus = vA if (vA == vA and vA > 4.0) else 6.0
                    duty = {}
                    for j in joints:
                        i = ids[j]
                        tau_cmd = cur[i] / 1000 * kt_lead[j]                       # 电机坐标（cur 已含 sA）
                        v_raw = sA[j] * obs[j]["vL"]                               # 电机坐标的观测速度
                        free = max(0.0, 1.0 - gate[j] / 0.3)                       # 门控 >0.3 当接触：前馈和电流环都停，纯电压模式
                        g_, ke_, r_ = pwm_g[j], pwm_ke[j], pwm_r[j]
                        v_ff = args.pwm_cancel * free * ke_ * v_raw                 # 反电动势前馈（只抵消一部分，估高了是负阻尼）
                        # 软件电流环：实测电流追指令电流的积分修正。慢（--pwm-iloop），所以低频像电流模式、高频保留电磁阻尼。
                        # 积分只允许表示反电动势：|u_I| ≤ 1.3·Ke·|v̂| + 0.3 V，堵转时它就是 ~0，松手瞬间不会有积分在推
                        if free > 0 and args.pwm_iloop > 0 and abs(duty_prev.get(i, 0.0)) < args.pwm_limit - 1e-6:
                            ki_ = 2 * math.pi * args.pwm_iloop * r_eff(j, cur[i] / 1000, v_bus)       # 交越 = Ki/R_表观，断续导通区自动放大
                            pwm_ui[j] += ki_ * (cur[i] - stA[i][0]) / 1000 * dt
                        lim_ui = 1.3 * ke_ * abs(v_raw) + 0.5
                        pwm_ui[j] = float(np.clip(pwm_ui[j], -lim_ui, lim_ui)) * (1.0 if free > 0 else 0.98)   # 接触时慢慢放掉
                        d = (duty_for_current(j, cur[i] / 1000, v_bus) + (v_ff + pwm_ui[j]) / v_bus) * 100   # 力矩部分查表，反电动势和积分是电压
                        # 电压模式没有硬件限流：按 i = (duty·V − Ke·v)/R 把占空比限制在 |i| ≤ leader-limit 内，
                        # 再用实测电流兜底（超 25% 立刻按比例压下来）
                        i_lim = args.leader_limit / 1000
                        d_hi = (duty_for_current(j, i_lim, v_bus) + ke_ * v_raw / v_bus) * 100
                        d_lo = (-duty_for_current(j, i_lim, v_bus) + ke_ * v_raw / v_bus) * 100
                        d = min(max(d, d_lo, -args.pwm_limit), d_hi, args.pwm_limit)
                        i_m = abs(stA[i][0]) / 1000
                        if i_m > 1.25 * i_lim:
                            d *= i_lim / i_m
                        duty[i] = float(d)
                    duty_prev = duty
                    leader.write_pwms(duty)
                else:
                    leader.write_currents(cur)
                # 每秒一次的例行检查（电压、硬件错误、看门狗）拆成 6 小步，每个周期只做一步、一次快速读（≈1 ms），
                # 不再一次卡住 5–20 ms
                if t_loop - t_wd > 1.0 and hk_step == 0:
                    hk_step = 1
                if hk_step:
                    which, b = [(None, None), ("主手", leader), ("从手", follower), ("主手", leader), ("从手", follower),
                                ("主手", leader), ("从手", follower)][hk_step]
                    if b is not None:
                        if hk_step <= 2:
                            vt = b.fast_voltage_temp()
                            v_ = float(np.median([x[0] for x in vt.values()])) if vt else b.voltage()
                            if which == "主手":
                                vA = v_
                            else:
                                vB = v_
                        elif hk_step <= 4:
                            hw = b.fast_hw_errors()
                            if hw is None:
                                hw = b.hardware_errors()
                            if hw:
                                msg = f"{which}硬件保护: " + "，".join(f"ID {i} {'/'.join(v)}" for i, v in hw.items()) + "（扭矩已被电机关掉）"
                                if not alerts or alerts[-1][9:] != msg:
                                    alerts.append(f"{time.strftime('%H:%M:%S')} {msg}")
                                del alerts[:-3]
                        elif b.check_watchdog():
                            alerts.append(f"{time.strftime('%H:%M:%S')} {which}看门狗触发过，已重新打开")
                            del alerts[:-3]
                    hk_step = 0 if hk_step >= 6 else hk_step + 1
                    if hk_step == 0:
                        t_wd = t_loop
            n_loop += 1
            now = time.time()
            if now - t_report > 0.25:
                fv = lambda v: "--" if (v != v or v < 1.0) else f"{v:.1f} V"
                lines = [f"控制 {n_loop / (now - t_report):4.0f} Hz | 主手 {fv(vA)} 电流合计 指令 {sum(abs(v) for v in cur.values()):5.0f} mA  实测 {sum(abs(stA[i][0]) for i in cur):5.0f} mA"
                         + (f" | 从手 {fv(vB)} 电流合计 {sum(abs(stB[i][0]) for i in cur):5.0f} mA" if stB else ""),
                         f"{'ID':>3} {'关节':22s} {'主手°':>7s} {'从手°':>7s} {'差°':>6s} {'主手mA':>7s} {'从手mA':>7s} {'从手Kp':>6s} 接触"]
                for j in joints:
                    i = ids[j]
                    lines.append(f"{i:3d} {j:22s} {math.degrees(qA[j]):+7.1f} {math.degrees(qB[j]):+7.1f} {math.degrees(err[j]):+6.1f} {cur[i]:+7.0f} {(stB[i][0] if stB else 0):+7d} {kp_cmd[j]:6.0f}  {'●' if contact[j] else ''}")
                lines += ["!! " + a for a in alerts] + [""] * (3 - len(alerts))
                lines.append("")
                print(f"\033[{n_lines}F" + "\n".join(l + "\033[K" for l in lines), flush=True)
                n_loop, t_report = 0, now
            sleep = period - (time.time() - t_loop)
            if sleep > 0:
                time.sleep(sleep)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        import traceback
        print(f"\n!! 出错，已停止: {e!r}")
        traceback.print_exc()
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if args.log is not None and log:
            import csv
            os.makedirs(os.path.dirname(os.path.abspath(args.log)), exist_ok=True)
            with open(args.log, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["t"] + [f"{j}:{k}" for j in joints for k in ("qA", "dqA", "iA_meas", "iA_cmd", "qB", "dqB", "iB_meas", "kpB", "gate", "err", "tau_fb", "ff_mA", "dir", "vL_obs", "iB_free")])
                w.writerow(["# models"] + [f"{j}:{MODEL_NAME[models[j]]}" for j in joints])
                w.writerow(["# settings", f"follower_kp:{kp0}",
                            f"follower_kd:{args.follower_kd if args.follower_kd is not None else 0}", f"follower_ki:{args.follower_ki or 0}", f"follower_limit_mA:{args.follower_limit}",
                            f"leader_kp:{args.kp}", f"deadband_deg:{args.deadband}",
                            f"adaptive_kp:{'%d/%d/%.0f' % (args.adaptive_kp[0], args.adaptive_kp[1], args.adapt_ref) if args.adaptive_kp else 'off'}"])
                w.writerows(log)
            print(f"* 已保存 {len(log)} 行到 {args.log}（每关节 15 列：qA dqA iA_meas iA_cmd qB dqB iB_meas kpB gate err tau_fb ff_mA dir vL_obs iB_free；"
                  f"python tools/plot_log.py {args.log} 画图）")
            if stat_n:
                print("  接触门控打开的时间占比：" + "，".join(f"{j.replace('_motor', '')} {100 * stat_gate.get(j, 0) / stat_n:.0f}%" for j in joints))
        if not args.dry_run:
            leader.safe_stop()
            if follower is not None:
                follower.safe_stop()
            print("* 两边都已清零电流并关扭矩")
        leader.close()
        if follower is not None:
            follower.close()


if __name__ == "__main__":
    main()
