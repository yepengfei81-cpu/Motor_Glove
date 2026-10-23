#!/usr/bin/env python3
"""
统一入口：按"设备 + 命令"调用各个脚本，串口、标定文件、模型都从 config/devices.json 取，不用每次记参数。
设备：A = 手套 A（戴手上），B = 手套 B，H = 直驱机械手。

  电机 / ID
    python tools/mg.py H show                  # 总线上有哪些电机（1 Mbps，含电压、硬件错误）
    python tools/mg.py H probe                 # 所有波特率、所有 ID 扫一遍（找出厂设置的、改丢了的电机）
    python tools/mg.py A baud 3000000          # 把这台设备的电机全部改到 3 Mbps（新标准；Bus 打开时会自动找 3M/1M）
    python tools/mg.py H ids thumb             # 核对 ID：不用标定，仿真里显示机械手，你动哪个关节、仿真里就该是哪个关节在动
    python tools/mg.py H set thumb_ip_motor    # 给链上唯一一个新电机设 ID（一次只接一个新的）
    python tools/mg.py H reassign 5 index_pip_motor [--duplicate]   # ID 设错了，改过来
    python tools/mg.py H verify                # 逐个 ping 查撞号，再逐个关节动一下核对

  标定 / 检查
    python tools/mg.py H calib thumb           # 逐个关节标零点和方向（仿真窗口显示机械手）
    python tools/mg.py H calib thumb --joint thumb_ip_motor
    python tools/mg.py H live                  # 标定后实时镜像到仿真（--cam side 从侧面看）
    python tools/mg.py A live --tips --cam side   # 画出标定好的指腹球、实时打印拇指到各指间隙（演示指尖对齐用）
    python tools/mg.py A live --show hand --cam side --map config/map_AB.json --scale thumb_mcp_flex_motor=1.3
                                               # 手套 A 驱动仿真里的机械手：预览遥操作映射，不碰真机
    python tools/mg.py A zero                  # 位置模式回零并保持，大电流下看偏差（默认 1750 mA）
    python tools/mg.py A zero --here           # 戴着摆好姿势再启动，锁在这个姿态，用力推：测机械柔度
    python tools/mg.py teleop A H --lock ...   # 不用从手：回车把当前姿态锁成虚拟墙，用遥操作同样的反馈律顶你，测力反馈本身
    python tools/mg.py A stops save            # 机械限位快照（大电流测试前）
    python tools/mg.py A stops compare         # 测试后再比一次，看零点有没有真的漂
    python tools/mg.py A stops save --joint thumb_mcp_flex_motor --repeat 6   # 一个关节反复推限位，查打滑
    python tools/mg.py A tips record           # 自碰触标定：戴着手套做指尖接触，录样本
    python tools/mg.py A tips record --cuffs   # 手套脱下：拇指指套尖端对其他指套尖端，之后 tips check 检验模型本身
    python tools/mg.py A tips record --known   # 拇指指腹顶在食指/中指/无名指指套尖端（CAD 已知点），专门标拇指
    python tools/mg.py A tips solve            # 解你的指尖球心 -> config/tips_A.json，并存检查图 data/tips_A.png
    python tools/mg.py A tips solve --zeros thumb   # 指尖和拇指零点一起解（--apply 才写回标定）
    python tools/mg.py H tips record           # 机械手：扭矩关，手动把它的指尖捏到一起
    python tools/mg.py H tips solve --apply    # 解机械手 13 个零点误差并写回标定
    python tools/mg.py A jog                   # 位置模式回零，逐个关节微调零点（不改方向），点保存写回标定
    python tools/mg.py A jog thumb             # 只调拇指

  两台设备
    python tools/mg.py teleop A B thumb index [bilateral.py 的参数]   # 双边遥操作，从手是手套
    python tools/mg.py teleop T1 T2 thumb --preset bench_free      # 预设参数（config/teleop_presets.json），后面可以再加参数覆盖
    python tools/mg.py teleop A H thumb index --map config/map_AB.json --scale thumb_mcp_flex_motor=1.5
                                               # 从手是机械手：自动套 config/glove_to_hand.json；--map/--scale 是对手套 A 的经验修正
    python tools/mg.py jog A B                 # 两台同模型设备逐关节对照，以 A 为准微调 B 的零点
    python tools/mg.py fitmap A H --closed 3   # 戴着 A 捏食指/中指/无名指，机械手自动合到碰上，拟合 A->H 的关节映射 -> config/map_AH.json

每次都会先打印实际执行的命令，想直接用底层脚本时照抄即可。
"""
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import devices  # noqa: E402

TOOLS = os.path.dirname(os.path.abspath(__file__))
SETUP_CMDS = {"show", "scan", "status", "verify", "reboot", "blink", "set", "reassign", "forget"}


def gm_fingers():
    import glove_model
    return list(glove_model.FINGERS)


def run(script, *args):
    cmd = [sys.executable, os.path.join(TOOLS, script), *map(str, args)]
    print("$ python tools/" + " ".join([script, *map(str, args)]), flush=True)
    p = subprocess.Popen(cmd)
    while True:
        try:
            sys.exit(p.wait())
        except KeyboardInterrupt:          # Ctrl+C 交给子脚本自己收尾（关扭矩），这里只等它退出
            continue


def main():
    a = sys.argv[1:]
    if len(a) < 2 or a[0] in ("-h", "--help"):
        print(__doc__)
        sys.exit(0)
    if a[0] == "teleop":
        if len(a) < 3:
            sys.exit("用法: mg.py teleop <主手设备> <从手设备> [手指...] [bilateral.py 参数]")
        L, F = devices.get(a[1]), devices.get(a[2])
        if L["model"] != "glove":
            sys.exit("主手必须是手套")
        rest = a[3:]
        if "--preset" in rest:                           # 预设参数放前面，命令行里写的参数放后面覆盖它
            import json
            k = rest.index("--preset"); name = rest[k + 1]
            presets = json.load(open(os.path.join(TOOLS, "..", "config", "teleop_presets.json"), encoding="utf-8"))
            if name not in presets:
                sys.exit(f"没有预设 {name}，可选: {[n for n in presets if not n.startswith('_')]}")
            print(f"* 预设 {name}：{presets[name]['desc']}")
            rest = presets[name]["args"] + rest[:k] + rest[k + 2:]
        run("bilateral.py", *rest, "--leader-port", L["port"], "--leader-glove", L["name"],
            "--follower-port", F["port"], "--follower-glove", F["name"], "--follower-model", F["model"])
    if a[0] == "jog":
        if len(a) < 3:
            sys.exit("用法: mg.py jog <参照设备> <要微调的设备> [手指...]")
        run("twin_jog.py", *a[3:], "--dev", a[1], "--dev", a[2])
    if a[0] == "fitmap":
        if len(a) < 3:
            sys.exit("用法: mg.py fitmap <主手> <从手> [--closed N] [--finger thumb --finger index ...]")
        L, F = devices.get(a[1]), devices.get(a[2])
        rest = a[3:]
        if "--finger" not in rest:
            rest += [x for f in gm_fingers() for x in ("--finger", f)]
        run("fit_map.py", *rest, "--leader-port", L["port"], "--leader-glove", L["name"],
            "--follower-port", F["port"], "--follower-glove", F["name"], "--follower-model", F["model"])

    dev, cmd, rest = devices.get(a[0]), a[1], a[2:]
    print(f"* 设备 {dev['name']}：{dev['desc']}，串口 {dev['port']}，模型 {dev['model']}")
    if cmd in SETUP_CMDS:
        if cmd == "verify":
            rest = rest or ["--move"]
        run("dxl_setup.py", "--port", dev["port"], "--glove", dev["name"], cmd, *rest)
    if cmd == "probe":
        run("dxl_probe.py", "--port", dev["port"], *rest)
    if cmd == "baud":
        if not rest:
            sys.exit("用法: mg.py <设备> baud 3000000   # 把这台设备总线上标定过的电机全部改到这个波特率")
        sys.path.insert(0, TOOLS)
        from dxl_bus import Bus, load_registry, load_calibration
        reg, cal = load_registry(), load_calibration(dev["name"])
        ids = [reg[j] for j in cal["joints"] if j in reg]
        bus = Bus(ids, port=dev["port"])
        print(f"* {len(ids)} 个电机现在在 {bus.baud} bps -> {rest[0]} bps")
        bad = bus.set_baud_all(int(rest[0]))
        bus.close()
        sys.exit(f"!! 切换后这些 ID 没应答: {bad}（接到 U2D2 上用 bench_bus.py --set-baud 改回来）" if bad else f"* 全部在 {rest[0]} bps 应答正常")
    if cmd == "ids":
        run("glove_live.py", "--id-check", "--port", dev["port"], "--model", dev["model"], *rest)
    if cmd == "live":
        run("glove_live.py", "--glove", dev["name"], "--port", dev["port"], "--model", dev["model"], *rest)
    if cmd == "calib":
        if not rest:
            sys.exit("用法: mg.py <设备> calib <手指> [--joint 关节名]")
        run("calibrate.py", rest[0], "--glove", dev["name"], "--port", dev["port"], "--model", dev["model"], *rest[1:])
    if cmd == "zero":
        run("zero_check.py", dev["name"], *rest)
    if cmd == "jog":
        run("twin_jog.py", *rest, "--dev", dev["name"])
    if cmd == "tips":
        if not rest or rest[0] not in ("record", "solve", "check"):
            sys.exit("用法: mg.py <设备> tips record|solve|check [tip_calib.py 参数]")
        run("tip_calib.py", dev["name"], *rest)
    if cmd == "stops":
        if not rest or rest[0] not in ("save", "compare"):
            sys.exit("用法: mg.py <设备> stops save|compare [手指...]")
        run("zero_check.py", dev["name"], *rest[1:], "--stops", rest[0])
    sys.exit(f"不认识的命令 {cmd}。python tools/mg.py -h 看全部命令")


if __name__ == "__main__":
    main()
