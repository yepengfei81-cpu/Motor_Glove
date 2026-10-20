#!/usr/bin/env python3
"""
统一入口：按"设备 + 命令"调用各个脚本，串口、标定文件、模型都从 config/devices.json 取，不用每次记参数。
设备：A = 手套 A（戴手上），B = 手套 B，H = 直驱机械手。

  电机 / ID
    python tools/mg.py H show                  # 总线上有哪些电机（1 Mbps，含电压、硬件错误）
    python tools/mg.py H probe                 # 所有波特率、所有 ID 扫一遍（找出厂设置的、改丢了的电机）
    python tools/mg.py H ids thumb             # 核对 ID：不用标定，仿真里显示机械手，你动哪个关节、仿真里就该是哪个关节在动
    python tools/mg.py H set thumb_ip_motor    # 给链上唯一一个新电机设 ID（一次只接一个新的）
    python tools/mg.py H reassign 5 index_pip_motor [--duplicate]   # ID 设错了，改过来
    python tools/mg.py H verify                # 逐个 ping 查撞号，再逐个关节动一下核对

  标定 / 检查
    python tools/mg.py H calib thumb           # 逐个关节标零点和方向（仿真窗口显示机械手）
    python tools/mg.py H calib thumb --joint thumb_ip_motor
    python tools/mg.py H live                  # 标定后实时镜像到仿真
    python tools/mg.py A zero                  # 位置模式回零并保持，大电流下看偏差（默认 1750 mA）
    python tools/mg.py A stops save            # 机械限位快照（大电流测试前）
    python tools/mg.py A stops compare         # 测试后再比一次，看零点有没有真的漂

  两台设备
    python tools/mg.py teleop A B thumb index [bilateral.py 的参数]   # 双边遥操作（目前从手只支持手套）
    python tools/mg.py jog A B                 # 两只手套位置模式逐关节对照、微调 B 的零点

每次都会先打印实际执行的命令，想直接用底层脚本时照抄即可。
"""
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import devices  # noqa: E402

TOOLS = os.path.dirname(os.path.abspath(__file__))
SETUP_CMDS = {"show", "scan", "status", "verify", "reboot", "blink", "set", "reassign", "forget"}


def run(script, *args):
    cmd = [sys.executable, os.path.join(TOOLS, script), *map(str, args)]
    print("$ python tools/" + " ".join([script, *map(str, args)]), flush=True)
    sys.exit(subprocess.call(cmd))


def main():
    a = sys.argv[1:]
    if len(a) < 2 or a[0] in ("-h", "--help"):
        print(__doc__)
        sys.exit(0)
    if a[0] == "teleop":
        if len(a) < 3:
            sys.exit("用法: mg.py teleop <主手设备> <从手设备> [手指...] [bilateral.py 参数]")
        L, F = devices.get(a[1]), devices.get(a[2])
        if L["model"] != "glove" or F["model"] != "glove":
            sys.exit("目前双边遥操作只支持手套对手套；机械手作从手要用 config/glove_to_hand.json 的映射，下一步接进 bilateral.py")
        run("bilateral.py", *a[3:], "--leader-port", L["port"], "--leader-glove", L["name"],
            "--follower-port", F["port"], "--follower-glove", F["name"])
    if a[0] == "jog":
        if a[1:3] != ["A", "B"]:
            sys.exit("jog 目前只支持 A B（twin_jog.py 读的是手套 A、B 的标定）")
        run("twin_jog.py", *a[3:], "--a-port", devices.get("A")["port"], "--b-port", devices.get("B")["port"])

    dev, cmd, rest = devices.get(a[0]), a[1], a[2:]
    print(f"* 设备 {dev['name']}：{dev['desc']}，串口 {dev['port']}，模型 {dev['model']}")
    if cmd in SETUP_CMDS:
        if cmd == "verify":
            rest = rest or ["--move"]
        run("dxl_setup.py", "--port", dev["port"], "--glove", dev["name"], cmd, *rest)
    if cmd == "probe":
        run("dxl_probe.py", "--port", dev["port"], *rest)
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
    if cmd == "stops":
        if not rest or rest[0] not in ("save", "compare"):
            sys.exit("用法: mg.py <设备> stops save|compare [手指...]")
        run("zero_check.py", dev["name"], *rest[1:], "--stops", rest[0])
    sys.exit(f"不认识的命令 {cmd}。python tools/mg.py -h 看全部命令")


if __name__ == "__main__":
    main()
