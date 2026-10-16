"""
DYNAMIXEL 总线的最小封装：1 Mbps，Sync Read 读位置，标定换算成模型关节角。
calibrate.py 和 glove_live.py 共用。
"""
import json
import math
import os
import sys
import time

from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncRead, GroupSyncWrite, COMM_SUCCESS, DXL_LOBYTE, DXL_HIBYTE


def default_port():
    """DXL_PORT 环境变量优先；否则有 U2D2(/dev/ttyUSB*) 就用它，再退回 OpenRB(/dev/ttyACM0)"""
    import glob
    if os.environ.get("DXL_PORT"):
        return os.environ["DXL_PORT"]
    usb = sorted(glob.glob("/dev/ttyUSB*"))
    return usb[0] if usb else "/dev/ttyACM0"


def latency_warning(port):
    """U2D2 是 FTDI 芯片，latency timer 默认 16 ms，读写会很慢"""
    name = os.path.basename(os.path.realpath(port))
    path = f"/sys/bus/usb-serial/devices/{name}/latency_timer"
    if os.path.exists(path):
        v = open(path).read().strip()
        if v != "1":
            return f"提示：{port} 的 USB latency timer 是 {v} ms（建议 1 ms，见 tools/99-dxl-latency.rules）"
    return None


PORT = default_port()

BAUD = 1000000
ADDR_TORQUE, ADDR_PRESENT_POSITION = 64, 132
ADDR_MODEL, ADDR_MODE, ADDR_CURRENT_LIMIT, ADDR_BUS_WATCHDOG, ADDR_GOAL_CURRENT = 0, 11, 38, 98, 102
ADDR_HW_ERROR = 70
ADDR_GOAL_POSITION = 116
ADDR_POS_D_GAIN, ADDR_POS_P_GAIN = 80, 84
HW_ERRORS = {0: "输入电压超限", 2: "过热", 3: "编码器", 4: "电气冲击", 5: "过载"}
ADDR_PRESENT_CURRENT = 126          # 126 电流(2) + 128 速度(4) + 132 位置(4)，一次读 10 字节
VEL_UNIT = 0.229 * 2 * math.pi / 60  # Present Velocity 1 单位 = 0.229 rpm -> rad/s
# 输出轴力矩常数 (N·m/A) 和在我们 5~6 V 供电下的堵转力矩 (N·m)，按 ROBOTIS e-Manual 规格书估算
KT = {1200: 0.52 / 1.47, 1190: 0.215 / 1.47, 1240: 0.517, 1230: 0.333,
      1220: 1.2, 1210: 1.0}           # T 系列（12 V 电机）在 6 V 下只有大约一半的堵转力矩
STALL_TORQUE = {1200: 0.52, 1190: 0.215, 1240: 0.93, 1230: 0.60, 1220: 0.5, 1210: 0.43}
MODEL_NAME = {1200: "XL330-M288", 1190: "XL330-M077", 1240: "XC330-M288", 1230: "XC330-M181",
              1220: "XC330-T288(12V)", 1210: "XC330-T181(12V)"}
TICKS_PER_REV = 4096
ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
REG_FILE = os.path.join(ROOT, "config", "motors.json")
CALIB_FILE = os.path.join(ROOT, "config", "calibration.json")


def to_signed32(v):
    return v - (1 << 32) if v >= (1 << 31) else v


def wrap_ticks(dt):
    """把编码器差值折到 [-2048, 2048)：关节转动不会超过半圈，这样零点跨过 0/4095 也不出错"""
    return (dt + TICKS_PER_REV // 2) % TICKS_PER_REV - TICKS_PER_REV // 2


def ticks_to_rad(dt):
    return dt * 2 * math.pi / TICKS_PER_REV


def load_registry():
    if not os.path.exists(REG_FILE):
        sys.exit("还没有登记任何电机，先用 tools/dxl_setup.py set 设 ID")
    return {name: int(i) for i, name in json.load(open(REG_FILE)).items()}


def calib_path(glove="A"):
    """手套 A（戴在手上的）用 calibration.json，其他手套用 calibration_<名字>.json"""
    return CALIB_FILE if glove in ("A", "", None) else os.path.join(ROOT, "config", f"calibration_{glove}.json")


def load_calibration(glove="A"):
    p = calib_path(glove)
    return json.load(open(p)) if os.path.exists(p) else {"joints": {}}


def save_calibration(cal, glove="A"):
    p = calib_path(glove)
    os.makedirs(os.path.dirname(p), exist_ok=True)
    cal["updated"] = time.strftime("%Y-%m-%d %H:%M:%S")
    json.dump(cal, open(p, "w"), indent=1, ensure_ascii=False)


def present_ids(ids, port=None):
    """ping 一遍，返回有应答的 ID（不像 Bus() 那样缺一个就退出）"""
    ph = PortHandler(port or PORT)
    if not ph.openPort():
        sys.exit(f"打不开 {port or PORT}（板子和电池都接上了吗？）")
    ph.setBaudRate(BAUD)
    pkt = PacketHandler(2.0)
    ok = [i for i in ids if any(pkt.ping(ph, i)[1] == COMM_SUCCESS for _ in range(3))]
    ph.closePort()
    return ok


class Bus:
    def __init__(self, ids, port=None):
        self.ids = list(ids)
        self.portname = port or PORT
        self.port = PortHandler(self.portname)
        try:
            ok = self.port.openPort()
        except Exception:
            ok = False
        if not ok:
            sys.exit(f"打不开 {PORT}（板子和电池都接上了吗？）")
        self.port.setBaudRate(BAUD)
        w = latency_warning(PORT)
        if w:
            print("* " + w)
        self.pkt = PacketHandler(2.0)
        missing = [i for i in self.ids
                   if all(self.pkt.ping(self.port, i)[1] != COMM_SUCCESS for _ in range(3))]   # 偶发掉包重试 3 次
        if missing:
            self.close()
            sys.exit(f"这些 ID 没应答: {missing}。先用 python tools/dxl_setup.py show 看看总线上有哪些电机")
        self.reader = GroupSyncRead(self.port, self.pkt, ADDR_PRESENT_POSITION, 4)
        self.state_reader = GroupSyncRead(self.port, self.pkt, ADDR_PRESENT_CURRENT, 10)
        self.writer = GroupSyncWrite(self.port, self.pkt, ADDR_GOAL_CURRENT, 2)
        for i in self.ids:
            self.reader.addParam(i)
            self.state_reader.addParam(i)

    def model(self, dxl_id):
        v, res, _ = self.pkt.read2ByteTxRx(self.port, dxl_id, ADDR_MODEL)
        if res != COMM_SUCCESS:
            raise IOError(f"读 ID {dxl_id} 型号失败")
        return v

    def to_signed16(self, v):
        return v - (1 << 16) if v >= (1 << 15) else v

    def enter_current_mode(self, limit_ma, watchdog_ms=100):
        """扭矩关 -> 电流模式 -> 写电流上限(EEPROM) -> 目标电流 0 -> 扭矩开 -> 打开总线看门狗"""
        for i in self.ids:
            self.pkt.write1ByteTxRx(self.port, i, ADDR_TORQUE, 0)
            self.pkt.write1ByteTxRx(self.port, i, ADDR_BUS_WATCHDOG, 0)
            self._check(self.pkt.write1ByteTxRx(self.port, i, ADDR_MODE, 0), i, "设电流模式")
            self._check(self.pkt.write2ByteTxRx(self.port, i, ADDR_CURRENT_LIMIT, int(limit_ma)), i, "写电流上限")
            self._check(self.pkt.write2ByteTxRx(self.port, i, ADDR_GOAL_CURRENT, 0), i, "目标电流清零")
        for i in self.ids:
            self._check(self.pkt.write1ByteTxRx(self.port, i, ADDR_TORQUE, 1), i, "开扭矩")
            # 超过 watchdog_ms 没收到指令，电机自己停（防止程序卡死时电机一直出力）
            self._check(self.pkt.write1ByteTxRx(self.port, i, ADDR_BUS_WATCHDOG, max(1, watchdog_ms // 20)), i, "开看门狗")

    def voltage(self):
        """总线电压 V（读第一个电机的 Present Input Voltage，地址 144）"""
        v, res, _ = self.pkt.read2ByteTxRx(self.port, self.ids[0], 144)
        return v / 10 if res == COMM_SUCCESS else float("nan")

    def hardware_errors(self):
        """{id: [错误名]}，只列出有错误的电机"""
        out = {}
        for i in self.ids:
            e, res, _ = self.pkt.read1ByteTxRx(self.port, i, ADDR_HW_ERROR)
            if res == COMM_SUCCESS and e:
                out[i] = [name for bit, name in HW_ERRORS.items() if e >> bit & 1]
        return out

    def clear_hardware_errors(self, names=None):
        """有硬件错误（过载、过热等保护）的电机软件重启一次清掉；返回 (清掉的, 重启后还有错误的)"""
        errs = self.hardware_errors()
        for i in errs:
            self.pkt.reboot(self.port, i)
        if errs:
            time.sleep(0.6)
        still = self.hardware_errors() if errs else {}
        return errs, still

    def check_watchdog(self, watchdog_ms=100):
        """看门狗触发后 Bus Watchdog 变成 -1，目标值被锁住；清零再重新打开。返回触发过的 ID"""
        tripped = []
        for i in self.ids:
            v, res, _ = self.pkt.read1ByteTxRx(self.port, i, ADDR_BUS_WATCHDOG)
            if res == COMM_SUCCESS and v == 0xFF:
                tripped.append(i)
                self.pkt.write1ByteTxRx(self.port, i, ADDR_BUS_WATCHDOG, 0)
                self.pkt.write2ByteTxRx(self.port, i, ADDR_GOAL_CURRENT, 0)
                self.pkt.write1ByteTxRx(self.port, i, ADDR_BUS_WATCHDOG, max(1, watchdog_ms // 20))
        return tripped

    def enter_position_mode(self, current_limit_ma, watchdog_ms=100):
        """从手：扭矩关 -> 模式 5（电流限制的位置控制）-> 扭矩开 -> 目标电流 = 力的上限 -> 看门狗"""
        for i in self.ids:
            self.pkt.write1ByteTxRx(self.port, i, ADDR_TORQUE, 0)
            self.pkt.write1ByteTxRx(self.port, i, ADDR_BUS_WATCHDOG, 0)
            self._check(self.pkt.write1ByteTxRx(self.port, i, ADDR_MODE, 5), i, "设位置模式")
            self._check(self.pkt.write2ByteTxRx(self.port, i, ADDR_CURRENT_LIMIT, int(current_limit_ma)), i, "写电流上限")
        for i in self.ids:
            self._check(self.pkt.write1ByteTxRx(self.port, i, ADDR_TORQUE, 1), i, "开扭矩")
            self._check(self.pkt.write2ByteTxRx(self.port, i, ADDR_GOAL_CURRENT, int(current_limit_ma)), i, "写目标电流")
            self._check(self.pkt.write1ByteTxRx(self.port, i, ADDR_BUS_WATCHDOG, max(1, watchdog_ms // 20) if watchdog_ms > 0 else 0), i, "开看门狗")   # 0 = 不开
        self.pos_writer = GroupSyncWrite(self.port, self.pkt, ADDR_GOAL_POSITION, 4)

    def set_position_gains(self, p=None, d=None):
        """从手位置环的 P / D 增益（切到模式 5 之后再设，切模式会把它们复位成默认值 P=400, D=0）"""
        for i in self.ids:
            if p is not None:
                self._check(self.pkt.write2ByteTxRx(self.port, i, ADDR_POS_P_GAIN, int(p)), i, "写 P 增益")
            if d is not None:
                self._check(self.pkt.write2ByteTxRx(self.port, i, ADDR_POS_D_GAIN, int(d)), i, "写 D 增益")

    def write_p_gains(self, gains):
        """{id: 位置环 P 增益}，一次 Sync Write；扭矩开着也能写（RAM 区），用于刚度随接触变化"""
        if not hasattr(self, "gain_writer"):
            self.gain_writer = GroupSyncWrite(self.port, self.pkt, ADDR_POS_P_GAIN, 2)
        self.gain_writer.clearParam()
        for i, v in gains.items():
            v = int(max(0, min(16383, round(v))))
            self.gain_writer.addParam(i, [DXL_LOBYTE(v), DXL_HIBYTE(v)])
        self.gain_writer.txPacket()

    def write_positions(self, ticks):
        """{id: 目标位置 tick（可为负 / 多圈）}"""
        self.pos_writer.clearParam()
        for i, v in ticks.items():
            v = int(round(v)) & 0xFFFFFFFF
            self.pos_writer.addParam(i, [v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF, (v >> 24) & 0xFF])
        self.pos_writer.txPacket()

    def _check(self, result, dxl_id, what):
        res, err = result
        if res != COMM_SUCCESS:
            raise IOError(f"ID {dxl_id} {what}失败: {self.pkt.getTxRxResult(res)}")
        if err & 0x7F:                                   # 低 7 位才是指令本身的错误；0x80 只是"有硬件报警"的提示位，指令已执行
            raise IOError(f"ID {dxl_id} {what}报错: {self.pkt.getRxPacketError(err)}")

    def check_torque(self, label=""):
        """开扭矩之后核对：被硬件保护（例如输入电压超限）关掉的电机记到 self.disabled，只读不出力，不让整个程序停"""
        self.disabled = set()
        for i in self.ids:
            v, res, _ = self.pkt.read1ByteTxRx(self.port, i, ADDR_TORQUE)
            e, _, _ = self.pkt.read1ByteTxRx(self.port, i, ADDR_HW_ERROR)
            if res == COMM_SUCCESS and not v:
                self.disabled.add(i)
                errs = [n for b, n in HW_ERRORS.items() if e >> b & 1]
                print(f"!! {label}ID {i} 开不了扭矩（硬件保护: {'、'.join(errs) or '未知'}）：这个关节只读位置、不出力", flush=True)
        return self.disabled

    def read_state(self):
        """{id: (电流 mA, 速度 rad/s, 位置 tick)}"""
        res = self.state_reader.txRxPacket()
        if res != COMM_SUCCESS:
            raise IOError(self.pkt.getTxRxResult(res))
        out = {}
        for i in self.ids:
            cur = self.to_signed16(self.state_reader.getData(i, ADDR_PRESENT_CURRENT, 2))
            vel = to_signed32(self.state_reader.getData(i, ADDR_PRESENT_CURRENT + 2, 4)) * VEL_UNIT
            pos = to_signed32(self.state_reader.getData(i, ADDR_PRESENT_CURRENT + 6, 4))
            out[i] = (cur, vel, pos)
        return out

    def write_currents(self, ma):
        """{id: mA}，一次 Sync Write 发给所有电机（不等回应）"""
        self.writer.clearParam()
        for i, v in ma.items():
            if i in getattr(self, "disabled", ()):
                continue
            v = int(round(v)) & 0xFFFF
            self.writer.addParam(i, [DXL_LOBYTE(v), DXL_HIBYTE(v)])
        self.writer.txPacket()

    def safe_stop(self):
        """目标电流清零、关扭矩、关看门狗、模式改回电流模式（出任何错都要执行到）"""
        for i in self.ids:
            for addr, size, val in ((ADDR_GOAL_CURRENT, 2, 0), (ADDR_TORQUE, 1, 0), (ADDR_BUS_WATCHDOG, 1, 0), (ADDR_MODE, 1, 0)):
                try:
                    (self.pkt.write2ByteTxRx if size == 2 else self.pkt.write1ByteTxRx)(self.port, i, addr, val)
                except Exception:
                    pass

    def torque_off(self):
        for i in self.ids:
            self.pkt.write1ByteTxRx(self.port, i, ADDR_TORQUE, 0)

    def read_raw(self):
        """{id: 有符号编码器值}；一次 Sync Read 读完所有电机"""
        res = self.reader.txRxPacket()
        if res != COMM_SUCCESS:
            raise IOError(self.pkt.getTxRxResult(res))
        out = {}
        for i in self.ids:
            if not self.reader.isAvailable(i, ADDR_PRESENT_POSITION, 4):
                raise IOError(f"ID {i} 没返回数据")
            out[i] = to_signed32(self.reader.getData(i, ADDR_PRESENT_POSITION, 4))
        return out

    def read_avg(self, n=10):
        acc = {i: [] for i in self.ids}
        for _ in range(n):
            for i, v in self.read_raw().items():
                acc[i].append(v)
            time.sleep(0.01)
        return {i: sum(v) / len(v) for i, v in acc.items()}, {i: max(v) - min(v) for i, v in acc.items()}

    def close(self):
        self.port.closePort()


def joint_angles(raw, cal, reg, axes=None):
    """
    原始编码器值 -> {关节名: 模型关节角(rad)}，只包括已标定的关节。
    axes: 当前模型的关节轴（glove_model.joint_axes()）。重新导出后某个关节轴反了，这里会自动修正方向
    （零位变了的话还是要重新标定零点）。
    """
    q = {}
    for name, c in cal["joints"].items():
        i = reg.get(name)
        if i in raw:
            q[name] = effective_sign(c, name, axes) * ticks_to_rad(wrap_ticks(raw[i] - c["zero_raw"]))
    return q


def effective_sign(c, name, axes=None):
    """模型关节角 = 这个符号 × 电机角度（相对零点）"""
    s = c["sign"]
    if axes is not None and "axis_base" in c and name in axes:
        dot = sum(a * b for a, b in zip(c["axis_base"], axes[name]))
        if dot < 0:
            s = -s
    return s
