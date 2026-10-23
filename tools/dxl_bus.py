"""
DYNAMIXEL 总线的最小封装：1 Mbps，Sync Read 读位置，标定换算成模型关节角。
calibrate.py 和 glove_live.py 共用。
"""
import json
import math
import os
import select
import struct
import sys
import time

from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncRead, GroupSyncWrite, COMM_SUCCESS, DXL_LOBYTE, DXL_HIBYTE


def default_port():
    """DXL_PORT 环境变量优先；否则有 U2D2(/dev/ttyUSB*) 就用它，再退回 OpenRB(/dev/ttyACM0)"""
    import glob
    if os.environ.get("DXL_PORT"):
        return os.environ["DXL_PORT"]
    usb = sorted(glob.glob("/dev/ttyUSB*"))
    if usb:
        return usb[0]
    from devices import find_tty
    return find_tty("OpenRB-150") or "/dev/ttyACM0"


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
BAUD_CANDIDATES = [3000000, 1000000, 2000000, 57600]     # 打开总线时按这个顺序找电机（3 M 是新标准，1 M 是旧标准）
BAUD_CODE = {9600: 0, 57600: 1, 115200: 2, 1000000: 3, 2000000: 4, 3000000: 5, 4000000: 6}
ADDR_TORQUE, ADDR_PRESENT_POSITION = 64, 132
ADDR_MODEL, ADDR_MODE, ADDR_CURRENT_LIMIT, ADDR_BUS_WATCHDOG, ADDR_GOAL_CURRENT = 0, 11, 38, 98, 102
ADDR_HW_ERROR = 70
ADDR_GOAL_POSITION = 116
ADDR_PWM_LIMIT, ADDR_GOAL_PWM, PWM_FULL = 36, 100, 885   # PWM 模式（Operating Mode 16）：Goal PWM ±885 = ±100% 占空比
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


# ---- Protocol 2.0 CRC 和 Fast Sync Read（0x8A）：所有电机的数据打在一个状态包里回来，省掉每个电机的包头 ----
_CRC_TABLE = []
for _i in range(256):
    _c = _i << 8
    for _ in range(8):
        _c = ((_c << 1) ^ 0x8005) & 0xFFFF if _c & 0x8000 else (_c << 1) & 0xFFFF
    _CRC_TABLE.append(_c)


def crc16(data):
    c = 0
    for b in data:
        c = ((c << 8) ^ _CRC_TABLE[((c >> 8) ^ b) & 0xFF]) & 0xFFFF
    return c


def make_packet(dxl_id, inst, params=b""):
    body = bytes([0xFF, 0xFF, 0xFD, 0x00, dxl_id]) + struct.pack("<H", len(params) + 3) + bytes([inst]) + params
    return body + struct.pack("<H", crc16(body))


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
    """ping 一遍，返回有应答的 ID（不像 Bus() 那样缺一个就退出）；按 BAUD_CANDIDATES 的顺序找，用第一个有应答的波特率"""
    ph = PortHandler(port or PORT)
    if not ph.openPort():
        sys.exit(f"打不开 {port or PORT}（板子和电池都接上了吗？）")
    pkt = PacketHandler(2.0)
    ok = []
    for b in BAUD_CANDIDATES:
        ph.setBaudRate(b)
        ok = [i for i in ids if any(pkt.ping(ph, i)[1] == COMM_SUCCESS for _ in range(2))]
        if ok:
            break
    ph.closePort()
    return ok


class Bus:
    def __init__(self, ids, port=None, baud=None, fast=True):
        """
        baud: 指定波特率；None = 按 BAUD_CANDIDATES 顺序试，用全部电机都应答的那个（环境变量 DXL_BAUD 优先）。
        fast: 读状态用 Fast Sync Read（0x8A，一个包回所有电机）；固件不支持或连续出错会自动退回普通 Sync Read。
        """
        self.ids = list(ids)
        self.portname = port or PORT
        self.port = PortHandler(self.portname)
        try:
            ok = self.port.openPort()
        except Exception:
            ok = False
        if not ok:
            sys.exit(f"打不开 {self.portname}（板子和电池都接上了吗？）")
        w = latency_warning(self.portname)
        if w:
            print("* " + w)
        self.pkt = PacketHandler(2.0)
        candidates = [int(baud or os.environ.get("DXL_BAUD") or 0)] if (baud or os.environ.get("DXL_BAUD")) else BAUD_CANDIDATES
        self.baud, missing = None, list(self.ids)
        for b in candidates:
            self.port.setBaudRate(b)
            missing = [i for i in self.ids if all(self.pkt.ping(self.port, i)[1] != COMM_SUCCESS for _ in range(2))]
            if not missing:
                self.baud = b
                break
        if missing:
            self.close()
            sys.exit(f"这些 ID 没应答: {missing}（试过波特率 {candidates}）。先用 python tools/dxl_setup.py show 看看总线上有哪些电机")
        if self.baud != BAUD_CANDIDATES[0]:
            print(f"* {self.portname} 电机在 {self.baud} bps（新标准是 {BAUD_CANDIDATES[0]}，用 mg.py <设备> baud {BAUD_CANDIDATES[0]} 改）")
        self.fast = fast
        self.fast_fail = 0
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

    def enter_pwm_mode(self, limit_pct, watchdog_ms=100):
        """扭矩关 -> PWM(电压)模式(16) -> 写 PWM 上限(EEPROM，占空比 %) -> 目标 PWM 0 -> 扭矩开 -> 看门狗。
        电压模式下电机自己的电流环不再抵消反电动势：绕组被电压源驱动，Kt²/R 的电磁阻尼是物理的、无延迟的"""
        lim = int(max(1, min(100, limit_pct)) / 100 * PWM_FULL)
        for i in self.ids:
            self.pkt.write1ByteTxRx(self.port, i, ADDR_TORQUE, 0)
            self.pkt.write1ByteTxRx(self.port, i, ADDR_BUS_WATCHDOG, 0)
            self._check(self.pkt.write1ByteTxRx(self.port, i, ADDR_MODE, 16), i, "设 PWM 模式")
            self._check(self.pkt.write2ByteTxRx(self.port, i, ADDR_PWM_LIMIT, lim), i, "写 PWM 上限")
            self._check(self.pkt.write2ByteTxRx(self.port, i, ADDR_GOAL_PWM, 0), i, "目标 PWM 清零")
        self.pwm_writer = GroupSyncWrite(self.port, self.pkt, ADDR_GOAL_PWM, 2)
        for i in self.ids:
            self._check(self.pkt.write1ByteTxRx(self.port, i, ADDR_TORQUE, 1), i, "开扭矩")
            self._check(self.pkt.write1ByteTxRx(self.port, i, ADDR_BUS_WATCHDOG, max(1, watchdog_ms // 20)), i, "开看门狗")

    def write_pwms(self, pct):
        """{id: 占空比 %（-100..100）}，一次 Sync Write；超过 PWM 上限的值电机自己截断"""
        self.pwm_writer.clearParam()
        for i, v in pct.items():
            if i in getattr(self, "disabled", ()):
                continue
            v = int(round(v / 100 * PWM_FULL)) & 0xFFFF
            self.pwm_writer.addParam(i, [DXL_LOBYTE(v), DXL_HIBYTE(v)])
        self.pwm_writer.txPacket()

    def ensure_return_delay(self, label=""):
        """Return Delay Time（地址 9，EEPROM）不为 0 的电机写成 0：每个电机 250 µs 的应答延迟，13 个就是 3 ms。扭矩关着时调用"""
        fixed = []
        for i in self.ids:
            v, res, _ = self.pkt.read1ByteTxRx(self.port, i, 9)
            if res == COMM_SUCCESS and v != 0:
                self.pkt.write1ByteTxRx(self.port, i, 9, 0)
                fixed.append(i)
        if fixed:
            print(f"* {label}ID {fixed} 的 Return Delay 不是 0，已改成 0（通信更快）")
        return fixed

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
        """看门狗触发后 Bus Watchdog 变成 -1，目标值被锁住；清零再重新打开。返回触发过的 ID。有 fast 就一次读完所有电机"""
        tripped = []
        vals = None
        if self.fast:
            try:
                vals = {i: d[0] for i, d in self._fast_read(ADDR_BUS_WATCHDOG, 1).items()}
            except IOError:
                vals = None
        for i in self.ids:
            if vals is not None:
                v, res = vals[i], COMM_SUCCESS
            else:
                v, res, _ = self.pkt.read1ByteTxRx(self.port, i, ADDR_BUS_WATCHDOG)
            if res == COMM_SUCCESS and v == 0xFF:
                tripped.append(i)
                self.pkt.write1ByteTxRx(self.port, i, ADDR_BUS_WATCHDOG, 0)
                self.pkt.write2ByteTxRx(self.port, i, ADDR_GOAL_CURRENT, 0)
                self.pkt.write2ByteTxRx(self.port, i, ADDR_GOAL_PWM, 0)
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

    # ---- Fast Sync Read ----
    def _fast_read(self, addr, length, timeout=0.006):
        """
        一次 0x8A 读所有电机 addr 起 length 字节。返回 {id: bytes}。
        回包格式（XL330 实测）：FF FF FD 00 | FE | LEN(2) | 55 | 每电机 [ERR ID DATA CRC(2)]，每段的 CRC 是到该段为止的累计 CRC。
        收包按包头重新对齐（偶尔会有上一包迟到的尾巴），整包 CRC 校验。
        """
        ser = self.port.ser
        ser.reset_input_buffer()
        ser.write(make_packet(0xFE, 0x8A, struct.pack("<HH", addr, length) + bytes(self.ids)))
        seg = length + 4
        expect = 8 + len(self.ids) * seg
        buf = b""
        deadline = time.perf_counter() + timeout
        while True:
            # 按包头对齐
            k = buf.find(b"\xff\xff\xfd\x00")
            if k > 0:
                buf = buf[k:]
            elif k < 0 and len(buf) > 3:
                buf = buf[-3:]
            if len(buf) >= expect:
                break
            remaining = deadline - time.perf_counter()
            if remaining <= 0:
                raise IOError(f"Fast Sync Read 超时：期望 {expect} 字节，收到 {len(buf)}")
            select.select([ser.fd], [], [], remaining)
            chunk = ser.read(expect - len(buf))
            if chunk:
                buf += chunk
        pkt = buf[:expect]
        if pkt[7] != 0x55:
            raise IOError("Fast Sync Read 回包不是状态包")
        if pkt[8] & 0x7F == 0x02:
            raise IOError("固件不支持 Fast Sync Read")
        if crc16(pkt[:-2]) != struct.unpack("<H", pkt[-2:])[0]:
            raise IOError("Fast Sync Read CRC 错")
        out = {}
        for n in range(len(self.ids)):
            k = 8 + n * seg
            err, i = pkt[k], pkt[k + 1]
            if err & 0x7F:
                raise IOError(f"ID {i} 回了错误 {err & 0x7F}")
            out[i] = pkt[k + 2: k + 2 + length]
        if set(out) != set(self.ids):
            raise IOError(f"Fast Sync Read 回来的 ID {sorted(out)} 和要的 {self.ids} 不一样")
        return out

    def fast_voltage_temp(self):
        """{id: (电压 V, 温度 °C)}，一次 Fast Sync Read（144–146）；fast 不可用返回 None"""
        if not self.fast:
            return None
        try:
            vt = self._fast_read(144, 3)
        except IOError:
            return None
        return {i: (struct.unpack("<H", vt[i][:2])[0] / 10.0, vt[i][2]) for i in self.ids}

    def fast_hw_errors(self):
        """{id: [错误名]}，只含有错误的电机；一次 Fast Sync Read（70）；fast 不可用返回 None"""
        if not self.fast:
            return None
        try:
            hw = self._fast_read(ADDR_HW_ERROR, 1)
        except IOError:
            return None
        return {i: [n for b, n in HW_ERRORS.items() if d[0] >> b & 1] for i, d in hw.items() if d[0] & 0x7F}

    def fast_status(self):
        """{id: (电压 V, 温度 °C, 硬件错误位)}；fast 不可用返回 None"""
        vt = self.fast_voltage_temp()
        if vt is None:
            return None
        try:
            hw = self._fast_read(ADDR_HW_ERROR, 1)
        except IOError:
            return None
        return {i: (vt[i][0], vt[i][1], hw[i][0]) for i in self.ids}

    def _fast_or_fallback(self, addr, length):
        """fast 可用就用，连续失败 3 次就退回 SDK 的 Sync Read（返回 None 表示要走旧路）"""
        if not self.fast:
            return None
        try:
            data = self._fast_read(addr, length)
            self.fast_fail = 0
            return data
        except IOError as e:
            self.fast_fail += 1
            if self.fast_fail >= 3:
                self.fast = False
                print(f"!! {self.portname} Fast Sync Read 连续失败（{e}），退回普通 Sync Read", flush=True)
            return None

    def read_state(self):
        """{id: (电流 mA, 速度 rad/s, 位置 tick)}"""
        data = self._fast_or_fallback(ADDR_PRESENT_CURRENT, 10)
        if data is not None:
            out = {}
            for i, d in data.items():
                cur, vel, pos = struct.unpack("<hii", d)
                out[i] = (cur, vel * VEL_UNIT, pos)
            return out
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
            for addr, size, val in ((ADDR_GOAL_CURRENT, 2, 0), (ADDR_GOAL_PWM, 2, 0), (ADDR_TORQUE, 1, 0), (ADDR_BUS_WATCHDOG, 1, 0), (ADDR_MODE, 1, 0)):
                try:
                    (self.pkt.write2ByteTxRx if size == 2 else self.pkt.write1ByteTxRx)(self.port, i, addr, val)
                except Exception:
                    pass

    def torque_off(self):
        for i in self.ids:
            self.pkt.write1ByteTxRx(self.port, i, ADDR_TORQUE, 0)

    def set_baud_all(self, new_baud):
        """把这条总线上的电机全部改到 new_baud（EEPROM 地址 8），然后切过去 ping 确认。返回切换后没应答的 ID"""
        code = BAUD_CODE[int(new_baud)]
        self.torque_off()
        for i in self.ids:
            self.pkt.write1ByteTxRx(self.port, i, 9, 0)                    # Return Delay 也清零
            res, err = self.pkt.write1ByteTxRx(self.port, i, 8, code)
            if res != COMM_SUCCESS or err & 0x7F:
                print(f"!! ID {i} 写波特率失败: {self.pkt.getTxRxResult(res)}")
        time.sleep(0.3)
        self.port.setBaudRate(int(new_baud))
        self.baud = int(new_baud)
        return [i for i in self.ids if all(self.pkt.ping(self.port, i)[1] != COMM_SUCCESS for _ in range(3))]

    def read_raw(self):
        """{id: 有符号编码器值}；一次 Sync Read 读完所有电机"""
        data = self._fast_or_fallback(ADDR_PRESENT_POSITION, 4)
        if data is not None:
            return {i: struct.unpack("<i", d)[0] for i, d in data.items()}
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
