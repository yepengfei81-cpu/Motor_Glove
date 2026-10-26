# Dynamixel XL330 力反馈遥操作：通信、辨识、控制 使用指南

适用：任何用 DYNAMIXEL XL330（M288 / M077，Protocol 2.0）做主手的力反馈遥操作，包括我们的同构手套（13 电机）和 GELLO 式主臂（6–7 个 M288）驱动 YAM。
写给要把这套东西搬到别的系统上的人。先讲结论和数字，再讲怎么用；原理只讲到够决定参数的程度。
更细的实验记录在 `docs/motor_optimization_plan.md`，给非专业读者的版本是那个网页。

一句话结论：**XL330 做力反馈的瓶颈是 8 ms 的回路延迟（6 ms 位置上报 + 2 ms 出力），它决定了弹簧能有多硬；
能改的是总线速度（183 → 750 Hz）、对电机的了解（摩擦、惯量、反电动势全部量出来）、驱动方式（电压模式让电机自己刹车），
以及从手不再用电机内部的位置环。** 这四样做完，空载不黏、顶住不抖、从手不卡。

---

## 1. 通信：先把每一段时间量出来

### 1.1 两种 USB 桥，差别在哪

| | OpenRB-150（我们的手套）| U2D2（FTDI FT232H）|
|---|---|---|
| 原厂能力 | 桥接固件只能 1 Mbps | 3 Mbps 没问题 |
| 单电机一次读 | 0.6 ms @1 M，**0.36 ms @3 M（自研固件）** | **1.00 ms，任何波特率**（FTDI latency timer 地板） |
| 13 电机模拟读 @3 M | 0.82 ms | 1.0 ms |
| 读+写全速环 | 1.95 kHz | 1.0 kHz |
| 800 Hz 节拍 | 0% 迟到 | 0% 迟到 |

结论：要快，桥必须是 OpenRB 跑我们的固件；U2D2 每次读 1 ms 是硬地板，7 个电机的 GELLO 在 U2D2 上也就 500 Hz 左右。
机械手那条线（U2D2）继续用也行，收益小。

### 1.2 OpenRB-150 刷自研桥接固件

原厂固件逐字节中断搬数据，2 Mbps 起每 7–10 个字节丢 1 个，提中断优先级没用。
`firmware/openrb_bridge/` 用 DMA 接收环形缓冲 + 8 倍过采样，2 M / 3 M 500 包零错，4 M 有 0.2% 坏包，所以**标准定 3 Mbps**。

    arduino-cli compile --fqbn OpenRB-150:samd:OpenRB-150 firmware/openrb_bridge
    arduino-cli upload  --fqbn OpenRB-150:samd:OpenRB-150 -p /dev/ttyACM2 firmware/openrb_bridge

烧录后串口号可能变（`arduino-cli board list`）。刷不进就双击 RESET 进 bootloader 再 upload。刷回原厂的方法在 `firmware/openrb_bridge/README.md`。

串口别按 ttyACMx 的号写死，装 `tools/99-dxl-names.rules`（按产品名建 /dev/openrb、/dev/u2d2），U2D2 再装 `tools/99-dxl-latency.rules`（latency timer 1 ms）。

### 1.3 波特率迁移

    python tools/mg.py A baud 3000000                       # 一台设备的电机全部改到 3 M（EEPROM）
    python tools/bench_bus.py --port /dev/ttyUSB0 --baud 1000000 --set-baud 3000000   # 单个电机

`dxl_bus.Bus()` 打开时按 3 M → 1 M → 2 M → 57600 自动找，所以改完不用动别的代码。
**同一条总线上所有电机必须同一波特率**：拆下来单独测过的电机装回去前改回和总线一致。

### 1.4 一个周期里的协议

- **Fast Sync Read（0x8A）**：一包问全部电机，回包串成一条：ID 字段是 0xFE，每段 `ERR ID DATA CRC`，每段的 CRC 是到该段为止的累计 CRC，末段 CRC 就是整包 CRC。
  `Bus.read_state()` 默认走它，按包头重对齐 + 整包 CRC 校验，连续 3 次失败自动退回普通 Sync Read。
- 读 10 字节（电流 2 + 速度 4 + 位置 4）；写用 Sync Write 不等回应。
- 每个电机 Return Delay 设 0；总线看门狗 100 ms（没指令电机自己停，看门狗触发后 Goal 被锁住，要清零再开，`check_watchdog()` 做这件事）。
- 周期 ≈ 0.82 读 + 0.3 USB 往返 + 0.15 写 ≈ 1.3 ms → 750 Hz（台架实测 740–790 Hz）。13 电机在真手套上估算 800 Hz 量级，还没实测。

### 1.5 PC 侧

- `--rt`：SCHED_FIFO 50 + 绑一个空闲核。需要 `/etc/security/limits.conf` 放开 rtprio。不要关别的进程（ROS2 等），绑核就够了。
- 例行检查（电压、硬件错误、看门狗）拆成每周期一步，一次只做一次快速读，不要在环里一次做完（那会卡 5–20 ms）。
- 两条总线并行读：从手一个常驻线程，主线程同时读主手。
- Python SDK 的开销只有几十微秒，不用重写包层。

### 1.6 `dxl_bus.Bus` 一览

```python
from dxl_bus import Bus
b = Bus([1, 2, 3], port="/dev/openrb")          # 自动找波特率
b.enter_current_mode(limit_ma=800, watchdog_ms=100)   # 电流模式
b.enter_pwm_mode(limit_pct=90)                        # 电压模式（Operating Mode 16）
b.enter_position_mode(current_limit_ma=500); b.set_position_gains(p=600, d=20, i_gain=0)
st = b.read_state()            # {id: (电流 mA, 速度 rad/s, 位置 tick)}，一次 Fast Sync Read
b.write_currents({1: 120})     # mA        b.write_pwms({1: 12.5})   # 占空比 %
b.write_positions({1: 2048})   # tick
b.fast_voltage_temp(); b.fast_hw_errors(); b.check_watchdog(); b.safe_stop()
```

---

## 2. 电机的物理事实（两种型号都量过）

台架：电机固定，输出轴装 18 mm、0.98 g 的杆。M077 两台（T1 OpenRB、T2 U2D2），M288 两台（T3、T4，从手套拆下）。
所有"mA"都是电机电流传感器的读数单位，绝对标定和手册对不上（差 2–3 倍），但自洽，控制里全部用这个单位。

| 量 | XL330-M077 | XL330-M288 | 按减速比 288/77 = 3.74 预期 |
|---|---|---|---|
| 起动电流 | 17–18 mA | 18–21 mA | 相同（起动摩擦在转子侧） |
| 库仑摩擦 tc（反驱） | 3.6 mN·m | 15.8 mN·m（两台 14.7–15.8） | ×3.74 = 13.5 |
| 静摩擦（ramp） | 3.0 mN·m | 16–17.6 mN·m | |
| 惯量 J（输出轴） | 3.2 ×10⁻⁵ kg·m² | 5.6 ×10⁻⁴ kg·m² | ×14 = 4.5e-4 |
| 粘滞 b | 0.04 mN·m·s/rad | 0.05–0.3 mN·m·s/rad | 两者都可忽略 |
| 净 Kt（推负载，传感器单位） | 0.17–0.21 N·m/A | 0.76–1.0 N·m/A | ×3.74 |
| 反电动势 Ke | 0.12 V·s/rad | 0.45 V·s/rad | ×3.74 |
| 6 V 空载转速 | ≈31 rad/s（电流模式实测封顶）| ≈13 rad/s | |
| 负载摩擦（径向力压输出轴套） | 10.9 mN·m/N | ≈9 mN·m/N | 同一种轴套 |
| 反驱时摩擦扛掉的负载 | 77–89% | 73–86% | |
| 电压模式归零刹车 b_em | 1.3–1.8 mN·m·s/rad | 21–26 mN·m·s/rad | ×14 |
| 电流响应 / 位置上报 / 速度上报延迟 | 2 / 6 / 22 ms | 2 / 6 / 20 ms | 同一套电子 |
| 堵转电阻（传感器单位，88% 占空比） | 4.1 Ω | 沿用 4.1 Ω | 同一绕组 |

要记住的几条：

1. **延迟 8 ms 在位置上报里**，电流模式和电压模式一样，换驱动方式减不掉。电机自报的速度再晚 22 ms，不能用，速度要自己从位置算。
2. **负载摩擦是手感的大头**：连杆拉舵盘就是径向负载，1.6 N 时 90% 的反驱力矩被摩擦吃掉。机器人"顶住人"几乎不花电流，"推动人"要全额电流。机械上值得给输出轴加支撑或轴承。
3. **电压模式下驱动桥在 PWM 关断期是断开的**（断续导通）：占空比和电流不成正比，5% 时表观电阻 14 Ω，50% 时 6 Ω，88% 才 4.1 Ω。
   电磁阻尼也因此只有理论值 Kt·Ke/R 的三分之一。控制里力矩→占空比**必须查表**（`config/motor_params/*.json` 的 `pwm.stall_map`，两型号共用）。
4. **同型号两台摩擦差 7–14%**，Kt 和负载摩擦差 <5%。摩擦前馈不能按 100% 给。
5. M288 的惯量是 M077 的 14 倍，手上感觉得到，补不了（加速度前馈在高频就是负阻尼）。要细腻手感的关节用低减速比。

---

## 3. 辨识：怎么把一台新电机量出来

同型号直接用 `config/motor_params/XL330-M288.json` / `XL330-M077.json`，不用重做。
新型号、或者怀疑某台电机不一样，按下面来。全套约 40 分钟，起动摩擦和衰减两条只要 5 分钟。

### 3.1 登记成"单关节设备"

    python tools/bench_bus.py --port /dev/ttyACM2 --baud 1000000 --n 10          # 扫 ID，顺便量通信时间
    python tools/bench_bus.py --port /dev/ttyACM2 --baud 1000000 --set-baud 3000000
    python tools/bench_bus.py --port /dev/ttyACM2 --baud 3000000 --as-device T5 --joint index_mcp_flex_motor

`--as-device` 写 `config/devices.json` 和 `config/calibration_T5.json`（交互标零点和方向）。
`--joint` 给电机现在的 ID 对应的关节名（表在 `config/motors.json`），ID 就不会被改；不给默认 thumb_cmc_abd_motor，ID 会被改成 1。

### 3.2 实验顺序（电机可以放倒，18 mm 轻杆的重力矩 <3% 摩擦）

    python tools/motor_id.py T5 spin  --axis horizontal --pulse-ma 150 --pulse-s 0.4 --decay-s 3   # 甩起来零电流自停 → b/J, tc/J
    python tools/motor_id.py T5 spin  --axis horizontal --pulse-ma 150 --pulse-s 0.4 --decay-s 3 --coast   # 同上但关扭矩 → 电磁阻尼对照
    python tools/motor_id.py T5 ramp  --axis horizontal                       # 电流慢爬到杆动 3° → 起动电流
    python tools/motor_id.py T5 chirp --axis horizontal --amp-ma 150          # 0.3–20 Hz 扫频 30 s → J、电流滞后
    python tools/motor_id.py T5 step  --amp-ma 150                            # 阶跃 → 上报延迟
    python tools/motor_id.py T5 sweep --axis horizontal --mass 0.164 --arm 0.018 --turns 2 --rate 15   # 挂砝码转两圈 → Kt、负载摩擦
    python tools/motor_id.py T5 sweep --axis horizontal --mass 0.328 --arm 0.018 --turns 2 --rate 15   # 重一点再来一次（M288 用重的）
    python tools/motor_id.py T5 fit                                           # 拟合，写 config/motor_params/T5.json
    python tools/motor_id.py T5 spin --mode pwm --pulse-pct 25                # 电压模式归零刹车 → b_em
    python tools/motor_id.py T5 step --mode pwm --amp-pct 15                  # 电压模式阶跃 → Ke
    python tools/motor_id.py T5 step --mode pwm --amp-pct 15 --tag blocked    # 杆用挡块挡死 → 堵转电流 → R

只有两条 sweep 挂砝码（砝码用短线吊在杆端，线竖直，杆会把砝码抬过顶），其余全部空载。
`--mode pwm` 的两条会把 Ke、R 直接打印；把它们和 b_em 写进参数文件的 `pwm` 块（`G_Nm_per_V = Kt/R`、`Ke_V_s_per_rad`、`R_sensor_ohm`、`b_em_Nms_per_rad`），`stall_map` 直接从 XL330-M288.json 抄。

### 3.3 fit 输出怎么核对

- 起动电流、tc/J、净 Kt 和第 2 节的表比，差 10% 以内算同一批。
- 不同减速比之间按 3.74 和 14 倍换算应该对得上（我们两型号五个量都对上了），对不上先怀疑 `--arm`、`--mass` 和轴朝向。
- 同型号多台只做 spin 和 ramp，看摩擦离散，决定前馈留多少余量（见 4.5）。

---

## 4. 控制律：现在跑的是什么、参数怎么给

入口 `tools/mg.py teleop <主手> <从手> <手指组> --preset <预设> [覆盖参数]`，实际执行 `tools/bilateral.py`。预设在 `config/teleop_presets.json`，预设参数在前，命令行在后覆盖。

### 4.1 主手侧（每个关节独立）

| 项 | 公式 | 参数 |
|---|---|---|
| 速度观测 | v̂ = LPF(Δq/Δt)，â = LPF₆(Δv̂/Δt)；一拍跳 ≥1 rad 视为折返 | `--obs-hz 15`（M288 可 25） |
| 摩擦前馈 | i_ff = c·tc/Kt·ramp(\|v̂\|; v_dead, v_ramp)·sgn(v̂) | `--comp-gain 0.8`，`--v-dead 0.05`，`--v-ramp 0.2` |
| 接触判断（模型法） | i_free = (J·â + b·v̂ + tc·tanh(v̂/v₀))/Kt；g = f(\|ī_B − i_free\| − I_c, \|v̂_B\|)，低通 τ_gate | `--contact-ma 150`，`--contact-vel 0.8`，`--gate-tau 0.15` |
| 反馈力 | τ = −K·(\|e\| − δ)·sgn(e)·g − B·v̂·g，B = 2ζ√(K·J)；单向（往被挡住那侧压才出力） | `--kp 0.8`，`--deadband 2`，`--zeta 0.7`，`--max-torque 0.25`，`--leader-limit 600` |
| 推人效率补偿 | τ ← τ·(1 + φ·drive·g)，φ = min(0.5, k_load/r_load) | `--r-load 0.025`，`--r-load-map NAME=m`（每关节舵盘半径） |
| 电压模式 | duty = map(i_cmd) + (k·Ke·v̂ + u_I)/V，k = cancel·(1 − g)；u_I 慢电流环；限流 \|i\| ≤ leader-limit | `--mode pwm`，`--pwm-cancel 0.9`，`--pwm-iloop 5`，`--pwm-limit 90` |

电压模式是默认推荐：空载时反电动势被抵消，和电流模式一样轻；一判断到接触，抵消停掉，电机自己的电磁阻尼全额、零延迟地作用在主手上，松手不荡。

### 4.2 从手侧

- `--follower-mode position`（旧）：电机内部位置环，`--follower-kp/kd/ki`，电流上限 `--follower-limit`。M288 在这个模式下慢速跟随一格一格跳（P 600 要 1.2° 误差才顶过静摩擦），加 I 会在目标附近来回蹭。
- `--follower-mode current`（新，推荐）：电流模式，我们自己做位置环加全部模型前馈：
  τ = K(q_ref − q) + B(v_ref − v̂) + J·a_ref + b·v_ref + c·tc·sgn(v_ref)，K = J·(2πf)²，B = 2ζ√(KJ)。
  `--follower-bw 6`（M288），`--follower-bw-map M077=12`（小惯量要更高带宽才有抓握刚度），`--follower-zeta 0.8`，`--follower-comp 0.8`。
  前馈沿参考轨迹，不是反馈，不会变负阻尼。台架实测从手不再卡顿。

### 4.3 力反射（摸软东西靠它）

位置差弹簧只能渲染"从手过不去"，软物体和慢起伏要把从手的接触力直接反射回来：
f̂ = LPF(τ_从手指令) − J·LPF(â_B) − LPF(b·v̂_B + c·tc·sgn(v̂_B))，三项**同一个低通**（`--fb-force-lpf 6`），
τ_主手 −= `--fb-force` × (|f̂| − 死区)·sgn(f̂)。从手是 current 模式时 τ_指令 是我们算的，干净；position 模式退回用实测电流。
滤波不配平的版本（指令瞬时、模型滞后）会在主手一加速时产生假接触力，两边一起抖，别改回去。
上限：8 ms 上报滞后决定 20 Hz 以上的纹理反射不出来；M288 自身摩擦 16 mN·m 以下的接触力淹在模型误差里。

### 4.4 预设

| 预设 | 用途 |
|---|---|
| `demo_old` / `demo_old_288` | 原始方案对照（100 Hz、无模型、双向弹簧、低通 4–5 Hz），故意调差 |
| `bench_free` | M077 台架电流模式，用户认可的空载手感 |
| `bench_pwm` / `demo_new` | M077 台架电压模式 |
| `demo_new_288` | M288 台架：电压模式 + 前馈 0.9 + 抵消 0.9 + 死区 0.05 + 观测器 25 Hz；加 `--follower-mode current --follower-limit 300` 就是目前最好的配置 |
| `glove_pwm` | 手套 A → 机械手 H：`--model auto` 按型号选参数、电压模式、从手 current、M077 关节 12 Hz、前馈 0.8 |

### 4.5 哪些能调、哪些别碰

- 能调：`--comp-gain`（0.7–0.9，每台电机的摩擦离散决定上限，超过真实摩擦关节会自己蠕动）、`--pwm-cancel`（0.5–0.9，Ke 可靠就往上）、`--kp`（0.5–1.0）、`--leader-limit`、`--follower-bw`、`--fb-force`（0 起，0.3 一档）。
- 别碰：观测器的 6 Hz 加速度低通、门控的速度条件、堵转表、电流环上限、力反射三项同带宽。这些每一项都是一次振荡换来的。
- 换电机：改参数文件，不改代码。`--model auto` 按总线报的型号选文件；一个型号一个文件。

---

## 5. 几条教训，省得再踩

1. **弹簧能多硬由延迟定**：K·T/2 < b（Colgate）。T = 8 ms、电机自身阻尼 0.45 mN·m·s/rad 时 K 只能 0.11 N·m/rad；K 0.8 是靠接触门控和手指自己的阻尼活下来的，手一松就荡。电压模式把 b 提到 2–20 倍，门限才宽裕。
2. **软件阻尼救不了延迟弹簧**：B·ė 经过同一个 8 ms 和 15 Hz 观测器，在 25 Hz 处相位 −131°，它自己就是负阻尼。要物理的阻尼。
3. **反电动势前馈不能全抵消**：Ke 估高 10% 就是负阻尼，电机自己跑。抵消 0.5–0.9，剩下交给慢电流环。
4. **电压模式必须查表**：按一个线性 G 换算时 13 mA 的前馈只给了 0.8% 占空比，接触力只送到四分之一，前馈靠电流环 200 ms 才追上，手上就是"没补偿"。
5. **松手那 9 ms 是硬物理**：Δv = τ·T/J，松手后电机还要出力一个回路延迟。电压模式只把之后的滑行从 ~100° 缩到 ~25°。想再小：用电流（2 ms 就反映速度）做松手判据。
6. **从手别用 I 增益对付摩擦**，用模型前馈。
7. **摩擦前馈留余量**：同型号摩擦差 14%，前馈 0.8 起步，装上以后按关节做 ramp 再放开。
8. 电机供电 ≤ 6 V（XL330 规格 3.7–6.0 V）；看门狗 100 ms 没喂就把 Goal 锁死，中间有 sleep 的脚本要喂狗（`motor_id.Motor.pause`）。

---

## 6. 搬到 YAM / GELLO 上的清单

GELLO 主臂 6–7 个 M288，YAM 是从臂。能直接用的是**通信 + 主手侧全部**；从手侧要看 YAM 给什么接口。

1. **桥**：GELLO 若接 U2D2，每周期 1 ms 地板，7 电机约 500 Hz，够用；要更快换 OpenRB-150 刷 `firmware/openrb_bridge`。
2. **登记**：`config/motors.json` 填关节名 ↔ ID；`config/devices.json` 加一台设备（port、model）；`tools/calibrate.py` 或 `bench_bus.py --as-device` 标零点和方向。
   bilateral 里"手指组"只是关节名的分组，GELLO 可以把 7 个关节放一组。
3. **参数**：全部 M288 → `--model auto` 直接拿 `XL330-M288.json`，什么都不用量。想稳妥：每个关节装着做一次 ramp（起动电流），和 21 mA 比，决定 `--comp-gain`。
4. **舵盘半径**：GELLO 没有连杆，`--r-load` 填手握住的位置到轴的距离（推人效率补偿用），不准也只影响接触力大小两三成。
5. **主手侧直接用**：`--mode pwm --pwm-cancel 0.9 --comp-gain 0.8 --obs-hz 25 --v-dead 0.05 --v-ramp 0.2 --kp 0.8 --zeta 0.7`，先 `--follower-port none --lock` 用虚拟墙测力反馈本身。
6. **从手是 YAM 时**：bilateral 现在假设两边都是 Dynamixel 总线。需要一个薄适配：每周期给它 YAM 的关节位置（算 e = q_主 − q_从）和 YAM 的关节力矩估计（当 f̂ 用，走 `--fb-force` 那条路），把它的从手指令换成发给 YAM 的目标。
   主手这边的观测器、门控、弹簧、电压模式、限流都在 `bilateral.py` 里按关节写的，不依赖从手是谁。
7. **先量频率再调手感**：`--hz 800` 跑起来看屏幕第一行的实际 Hz。低于 500 先查总线（波特率、Return Delay、latency timer），不要先调增益。

---

## 7. 文件地图

| 路径 | 作用 |
|---|---|
| `firmware/openrb_bridge/` | OpenRB-150 DMA 桥接固件（3 Mbps） |
| `tools/dxl_bus.py` | 总线层：自动波特率、Fast Sync Read、电流/电压/位置模式、看门狗、快速例行读 |
| `tools/bench_bus.py` | 通信基准、改波特率/ID、登记单关节台架设备 |
| `tools/motor_id.py` | 辨识实验 spin/ramp/chirp/step/sweep（+ `--mode pwm`）、fit |
| `tools/bilateral.py` | 双边遥操作：观测器、门控、弹簧阻尼、摩擦前馈、电压模式、从手软件环、力反射、日志 |
| `tools/mg.py` | 统一入口和预设 |
| `tools/compare_logs.py` | 两份日志对比（`--overlay` 新旧叠图） |
| `config/motor_params/XL330-M288.json`, `XL330-M077.json` | 按型号的参数（J、b、tc、Kt、负载摩擦、pwm 块含堵转表）|
| `config/motor_params/T1..T4.json` | 台架单台电机的原始辨识结果 |
| `config/teleop_presets.json` | 预设 |
| `config/motors.json`, `devices.json`, `calibration_*.json` | ID 表、设备表、零点方向 |
| `data/motor_id/` | 辨识原始数据 |
| `docs/motor_optimization_plan.md` | 过程记录和每一步的结论 |

版本：tag `bench-m288-v1`（2026-10-25）。
