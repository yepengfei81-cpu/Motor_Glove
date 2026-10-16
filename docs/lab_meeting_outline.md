# 组会汇报大纲（2026-10-15）/ Lab-meeting outline

**最终 12 页顺序**（PPT 里的页码）：1 标题 · 2 state 不是 action + 打分卡 · **3 数据手套四十年时间线（新）** · 4 UMI · 5 DexUMI / ART-Glove · 6 DITTO / DITTO-X · 7 我们的定位 + 对比表（含 MILE） · **8 为什么手套难做（新）** · 9 设计 · 10 进展与演示 · 11 力与阻抗 · 12 局限与下一步。下面各节的 P 编号是旧的 10 页编号，对应关系：旧 P3–P6 → 新 4–7，旧 P7–P10 → 新 9–12。

10 页，15 分钟。每页给三块：**Slide（英文，页面上的文字）**、**讲法（中文）**、**数字说明（每个数字从哪来、什么意思、被问到怎么答）**。末尾是可能被问的问题。

标注约定：〔论文〕= 引自该论文原文；〔实测〕= 我们自己测的；〔估计〕= 我们的推算，说的时候要带"大约"。

---

## P1 · Title

**Slide**
- An Isomorphic Force-Feedback Glove–Hand Pair for Dexterous Teleoperation
- 13 motor joints, joint-to-joint bilateral control, and what it lets us record for policy learning
- Pengfei Ye · HKUST / MIT CSAIL · ongoing work with E. Adelson

**讲法**
一句话：一只按人手关节设计的外骨骼手套，和一只结构完全相同的"手"，关节一一对应、带力反馈；今天讲三件事——为什么要机器人在环、我们做到了哪一步、记录下来的力和刚度怎么变成策略的监督信号。

**数字说明**
| 数字 | 含义 | 来源 |
|---|---|---|
| 13 | 电机驱动的主动关节数：拇指 4 + 食指 3 + 中指 3 + 无名指 3，没有小指 | 我们的设计 |

---

## P2 · Handheld devices record what the human did, not what the robot must do

**Slide**
- UMI-style collection has no motor in the loop: gripper width stops at contact, grasp force is never recorded
- A policy needs robot actions: force, compliance, actuator dynamics, feasibility
- DITTO: handheld-only data → 0 % at "hold and lift"; +10 teleop episodes → 70 %
- ForceMimic: force in the action → +54.5 % relative over vision-only
- ACP (UMI's own group): compliance is ignored by position-only visuomotor policies
- Four requirements for good dexterous data: ① dexterous ② no embodiment gap ③ operator feels contact ④ robot in the loop（打分卡）

**讲法**
这是全片的论点。UMI 这类手持采集记录的是夹爪位姿和开度，碰到物体后开度不再变，人捏了多大力完全没记下来，因为采集时根本没有电机。策略部署时需要的是机器人本体的动作：力、柔顺、执行器动力学、可行性，这些只有机器人在环时才会被记录。然后引出四条要求，后面每篇论文都用这张卡打分。

**数字说明**
| 数字 | 含义 | 来源 |
|---|---|---|
| 0 % → 70 % | DITTO 做的对比：只用手持（无机器人）数据训练的策略，在"抓住并抬起"阶段成功率 0；只加 10 条遥操作数据（机器人在环）就到 70 %。作者解释：施力的是人不是机器人执行器，手持演示不包含执行器动力学 | 〔论文〕DITTO, arXiv 2609.19196, Sec. VI-B |
| +54.5 % | ForceMimic：把力记录下来并放进动作后，任务成功率相对纯视觉策略提升 54.5 %。是**相对**提升，不是绝对百分点 | 〔论文〕ForceMimic, arXiv 2410.07554, ICRA 2025 |
| 13 × 220 Hz | 我们每个周期记录 13 个关节的 q*、q、τ、I、K；220 Hz 是仿真在环时单总线读写 13 个电机的实测循环频率；双手套两条总线时约 200 Hz | 〔实测〕 |
| ①–④ | 四条要求是我们自己的归纳，不是引用 | 我们 |

---

## 新 P3 · Forty years of data gloves

**Slide**：上轨 teleoperation（机器人在环）、下轨 demonstration capture（没有机器人），四个年代，颜色 = 手套带了什么（运动传感 / 触觉皮肤 / 力或触觉反馈 / 按某只机械手设计的外骨骼 / 我们）。

**讲法**：只讲一句：两条轨到 2025–2026 在外骨骼上汇合了。MILE 和 DITTO 已经做了同构主从对，MILE 没有电机、DITTO 只有两根手指，我们在它们中间：四指、每个关节有电机。

**数字说明**：条目和年份都核实过（见文末表格），GEX 的 "11-DoF hand + 12-DoF exo" 来自 GEX 论文（arXiv 2506.04982）；ART-Glove 22 DoF / 2048 taxels 见 P4；DITTO 7 DoF 见 P5。年代框只是分组，不是精确年份。

---

## 新 P8 · Why a glove is hard to wear

**Slide**：四个约束：joint axes are hidden（MCP 轴在皮肤下）、the thumb sweeps wide（拇指要 4 个关节）、no room between fingers（三个模块并排不能互碰）、hands differ in size（DexEXO 按 140–217 mm 手长设计；我们只有一个尺寸）。图全部是我们自己的 CAD。

**讲法**：每一条都对应下一页的一个设计决定：双平行四边形虚拟转轴、拇指 4 电机、模块化手指、尺寸是局限。

**数字说明**：140–217 mm 来自 DexEXO（arXiv 2603.17323）摘要；其余没有数字。

---

## P3 · UMI: make the collection device identical to the robot's end-effector

**Slide**
- Identical soft gripper on the handheld device and the robot
- Wrist GoPro, 155° fisheye + side mirrors: observation aligned with action
- Latency matching + relative-trajectory actions → cross-robot deployment
- Limits: parallel gripper only; state only; kinematic feasibility unknown at collection time
- $73 + $298 · 3× · 71.7 %

**讲法**
UMI 的贡献是让无本体数据能直接训练出可部署的策略，不是"开创了无机器人采集"（2020 年就有 Grasping in the Wild）。做法是让采集设备和机器人末端长得一样。局限：平行夹爪；只采 state；采集时不知道机器人运动学是否可行，只能事后过滤。过渡：夹爪做不了灵巧操作，换成手会怎样？

**数字说明**
| 数字 | 含义 | 来源 |
|---|---|---|
| $73 | 手持夹爪的 3D 打印件成本 | 〔论文〕UMI, arXiv 2402.10329 |
| $298 | GoPro 套件（相机、鱼眼镜头、镜子、支架）成本 | 〔论文〕UMI |
| 155° | 鱼眼镜头视场角 | 〔论文〕UMI |
| 3× | 用 UMI 采数据的速度约为 SpaceMouse 遥操作的 3 倍（单位时间的演示条数） | 〔论文〕UMI |
| 71.7 % | 野外"摆杯子"任务在没见过的环境里的成功率；训练数据来自 30 个地点、1400 条演示 | 〔论文〕UMI |

---

## P4 · DexUMI fits the exoskeleton to a robot hand; ART-Glove fits it to the human hand

**Slide**
- DexUMI (robot-first): hardware adaptation (exoskeleton optimized to the robot's fingertip workspace, joint encoders) + software adaptation (segment the human hand, inpaint, render the robot hand back); 86 % average success on Inspire (6 active) and XHand (12 active); authors' limits: per-hand tuning, human force > robot force so tactile drifts, fixed camera
- ART-Glove (human-first): 16 rigid plates, 22 encoder-measured joints, 2048 taxels, 120 Hz; Kapandji 10/10; measurement only, no robot, no policy; future work: an actuated hand built on the glove geometry

**讲法**
DexUMI 不是效果差，是"机器人优先"：外骨骼迁就一只现成的商用手，而且人感到的力和机器人感到的力不是一回事。ART-Glove 是"人手优先"但没有机器人。两者都是人在承受接触，机器人不在环。ART-Glove 作者未来工作里要做"按手套几何的驱动手"，正是我们的路线。

**数字说明**
| 数字 | 含义 | 来源 |
|---|---|---|
| 86 % | DexUMI 各任务、两种手的平均成功率 | 〔论文〕DexUMI, arXiv 2505.21864 |
| Inspire 6 active / 12 joints | Inspire 手 12 个关节、6 个主动驱动，其余联动 | 〔论文〕DexUMI 引用的手规格 |
| XHand 12 active | XHand 12 个主动自由度 | 同上 |
| 16 | ART-Glove 的刚性面板数（手指、手掌各段） | 〔论文〕ART-Glove, arXiv 2606.16370 |
| 22 | 编码器测量的关节数（对齐解剖关节） | 同上 |
| 2048 | 触觉点（taxel）总数 | 同上 |
| 120 Hz | 关节和触觉的同步采样率 | 同上 |
| Kapandji 10/10 | 拇指对掌能力的临床评分，满分 10，说明戴着手套手几乎不受限 | 同上 |

---

## P5 · DITTO closes the loop with an equivalent leader; DITTO-X trades isomorphism for generality

**Slide**
- DITTO (2026-09): leader and follower kinematically equivalent, thumb 4 + index 3 = 7 DoF, 1:1 motor mapping; joint-level force feedback from follower current at 200 Hz; with feedback users apply about half the contact force (6.9 N vs 13.0 N)
- DITTO-X (2026-10): one exoskeleton drives Sharpa Wave, Wuji 2, Inspire; index and middle 1:1, thumb retargeted in task space (Jacobian pseudo-inverse force), ring and pinky coupled; force + fingertip vibration complementary: 86.1 % vs 61.1 %; reverse teleoperation takeover 78.3 % vs 27.7 %; DAgger improves over two rounds

**讲法**
这是离我们最近的工作，老师读过。DITTO 第一次把 ②③④ 放在一起，但只有 7 个自由度；DITTO-X 为了一个外骨骼驱动三种手放弃了同构。我们的差异要讲成：同构做到 4 指 13 自由度、拇指完整 4 关节、主从一起设计。

**数字说明**
| 数字 | 含义 | 来源 |
|---|---|---|
| 7 DoF | 拇指 4 + 食指 3，主从各 7 个电机一一对应 | 〔论文〕DITTO, arXiv 2609.19196 |
| 200 Hz | DITTO 双边控制回路频率 | 〔论文〕DITTO |
| 6.9 N vs 13.0 N | 用户实验：有力反馈时完成任务的平均接触力 6.9 N，无反馈 13.0 N，约减半 | 〔论文〕DITTO |
| 86.1 % vs 61.1 % | 物体大小辨识实验：关节力反馈和指尖振动同时开 86.1 %；只开其中一个都是 61.1 % | 〔论文〕DITTO-X, arXiv 2610.00781 |
| 78.3 % vs 27.7 % | 反向遥操作（人接管运行中的策略）成功率，DITTO-X 78.3 %，对比 Manus 数据手套 27.7 % | 〔论文〕DITTO-X |
| 三种手 | Sharpa Wave、Wuji 2、Inspire | 〔论文〕DITTO-X |
| ~9 motors | 我们按文中描述推算（食指 3 + 中指 3 + 拇指约 3）。被问到就说"三根主动手指，具体电机数以论文为准" | 〔估计〕 |

---

## P6 · Design the glove around the human hand, then derive an identical robot hand

**Slide**
- 对比表（6 列）+ Generality ⟷ Fidelity 轴
- What isomorphism buys: force feedback joint-to-joint (no estimation, no retargeting); reverse teleop is exact (write joint angles, no IK); the simulation twin is the robot itself; recorded data needs no retargeting
- Cost: we must build the hand, and its kinematics are bound to the human hand

**讲法**
DITTO-X 在通用那一端，我们在保真那一端。同构的四个好处逐条讲，代价说清楚。类比机械臂上的同构主臂（GELLO、HOMIE）。

**数字说明**：表里的数字全部来自 P3–P5，外加 MILE 一列：

| 数字 | 含义 | 来源 |
|---|---|---|
| MILE 4 指 17 DoF | 上海交大 2025-11（v5 2026-09）：先按人手设计 4 指 17 自由度的外骨骼，再按外骨骼做 MILE-Tac 机械手，关节一一对应、不用在线重定向，指尖有视触觉传感器 | 〔论文〕MILE, arXiv 2512.00324 |
| MILE 76 % | 四任务平均成功率 76 %，手套基线 28 %，纯视觉 8 % | 〔论文〕MILE |
| MILE "nothing" | 外骨骼只有编码器、没有电机，论文原话"不向操作者提供任何主动力或触觉反馈"，触觉图像只显示在屏幕上；作者把操作者侧力反馈列为未来工作 | 〔论文〕MILE |

"Ours" 列的 "robot in the loop: sim / glove #2" 是实话：机械手还没造，现在从手是 MuJoCo 虚拟手或第二只手套。

---

## P7 · 13 joints, four fingers, every axis where the human joint is

**Slide**
- Thumb 4 (CMC abduction, CMC flexion, MCP flexion, IP); index / middle / ring 3 each (MCP flexion, MCP abduction, PIP)
- Double-parallelogram MCP → virtual axis inside the finger (middle finger)
- Distal 4 joints XL330-M077 (77.5:1) for easier back-driving, others XL330-M288 (288:1)
- OpenRB-150, 1 Mbps, ≈ 220 Hz loop; Onshape → URDF / MuJoCo export automated, closed chains included

**讲法**
"人手优先"的证据就是虚拟转轴和解剖关节对齐：中指 MCP 用双平行四边形连杆把转轴放进手指里，电机不在转轴上。骨架图上红色是弯曲轴、蓝色是侧摆/对掌轴、数字是电机 ID。

**数字说明**
| 数字 | 含义 | 来源 |
|---|---|---|
| 13 = 4 + 3 + 3 + 3 | 见 P1 | 设计 |
| 77.5:1 | XL330-M077 减速比，用在 4 个远端关节（拇指 IP、三指 PIP，ID 4/7/10/13），减速比低更容易反拖、手感轻 | DYNAMIXEL 规格 |
| 288:1 | XL330-M288 减速比，其余 9 个关节 | DYNAMIXEL 规格 |
| 1 Mbps | 总线波特率，OpenRB-150 的上限 | 硬件 |
| ≈ 220 Hz | 单总线 13 个电机"读位置速度电流 + 写指令"的实测循环频率 | 〔实测〕 |
| ID 1–13 | 拇指 1–4、食指 5–7、中指 8–10、无名指 11–13，每根从掌根往指尖编 | 约定 |

---

## P8 · It works: sim-in-the-loop force feedback and a two-glove bilateral pair

**Slide**
- Onshape → URDF / MuJoCo export automated; 13 motors ID'd and calibrated
- Force only in contact, only against penetration; 13-channel current budget; 100 ms bus watchdog
- Chart: feedback torque vs position error (kp 0.4 N·m/rad, deadband ±2°, limit 0.06 N·m)
- MuJoCo twin driven by the glove: grasp and lift the cube 7.8 cm
- ≈ 220 Hz (DITTO 200 Hz); per-joint limit 0.06 N·m (DITTO 0.08)
- [VIDEO 1] glove drives the MuJoCo hand · [VIDEO 2] two gloves pinching a cup

**讲法**
先放视频再讲参数。控制律：e = q_主 − q_从；τ = −kp·(|e| − 死区)·sign(e)；只在接触时出力、只往"压进去"的反方向推。

**数字说明**
| 数字 | 含义 | 来源 |
|---|---|---|
| kp 0.4 N·m/rad | 仿真在环版本的主手反馈刚度：每压进 1 rad（57°）给 0.4 N·m，所以 10° 时约 0.07 N·m | 我们的参数（force_feedback.py） |
| 死区 ±2° | 位置差小于 2° 不出力，避免噪声抖动 | 同上 |
| 0.06 N·m（≈ 170 mA） | 单关节反馈力矩上限；换算：0.06 / Kt(M288) 0.354 N·m/A ≈ 170 mA | 同上；Kt 见 P9 |
| 今天双手套版本的参数 | kp 0.8 N·m/rad，死区 2°，上限 0.15 N·m，主手电流上限 450 mA（USB 供电限制），从手 P 增益 600、电流上限 400 mA，接触门控 100 mA | 〔实测〕bilateral.py 今天的命令 |
| DITTO 0.08 N·m / 200 Hz | DITTO 的单关节反馈上限和回路频率，用来对标 | 〔论文〕DITTO |
| 7.8 cm | MuJoCo 里虚拟手把积木抓离地面的高度。**诚实说明**：仿真手指没有软指垫，纯摩擦只能抬 1 cm，所以检测到捏合后用了一个焊接约束把积木固定在指尖；这段演示说明的是遥操作和力反馈，不是摩擦抓取的物理 | 〔实测〕sim_scenes.py |
| 100 ms | 总线看门狗：100 ms 收不到指令电机自动清零电流，防止程序卡死时手套失控 | 我们的安全设置 |
| 2.5 A | 主手 13 路电流指令之和的上限（电源预算） | 同上 |

---

## P9 · Every cycle we log command, result and force; stiffness itself can be an action

**Slide**
- Logged per joint at 220 Hz: q* (leader = command), q (follower = result), τ (follower current × Kt = contact torque), I (current rendered to the operator), K_t (follower stiffness this cycle)
- ① Force as observation: policy input += τ (FACTR)
- ② Force as action: action chunk = (q*, τ*), hybrid position / force control; Goal Current is the action (Bi-ACT, ForceMimic)
- ③ Impedance as action: action chunk = (q*, K_t), τ_cmd = K(q* − q) + D(q̇* − q̇); label K_t = f(τ_t): high force → soft (ACP rule); XL330 P / D gains writable online (ACP, Comp-ACT, CompVLA)
- Measured: K = 0.14 vs 0.58 N·m/rad for Kp 150 vs 600; same pinch gives 25 vs 57 mN·m; Kp scheduled by contact current

**讲法**
三种方法递进：① 策略只"看见"力；② 策略"指定"力；③ 策略指定"力和位置的关系"，让底层在接触时自己决定力。老师关心的"Kp/Kd 随电流变化"就是 ③。然后讲三张实测图：轻捏对用力捏的电流曲线（人在力反馈下调节的抓握力被机器人执行器记录下来了，这是 UMI 拿不到的）；两条刚度斜率（K 可测、可设）；Kp 随接触电流下降（K_t = f(τ_t) 实物跑通）。不要说"我们训练了策略"，这是数据管线和刚度调度的演示。

**数字说明**
| 数字 | 含义 | 来源 |
|---|---|---|
| Kt = 0.354 N·m/A（M288） | 力矩常数：堵转力矩 0.52 N·m ÷ 堵转电流 1.47 A。我们用它把从手电流换算成接触力矩 τ = I·Kt | DYNAMIXEL 规格 + 换算 |
| Kt = 0.146 N·m/A（M077） | 同理：0.215 N·m ÷ 1.47 A | 同上 |
| Kp 150 / 600 | DYNAMIXEL 位置环 P 增益寄存器的值（无量纲，出厂默认 400） | 我们设定 |
| K = 0.14 / 0.58 N·m/rad | 从手的**物理**刚度：在接触样本（从手电流 > 25 mA）上，对"接触力矩 τ 对 位置差 e = q* − q"做最小二乘，斜率就是 K。测的是食指 MCP 关节，捏纸杯 | 〔实测〕plot_log.py |
| 4.1 ≈ 4.0 | 两个斜率之比 0.58/0.14 = 4.1，Kp 之比 600/150 = 4.0：物理刚度和 P 增益成正比，K ≈ Kp / 1030 N·m/rad。意义：策略输出一个 K，可以直接换算成寄存器值写进电机 | 〔实测〕推算 |
| 25 vs 57 mN·m | 两段录制里的最大接触力矩。同一个人、同一个杯子、同样的位置指令，从手软时机器人只出 25 mN·m，硬时 57 mN·m：**机器人出多大力由 K 决定，所以 K 必须在动作里** | 〔实测〕 |
| 600 → ~380 | 自适应那段：Kp = 600 − 450 · min(1, Ī / 200 mA)，Ī 是从手电流 0.3 s 滑动平均。接触电流到约 100 mA 时 Kp 降到 380 左右（没到设定的最低 150，因为电流没到 200 mA 的参考值）。规则本身跑通了 | 〔实测〕bilateral.py --adaptive-kp |
| 0.34 N·m/rad | 自适应那段的平均斜率，介于软硬之间，因为接触过程中 Kp 在变 | 〔实测〕 |
| 100 mA | 接触门控：从手电流到 100 mA 门全开、50 mA 开始开；低于它主手不出力，保证空中自由 | 我们的参数 |
| 0.03 N·m | 电流反射项的阈值：从手力矩超过 0.03 N·m 的部分乘 kc 反射给主手 | 我们的参数 |
| 220 Hz | 见 P2 | 〔实测〕 |

---

## P10 · What is missing, and what we do next

**Slide**
- Limitations: no physical robot hand yet (feedback from the MuJoCo twin or glove #2); no user study, no policy results; no fingertip vibration (DITTO-X ablation: both channels needed); no pinky, one glove size; back-drive friction of 288:1 joints not quantified (DITTO chose 181:1); power: 5-cell NiMH drifts 5.0–7.0 V around the 3.7–6.0 V XL330 spec, one 12 V-class motor runs under-volted
- Next: build the isomorphic robot hand → quantify and compensate back-drive friction → validate methods ① → ② → ③ on the real pair → user study → reverse teleoperation + DAgger
- Questions: gear ratio (transparency vs torque)? how many thumb DoF? should the pair also drive non-isomorphic hands?

**讲法**
用小打分卡讲那个半圆：机器人在环现在是仿真或第二只手套，机械手是下一步。主动说局限比被问出来好。

**数字说明**
| 数字 | 含义 | 来源 |
|---|---|---|
| 181:1 | DITTO 主手特意选 XC330-T181（181:1）降低反拖阻力；我们大部分是 288:1，透明度是要量化的风险 | 〔论文〕DITTO |
| 5.0–7.0 V | 5 节镍氢：充满 7.0 V 以上，放电平台 6.0–6.5 V，到 5.0 V 必须停。XL330 规格 3.7–6.0 V，所以充满时略超规格 | 规格 + 实测 |
| 12 V-class motor | 一个 XC330-T288（规格 6.5–12 V）混在了 6 V 总线上，欠压运行、力矩偏小；它的欠压保护下限 5.5 V 改不了，所以这条总线不能低于 5.5 V | 硬件现状 |
| 今天 USB 供电 | 主手套走电脑 Type-C 5 V 3 A，线上压降后电机端约 4.4 V，所以主手电流限在 450 mA | 〔实测〕 |

---

## 可能被问的问题

**Q: 220 Hz 怎么来的，为什么不是 1 kHz？**
1 Mbps 总线上一次 Sync Read 13 个电机（每个 10 字节：电流、速度、位置）加一次 Sync Write，来回约 4 ms，再加 USB 延迟，实测 200–240 Hz。DITTO 是 200 Hz。要更高得换 U2D2 走 4 Mbps 或分两条总线。

**Q: 刚度 K 是怎么测的？**
从手位置模式，被挡住时位置差 e 和电流都记录下来；电流乘力矩常数得接触力矩；接触样本上做最小二乘，斜率就是 K。它是从手的**有效**刚度（电机 P 增益、减速箱、连杆合在一起的），不是人的刚度。

**Q: 为什么 K ≈ Kp/1030？**
实测两组（150 → 0.14，600 → 0.58）的比值一致，说明线性；1030 是经验系数，包含了 P 增益到 PWM、电机力矩常数和减速比。只有两个点，再多测几个 Kp 才能说是定律。

**Q: 自适应那段 Kp 为什么没降到 150？**
降多少由接触电流和参考值 200 mA 的比例决定，这次接触电流只到 100 mA，所以降到一半左右（约 380）。把参考值改成 100 mA 就会降到底。

**Q: 这和 ACP 有什么区别？**
ACP 是在手持设备上装力传感器、事后用规则生成刚度标签、机械臂末端做阻抗控制；我们是关节级、机器人在环、刚度实时在从手上执行，而且执行端就是电机寄存器，不需要额外的阻抗控制器。

**Q: 7.8 cm 是真的摩擦抓起来的吗？**
不是。仿真手指没有软指垫，纯摩擦只能抬约 1 cm，演示里检测到捏合后用约束把积木固定在指尖。这段说明的是遥操作和力反馈闭环，不是抓取物理。

**Q: 力反馈有多大？能挡住手吗？**
主手单关节最大 0.15 N·m，指尖约 2–3 N，是提示不是硬墙；DITTO 的结果也是把接触力减半而不是挡住。要硬墙得换减速比更低、力矩更大的电机。

**Q: 两只手套指尖对不齐是怎么回事？**
关节角对齐到 1° 以内，但主手里有人的指尖、从手是空指套，闭合时人的指尖先碰上。现在用张开和捏合两组姿态拟合了一个每关节线性映射补偿；根本解法是给从手（机械手）装真正的指尖。

**Q: 和 MILE 有什么区别？它也是"按人手做外骨骼、再按外骨骼做机械手"。**
是，MILE 是最接近的工作，而且早（2025-11）。区别在外骨骼有没有电机：MILE 的外骨骼是被动的、只有编码器，操作者感不到机器人碰到了什么，作者自己写了这是未来工作；我们的手套里每个关节都有电机，所以是双边的，能把机器人的接触力渲染回来，也才有"刚度作为动作"这条线。MILE 的指尖视触觉是我们没有的，这是互补的点。

**Q: SEED-UMI 呢？**
北大 2026-09，人和机器人轮流戴同一个外骨骼，一一对应、不用重定向；但采集时机器人不在环，人直接碰物体，没有力反馈。它走的是 UMI 那条路（③ 真实接触、④ 否），我们走的是 DITTO 那条路（④ 是、③ 渲染）。

**Q: 为什么没有小指？**
先验证 4 指 13 关节的同构和力反馈，小指模块加上去是同一套设计。

---

## 另一版 PPT（时间线那页）里出现的工作，我核实过的

| 条目 | 核实结果 | 放在哪一轨 |
|---|---|---|
| MILE（SJTU 2025-11, arXiv 2512.00324） | 真。机械同构外骨骼 + 视触觉机械手，被动外骨骼、无力反馈，4 指 17 DoF，76 % | 遥操作（机器人在环） |
| SEED-UMI（PKU 2026-09, arXiv 2609.11753） | 真。人和机器人共用一个外骨骼，Wuji 手 20 DoF，无力反馈，采集时机器人不在环，效率 3.0×，70 % | 演示采集 |
| Feel Robot Feels / TAG（2026-03, arXiv 2603.28542，贾飞宇等，庞江淼组） | 真。磁式 21 DoF 动捕 + 每指 32 个触觉执行器阵列，触觉在环遥操作；不是同构 | 遥操作 |
| DexEXO（2026-03, arXiv 2603.17323，Dennis Hong 组 → UCLA） | 真。可穿戴优先的外骨骼，适配手长 140–217 mm；演示采集 | 演示采集 |
| ExoStart（2025-06, arXiv 2506.11775，CMU + DeepMind + MIT） | 真。传感外骨骼采演示，无机器人在环，仿真动力学滤波 + RL | 演示采集 |
| OSMO（2025-12, arXiv 2512.08920，Meta FAIR） | 真。12 个磁式触觉传感器的触觉手套，人到机器人技能迁移，擦拭任务 72 % | 演示采集 |
| DITTO（2026-09, arXiv 2609.19196, Palacios…Ciocarlie） | 真，Ciocarlie 组是 Columbia，"Columbia"标注正确 | 遥操作 |
| DOGlove（Tsinghua 2025）、HOMIE（Shanghai AI Lab 2025）、GEX（2025）、DexCap（Stanford 2024）、DEXOP（MIT 2025）、STAG（MIT 2019）、BiDex（CMU）、iCub3 avatar（IIT）、NimbRo avatar（Bonn）、CyberGrasp（1999）、Rutgers Master II（2002）、Sayre Glove（1977）、VPL DataGlove、CyberGlove | 都是真实工作 | — |
| DAPG（UW 2018） | 是策略学习方法（Demo Augmented Policy Gradient），用 CyberGlove 在仿真里采演示，放在"遥操作"轨不合适，建议删掉 | — |
| "DataGlove teleop NYU 1989" | 没核实到，建议删掉 | — |
| "Shaquille O'Neal hand cast; source TBD" | 来源未定，组会上别用这张图；"手长 140–217 mm"换成引用 DexEXO 即可 | — |
