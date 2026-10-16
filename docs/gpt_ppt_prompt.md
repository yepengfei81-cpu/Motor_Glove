# 任务：按我的 PPT 模板做一版 10 页组会汇报初稿（英文幻灯片 + 中文演讲备注）

你已经了解我这个项目的大概：一只戴在手上的外骨骼手套，和一只与它机械结构完全相同的机械手（同构，isomorphic），用 DYNAMIXEL 小电机做关节级的力反馈遥操作。下面是这两周我们把项目推进到的程度、明天组会要讲的故事线、每一页的内容、所有核对过的论文数字，以及我会提供给你的图片。请你：

1. 用我提供的 PPT 模板（我会上传）生成 **10 页**的第一版 .pptx 初稿，16:9。
2. **幻灯片正文用英文**，每页的**演讲者备注用中文**（把我下面给的"怎么讲"写进备注，每页备注 100–200 字）。
3. 去各篇论文（arXiv 号我都给了）**找我指定的插图**放进对应页面，并在图下标注来源（论文简称 + Fig. 编号）。找不到就留一个灰色占位框，写明"[论文名 Fig. X]"。
4. 我提供的图片（文件名在最后一节）按我指定的页面放。视频用占位框 "[VIDEO: …]"。
5. **不要编造任何数字**。所有数字只能用我下面给的；没给的地方写 "TBD"。
6. 每页正文不超过 6 条要点，每条不超过 2 行；标题是一句有观点的话（我每页都给了），不是名词短语。
7. 右上角放我给的小打分卡图（scorecard_xxx.png），第 3–6 页和第 10 页用。
8. 做完后列出：你没找到的论文图、你拿不准的地方、你觉得哪页信息过多。

---

## 一、项目现状（你之前不知道的部分）

- **硬件**：外骨骼手套，13 个 DYNAMIXEL XL330 电机 = 13 个主动关节。拇指 4 个（CMC 侧摆 / 对掌、CMC 弯曲、MCP 弯曲、IP），食指、中指、无名指各 3 个（MCP 弯曲、MCP 侧摆、PIP），没有小指。电机 ID 1–13 按拇指 → 食指 → 中指 → 无名指、每根手指从掌根往指尖编号。远端 4 个关节（PIP / IP）用减速比 77.5:1 的 XL330-M077，更容易反拖；其余是 288:1。
- **机构**：手指关节用双平行四边形连杆（double parallelogram），电机不在转轴上，转轴是"虚拟转轴"，落在人手指关节的位置；这是"按人手关节轴设计"的证据。
- **电子**：OpenRB-150 控制板，1 Mbps 总线，6 V 电池，USB 到 PC。控制回路在 PC 上跑，实测约 220 Hz（13 个电机全部读位置、速度、电流并写指令）。
- **第二只手套**：我们又造了一只一模一样的手套，放在桌上当"从手"（follower），戴在手上的是"主手"（leader）。机械手本体还没造，所以现在"机器人在环"有两种形式：MuJoCo 里的虚拟手（仿真在环），和第二只实物手套。
- **软件链路已经打通**：Onshape CAD → 自动导出 URDF 和 MuJoCo 模型（闭链连杆都处理了）→ 13 个电机设 ID、标零点和方向 → 实时镜像到 MuJoCo → **力反馈已经跑通**：戴手套驱动 MuJoCo 里的虚拟手抓积木，虚拟手碰到积木时把力反馈到手套上，操作者捏不下去；虚拟手能把积木抓起 7.8 cm。
- **力反馈控制律**（关节级，每个关节独立）：e = q_主 − q_从；τ = −kp·(|e| − 死区)·sign(e)，kp = 0.4 N·m/rad，死区 ±2°，单关节力矩上限 0.06 N·m（约 170 mA）；只在接触时出力（自由运动时从手跟得上，e 在死区内，主手不出力），并且只往"压进去"的反方向推，不往里拉。13 路电流总和有上限，100 ms 总线看门狗。
- **双手套主从**：主手电流模式渲染反馈力；从手是电流限位的位置模式（DYNAMIXEL 模式 5），目标位置 = 主手角度，电流上限 = 抓握力上限。从手被挡住时位置差 e 变大，主手被往回推。
- **记录的数据**（13 关节 × 220 Hz）：q\*（主手角度 = 指令）、q（从手角度 = 执行结果）、τ（从手电流 × 力矩常数 = 接触力矩）、I（渲染给人的电流），以及从手每个周期的刚度 K_t（P 增益）。

## 二、主线论点（整个 PPT 的逻辑）

**开场论点：UMI 这类无本体采集记录的是 state，不是 action。** 人用手捏夹爪，记录的是夹爪的位姿和开度；碰到物体以后开度不变，人捏了多大力完全没有被记录下来，因为采集时根本没有电机。策略部署时需要的是机器人本体的动作：力、柔顺、执行器动力学、可行性，这些只有**机器人在环**时才会被记录下来。

证据（都核对过原文）：
- DITTO（arXiv 2609.19196，Sec. VI-B）：只用手持数据训练的策略在"抓住并抬起"阶段成功率 **0%**；只加 10 条遥操作数据就升到 **70%**。作者的解释：施力的是人，不是机器人的执行器，所以手持演示"不捕获机器人的执行器动力学"。
- ForceMimic（arXiv 2410.07554，ICRA 2025）：人在接触任务里施加随时间变化的力，而现有算法主要学轨迹；给手持设备加力传感后成功率相对纯视觉提升 54.5%。
- Adaptive Compliance Policy, ACP（arXiv 2410.09309，Shuran Song 组，UMI 作者自己的后续工作）：柔顺性常被只做位置控制的视觉运动策略忽略。
- UMI 自己写的局限：采集时不知道机器人运动学是否可行，只能事后过滤。

由此得到 **好的灵巧操作数据要满足四条要求**（贯穿全片的打分卡）：
① Dexterous：足够多的独立自由度；② No embodiment gap：人的动作机器人能 1:1 原样执行；③ Operator feels contact：操作者能感到接触（真实的或渲染的）；④ Robot in the loop：机器人在环，能接管、能做 DAgger、记录执行器动力学。

**前人每一步解决一部分**：UMI 解决 ②③，DexUMI 用硬件适配解决一部分 ②，ART-Glove 解决 ①③，DITTO 第一次把 ②③④ 放在一起但只有 7 个自由度，DITTO-X 为了通用性（一个外骨骼驱动三种手）放弃了同构。**我们选另一端：把一一对应做到全手 4 指 13 自由度，而且主手和从手是一起设计的（人手优先：先按人手关节轴设计手套，再派生完全相同的机械手）。** DexUMI 是"机器人优先"（先有商用手，再做迁就它的外骨骼），我们正好相反。

**同构带来的四个好处**：力反馈关节对关节，不用估计、不用重定向；反向遥操作（策略在跑、人接管）是精确的，直接写关节角，不用解逆运动学；仿真孪生就是机器人本身；采到的数据不需要重定向。**代价**：机械手要自己造，机器人手的运动学被人手约束。类比：机械臂上已经有同构主臂（GELLO、HOMIE）证明这条路可行。

## 三、力和阻抗作为监督信号的三种方法（老师最感兴趣的是方法 3）

我们的采集链路里接触发生在从手（虚拟手 / 第二只手套 / 以后的机械手）。从手被挡住后主手继续压，位置差变大，从手电流上升，这个力经主手反馈给人，人据此决定捏多紧。所以记录里同时有"机器人执行器为了保持抓握出了多少力"和"人对这个力的调节"，这两样 UMI 都没有。三种方法的区别在于这个力在策略里扮演什么角色：

1. **Force as observation（力作为观测）**：策略输入多一项从手电流 τ，输出仍是位置 q\*。部署时同一只机械手的电流同样可得。代表：FACTR（arXiv 2502.17432）。
2. **Force as action（力作为动作）**：策略每步输出 (q\*, τ\*)，底层做混合位置 / 力控制：空中跟位置，接触后按 τ\* 出力。标签 τ\* 就是记录里从手的电流，即"人在感到力的情况下让机器人出的力"。在我们硬件上，模式 5 的 Goal Current 本来就是电流上限，把它当动作每步写入，就是一个现成的混合控制器。代表：Bi-ACT（arXiv 2401.17698）、ForceMimic。
3. **Impedance as action（阻抗作为动作）**：策略输出 (q\*, K_t)，可能再加 D_t，底层做阻抗控制 τ_cmd = K_t(q\* − q) + D_t(q̇\* − q̇)。标签 K_t = f(τ_t)：用一条规则从测到的接触力生成刚度，**力大就软、空中就硬**（ACP 的做法：沿受力方向降低刚度，幅度随力增大而减小，用 1 s 滑动平均）。这就是"Kp / Kd 随电流变化"：执行时刚度不是常数，而是策略根据接触情况输出的。XL330 的位置环 P、D 增益在通电状态下可以随时改写，所以 (q\*, Kp, Kd, 电流上限) 可以整组作为动作。更进一步的变体是用主手电机打小扰动来辨识人手指本身的 K(t)、D(t)（Hajian & Howe 1997；tele-impedance, Ajoudani 2012），作为未来工作。代表：ACP、Comp-ACT（arXiv 2406.14990）、CompVLA（arXiv 2609.23614）。

三者递进：方法 1 策略只"看见"力，方法 2 策略"指定"力，方法 3 策略指定"力和位置的关系"。我们今晚会在两只手套上做方法 3（刚度随接触电流实时变化）和方法 2（回放记录的力）的演示，数据图明天早上补，第 9 页先留占位。

## 四、逐页内容（10 页，共 15 分钟）

**P1 Title（0.5 min）**
- 标题：Isomorphic Force-Feedback Exoskeleton Glove: 13 DoF, Joint-to-Joint Bilateral Teleoperation for Dexterous Data Collection
- 副标题：Lab meeting, 2026-10-15；我的名字（我自己填）
- 图：cad_palm_oblique.png（左），右边留实物照片占位 "[PHOTO: two gloves]"
- 备注：一句话说清楚这是什么：一只按人手设计的手套和一只完全相同的机械手，关节一一对应，带力反馈。

**P2 UMI collects state, not action（2 min）**
- 标题：Handheld devices record what the human did, not what the robot must do
- 要点：
  - UMI-style collection: no motor in the loop → gripper width stops at contact, grasp force is never recorded
  - A policy needs robot actions: force, compliance, actuator dynamics, feasibility
  - DITTO: handheld-only data → 0% at "hold and lift"; +10 teleop episodes → 70%
  - ForceMimic: adding force to the action → +54.5% relative over vision-only
  - ACP (same group as UMI): "compliance is ignored by position-only visuomotor policies"
  - So: four requirements for good dexterous data → 放 scorecard.png（整页下半部分）
- 备注：这一页是全片的论点，讲清 state 和 action 的差别，再引出四条要求，告诉听众后面每篇论文都用这张卡打分。

**P3 UMI（1 min）**
- 标题：UMI: make the collection device identical to the robot's end-effector
- 要点：same soft gripper on both sides; wrist GoPro with 155° fisheye + side mirrors; latency matching + relative trajectory actions → cross-robot deployment；$73 printed parts, $298 GoPro kit；3× faster than SpaceMouse teleop；in-the-wild cup arrangement 71.7% (30 locations, 1400 demos)
- 局限：parallel gripper only; state-only; kinematic feasibility unknown at collection time (filtered afterwards)
- 图：去找 UMI 论文（arXiv 2402.10329）Fig. 1（手持采集到机器人部署）和夹爪硬件图
- 右上角：scorecard_umi.png
- 备注：UMI 的贡献是让无本体数据能直接训练出可部署的策略，而不是"开创了无机器人采集"（2020 年就有 Grasping in the Wild）。过渡：夹爪做不了灵巧操作，换成手会怎样？

**P4 Robot-first vs human-first exoskeletons（1.5 min）**
- 标题：DexUMI fits the exoskeleton to a robot hand; ART-Glove fits it to the human hand; neither has the robot in the loop
- 左半 DexUMI（arXiv 2505.21864）：hardware adaptation (exoskeleton optimized to the robot's fingertip workspace, joint encoders) + software adaptation (SAM2 segment human hand, ProPainter inpaint, render robot hand back); 86% average success; Inspire (12 joints, 6 active) and XHand (12 active); 局限（作者自己写的）：per-hand tuning, human force > robot force so tactile readings drift, rendering still needs the real robot, fixed camera only
- 右半 ART-Glove（arXiv 2606.16370）：16 rigid plates, 22 encoder-measured joints aligned to anatomical joints, 2048 taxels, 120 Hz; Kapandji score 10/10; 局限：measurement only, no robot, no policy; normal force only, one size; 作者未来工作：an actuated hand built on the glove geometry → 正是我们的路线
- 图：DexUMI Fig. 1（外骨骼视频变成补全后的机器人手视频）；ART-Glove 手套照片和触觉分布图
- 右上角：scorecard_dexumi.png 和 scorecard_artglove.png（并排两张小卡）
- 备注：DexUMI 不是效果差，是"机器人优先"，外骨骼迁就一只现成的手，而且人感到的力和机器人感到的力不是一回事。ART-Glove 是"人手优先"但没有机器人。

**P5 DITTO → DITTO-X: put the robot in the loop（2 min）**
- 标题：DITTO closes the loop with a kinematically equivalent leader; DITTO-X trades isomorphism for generality
- 左 DITTO（arXiv 2609.19196）：leader and follower kinematically equivalent, thumb 4 + index 3 = 7 DoF, 1:1 motor mapping; joint-level force feedback from follower current at 200 Hz; with feedback users apply about half the contact force (6.9 N vs 13.0 N); handheld-only 0% → +10 teleop 70%
- 右 DITTO-X（arXiv 2610.00781）：one exoskeleton drives Sharpa, Wuji, Inspire hands; index and middle 1:1, thumb retargeted in task space with Jacobian-pseudoinverse force estimate, ring and pinky coupled to middle, one thumb joint removed; force feedback + fingertip vibration complementary: object-size discrimination 86.1% with both, 61.1% with either alone; reverse teleoperation (take over a running policy) 78.3% vs Manus 27.7%; DAgger improves monotonically over two rounds
- 图：DITTO-X Fig. 1（正向和反向遥操作）、Fig. 2（运动学改动）、Table I；DITTO Fig. 1
- 右上角：scorecard_ditto.png 和 scorecard_dittox.png
- 备注：这是离我们最近的工作，老师读过。我们的差异必须讲成：同构做到 4 指 13 自由度、拇指完整 4 关节、主从一起设计；DITTO-X 为通用性放弃同构，我们选另一端。

**P6 Our position: human-first, fully isomorphic（1.5 min）**
- 标题：Design the glove around the human hand, then derive an identical robot hand
- 图：comparison_table.png 占整页上半；下半放一条"Generality ⟷ Fidelity"的轴，DITTO-X 在 generality 端，我们在 fidelity 端
- 要点（4 个好处 + 1 个代价，见第二节）
- 备注：类比 GELLO / HOMIE 的同构主臂。

**P7 Design: 13 motor joints aligned to anatomical axes（1.5 min）**
- 标题：13 joints, four fingers, every axis where the human joint is
- 图：skeleton_zero_palm_front.png（主图，红 = 弯曲轴、蓝 = 侧摆 / 对掌轴、数字 = 电机 ID）；modules_palm_front.png（按手指上色）；rcm_mid_mcp.png（中指 MCP 双平行四边形，三个姿态叠影，红球是不动的虚拟转轴）；rom_overlay_palm_oblique.png（活动范围叠影）
- 要点：thumb 4 (CMC abd, CMC flex, MCP flex, IP), index / middle / ring 3 each (MCP flex, MCP abd, PIP); double-parallelogram MCP → virtual axis inside the finger; distal 4 joints XL330-M077 (77.5:1) for easier back-driving, others 288:1; OpenRB-150, 1 Mbps, 6 V, ≈220 Hz loop
- 电子框图：system_diagram.png（可以放这页下方，或放 P8）
- 备注：强调"人手优先"的证据就是虚拟转轴和解剖关节对齐。

**P8 Progress and demo（2.5 min，最重要的一页）**
- 标题：It works: sim-in-the-loop force feedback and a two-glove bilateral pair
- 要点：Onshape → URDF / MuJoCo export automated (closed chains handled); 13 motors ID'd and calibrated; force only in contact, only against penetration, 13-channel current budget, ≈220 Hz (DITTO: 200 Hz); per-joint torque limit 0.06 N·m (DITTO: 0.08 N·m)
- 图：control_law.png（左）；sim_grasp_open.png 和 sim_grasp_hold.png（右上，"virtual hand lifts the cube 7.8 cm"）
- 视频占位：[VIDEO 1: glove drives the MuJoCo hand, grasps a cube, operator feels the block]；[VIDEO 2: two gloves, follower blocked by hand → leader pushes back]
- 右上角不放小卡
- 备注：先放视频再讲参数。

**P9 What we record, and how force / impedance become supervision（2 min，老师最关心）**
- 标题：Every cycle we log command, result and force; stiffness can be an action
- 图：data_record.png（上半页）；下半页留两个占位："[PLOT: soft vs firm grasp, follower current over time]"、"[PLOT: follower stiffness K_t = f(τ_t) dropping as contact current rises; τ-vs-e slope = K]"
- 要点：logged per joint at 220 Hz: q\*, q, τ (follower current × Kt), I (rendered to operator), K_t；① force as observation；② force as action: (q\*, τ\*) + hybrid position / force control (Goal Current is the action)；③ impedance as action: (q\*, K_t), K_t = f(τ_t): high force → soft (ACP rule); XL330 P / D gains are writable online, so (q\*, Kp, Kd, current limit) can all be actions；tonight's demo: stiffness scheduled from contact current on the real follower, and replay of recorded force (occupy placeholders)
- 备注：解释三种方法递进；重点讲方法 3，说明斜率就是可作为动作输出的 K。

**P10 Limitations, next steps, questions（1 min）**
- 标题：What is missing, and what we do next
- 右上角：scorecard_ours.png（第四格是半圆，主动说出来）
- 局限：no physical robot hand yet (feedback from sim or glove #2); no user study, no policy results; no fingertip vibration (DITTO-X ablation says both channels are needed); no pinky, one glove size; back-drive friction of 288:1 joints not yet quantified (DITTO chose 181:1 for transparency); 6 V supply slightly above XL330 spec, one 12 V motor under-volted
- 下一步：build the robot hand → quantify and compensate back-drive friction → validate methods 1–3 in order → user study → reverse teleop + DAgger
- 讨论题：gear ratio choice? how many thumb DoF? should we also support non-isomorphic hands?

**备用页（不计入 10 页，各一张即可，放最后）**
- DEXOP（arXiv 2509.04441）：被动外骨骼机械连接人手和机器人手，力天然传过来
- GEX（arXiv 2506.04982）：11 自由度三指手 + 12 自由度外骨骼，不是一一对应，需要重定向
- DOGlove：低成本力反馈手套，每根手指只有一个方向的力

## 五、打分卡数据（已做成图，这里给你核对）

| | UMI | DexUMI | ART-Glove | DITTO | DITTO-X | Ours |
|---|---|---|---|---|---|---|
| ① Dexterous | ✗ parallel gripper | ◐ Inspire 6 / XHand 12 active | ✓ 22 DoF measured | ◐ 2 fingers, 7 DoF | ◐ 3 fingers, ~9 motors | ✓ 4 fingers, 13 DoF |
| ② No embodiment gap | ✓ same gripper | ◐ fingertips only | ✗ retargeting needed | ✓ 1:1 joints | ◐ thumb retargeted | ✓ all 13 joints 1:1 |
| ③ Operator feels contact | ✓ real contact | ✓ real contact | ✓ real contact | ✓ rendered joint force | ✓ joint force + vibration | ✓ rendered joint force |
| ④ Robot in the loop | ✗ | ✗ | ✗ (measurement only) | ✓ | ✓ reverse teleop, DAgger | ◐ sim / glove #2 today |

## 六、对比表数据（已做成图 comparison_table.png）

| | UMI | DexUMI | ART-Glove | DITTO | DITTO-X | Ours |
|---|---|---|---|---|---|---|
| Dexterity | parallel gripper | Inspire 6 active / XHand 12 | 22 DoF measured | 2 fingers, 7 DoF | 3 fingers (~9 motors) | 4 fingers, 13 DoF |
| Correspondence to the robot | identical gripper | fingertips only | none (retargeting) | 1:1 | index, middle 1:1; thumb retargeted | all 13 joints 1:1 |
| What the operator feels | real contact | real contact | real contact | rendered joint force | joint force + fingertip vibration | rendered joint force |
| Robot in the loop | no | no | no | yes | yes, reverse teleop | yes (sim / glove #2; robot hand next) |
| Policy results | yes | yes, 86% avg. | none | yes | yes, with DAgger | not yet |

## 七、我会上传给你的图片（全部英文、白底、2000 px 宽）

| 文件 | 内容 | 用在 |
|---|---|---|
| cad_palm_oblique.png | 手套 CAD 干净渲染，斜视 | P1 |
| cad_palm_front.png / cad_dorsal.png / cad_thumb_side.png | 其他视角，备用 | 备用 |
| skeleton_zero_palm_front.png | 半透明 CAD 里的骨架 + 13 个关节轴 + 电机 ID | P7 主图 |
| skeleton_zero_palm_front_nomesh.png | 同上，不带 CAD，只有骨架和轴 | P7 备用 |
| skeleton_fist_palm_oblique.png | 握拳姿态的骨架 | 备用 |
| modules_palm_front.png | 按手指上色（拇指 ID 1–4、食指 5–7、中指 8–10、无名指 11–13） | P7 |
| rcm_mid_mcp.png | 中指 MCP 双平行四边形：0° / 30° / 60° 叠影 + 红色虚拟转轴 | P7 |
| rom_overlay_palm_oblique.png | 活动范围：open / half / fist 叠影 | P7 |
| system_diagram.png | 系统框图：主手套 → OpenRB → PC → 从手，位置正向、力反向 | P7 或 P8 |
| control_law.png | 力反馈控制律：力矩 vs 位置差（死区、斜率、饱和） | P8 |
| sim_grasp_open.png / sim_grasp_hold.png | MuJoCo 虚拟手张开 / 抓起积木 7.8 cm | P8 |
| data_record.png | 记录什么、三种方法怎么变成监督信号 | P9 |
| scorecard.png | 总打分卡 | P2 |
| scorecard_umi / dexumi / artglove / ditto / dittox / ours.png | 每个系统的 2×2 小卡 | P3–P6、P10 右上角 |
| comparison_table.png | 6 列对比表 | P6 |

## 八、需要你去论文里找的图

- UMI（arXiv 2402.10329）：Fig. 1 总览；夹爪 / 鱼眼相机硬件图
- DexUMI（arXiv 2505.21864）：Fig. 1（外骨骼视频 → 补全的机器人手视频）；外骨骼结构图
- ART-Glove（arXiv 2606.16370）：手套照片；触觉点分布图
- DITTO（arXiv 2609.19196）：Fig. 1；手持数据 0% vs 遥操作 70% 的那张表或图
- DITTO-X（arXiv 2610.00781）：Fig. 1（正向 / 反向遥操作）、Fig. 2（运动学改动）、Table I
- 备用页：DEXOP、GEX 的 Fig. 1

## 九、风格要求

- 用我上传的模板的配色和字体；图片尽量大，文字少；每页标题是一句有观点的话。
- 打分卡小图放右上角，宽约 1/5 页宽。
- 数字出现的地方都标来源（论文简称）。
- 第 8、9 页的视频和数据图用占位框，写清楚会放什么。
- 输出：.pptx 文件 + 一份每页备注的文本。
