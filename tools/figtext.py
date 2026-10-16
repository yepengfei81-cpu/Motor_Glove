"""图里文字的中英文切换：环境变量 FIG_LANG=zh 出中文，默认英文。用法: from figtext import T; T("bones")"""
import os

LANG = os.environ.get("FIG_LANG", "en")
_T = {
    # skeleton
    "legend_flex": ("Flexion axis (number = motor ID)", "弯曲轴（数字 = 电机 ID）"),
    "legend_abd": ("Abduction / opposition axis", "侧摆 / 对掌轴"),
    "legend_bones": ("Skeleton: wrist → MCP → PIP → fingertip", "骨架：腕 → MCP → PIP → 指尖"),
    # fig_ppt
    "thumb": ("Thumb (ID 1–4)", "拇指 (ID 1–4)"), "index": ("Index (ID 5–7)", "食指 (ID 5–7)"),
    "mid": ("Middle (ID 8–10)", "中指 (ID 8–10)"), "ring": ("Ring (ID 11–13)", "无名指 (ID 11–13)"), "base": ("Dorsal base", "手背基座"),
    "rcm_title": ("Middle-finger MCP, double parallelogram: the phalanx rotates about a virtual axis (red)\ninside the finger while the motor sits off-axis",
                  "中指 MCP：双平行四边形连杆，指节绕手指内部的虚拟转轴（红）转动，电机不在转轴上"),
    "rcm_axis": ("Virtual axis (fixed in all three poses)", "虚拟转轴（三个姿态下不动）"),
    "rom_title": ("Range of motion of the 13 motor joints: open / half / fist overlay", "13 个电机关节的活动范围：零位 / 半握 / 握拳 叠影"),
    "pose0": ("Open (0°)", "零位"), "pose1": ("Half (35°)", "半握（35°）"), "pose2": ("Fist (70°)", "握拳（70°）"),
    # diagrams
    "sys_title": ("Isomorphic bilateral teleoperation: position forward, force back, joint-to-joint, no retargeting",
                  "同构双边遥操作：位置正向、力反向，关节对关节，不需要重定向"),
    "sys_leader": ("Leader glove (worn)", "主手套（戴在手上）"),
    "sys_leader_l": (["13 × DYNAMIXEL XL330", "Current mode: renders feedback", "Reads angle, velocity, current"], ["13 × DYNAMIXEL XL330", "电流模式：渲染反馈力", "读：关节角、速度、电流"]),
    "sys_board": ("OpenRB-150", "OpenRB-150"), "sys_board_l": (["1 Mbps bus", "Motor power 5–6 V", "USB → PC"], ["1 Mbps 总线", "电机供电 5–6 V", "USB → PC"]),
    "sys_pc": ("PC (≈220 Hz)", "PC（≈220 Hz）"),
    "sys_pc_l": (["MuJoCo twin / policy", "τ = −kp·(|e|−deadband)·sign(e)", "Logs (q*, q, τ)"], ["MuJoCo 数字孪生 / 策略", "τ = −kp·(|e|−死区)·sign(e)", "记录 (q*, q, τ)"]),
    "sys_follower": ("Follower hand", "从手套 / 机械手"),
    "sys_follower_l": (["Glove #2 / robot hand, 13 joints", "Position mode + current limit", "Blocked → error e grows"], ["同一套 13 关节（1:1）", "位置模式 + 电流上限", "被挡住 → 位置差 e 变大"]),
    "sys_q": ("q_L", "q 主"), "sys_qstar": ("q* = q_L", "q* = q 主"), "sys_back": ("q_F, I_F", "q 从, I 从"), "sys_i": ("I_L ← τ", "I 主 ← τ"),
    "sys_note1": ("Free motion: follower keeps up, e ≈ 0, no force        Contact: follower blocked, e grows, leader pushes the finger back",
                  "自由运动：从手跟得上，e ≈ 0，主手不出力        接触：从手被挡住，e 增大，主手把手指往回推"),
    "sys_note2": ("The follower can be the virtual hand in MuJoCo (sim-in-the-loop) or a second physical glove", "从手可以是 MuJoCo 里的虚拟手（仿真在环），也可以是第二只实物手套"),
    "law_line": ("Leader feedback torque", "主手反馈力矩"), "law_db": ("Deadband ±2° (no force in free space)", "死区 ±2°（空中不出力）"),
    "law_lim": ("Torque limit ±0.06 N·m (≈170 mA)", "力矩上限 ±0.06 N·m（≈ 170 mA）"),
    "law_slope": ("Slope kp = 0.4 N·m/rad\n= wall stiffness", "斜率 kp = 0.4 N·m/rad\n= 墙的硬度"),
    "law_x": ("Position error e = q_leader − q_follower (°): penetration into the \"wall\"", "位置差 e = q主 − q从 （°）：手指压进“墙”的深度"),
    "law_y": ("Feedback torque (mN·m)", "反馈力矩 (mN·m)"),
    "law_title": ("Force-feedback law: only in contact, only against the direction of penetration", "力反馈控制律：只在接触时、只往“压进去”的反方向出力"),
    "rec_title": ("What is logged per joint per cycle, and how it becomes supervision for a policy", "每个周期、每个关节记录的东西，以及它们怎么变成策略的监督信号"),
    "rec_log": ("Logged (13 joints × 220 Hz)", "记录（13 关节 × 220 Hz）"),
    "rec_log_l": (["q*  leader angle = command", "q   follower angle = result", "τ   follower current → contact torque", "I   current rendered to the operator"],
                  ["q*  主手角度 = 指令", "q   从手角度 = 执行结果", "τ   从手电流 → 接触力矩", "I主 渲染给人的力"]),
    "rec_1": ("① Force as observation", "① 力作为观测"), "rec_1_l": (["policy input += τ", "(current is also available at deployment)"], ["策略输入 += τ（部署时电机电流同样可得）"]),
    "rec_2": ("② Force as action", "② 力作为动作"), "rec_2_l": (["action chunk = (q*, τ*)", "hybrid position / force control"], ["动作块 = (q*, τ)，底层混合位置/力控制"]),
    "rec_3": ("③ Impedance as action", "③ 阻抗作为动作"),
    "rec_3_l": (["K_t = f(τ_t): high force → soft (ACP rule)", "or identify human K, D by perturbation", "action chunk = (q*, K), impedance control", "τ_cmd = K(q*−q) + D(q̇*−q̇)"],
                ["K_t = f(τ_t)：力大 → 软（ACP 规则）", "或 扰动辨识人的 K(t)、D(t)", "动作块 = (q*, K)，底层阻抗控制", "τ_cmd = K(q*−q) + D(q̇*−q̇)"]),
    "rec_dep": ("Deployment", "部署"), "rec_dep_l": (["Isomorphic robot hand", "Same controller", "No retargeting"], ["同构机械手", "同一套控制器", "无需重定向"]),
    "rec_refs": ("Related: FACTR 2025 (①) · Bi-ACT 2024 / ForceMimic 2025 (②) · ACP 2024 / Comp-ACT 2024 / CompVLA 2026 (③)",
                 "代表工作：FACTR 2025（①）· Bi-ACT 2024 / ForceMimic 2025（②）· ACP 2024 / Comp-ACT 2024 / CompVLA 2026（③）"),
    # plot_log
    "pl_leader": ("leader", "主手"), "pl_follower": ("follower", "从手"), "pl_angle": ("Angle (°)", "角度 (°)"), "pl_time": ("Time (s)", "时间 (s)"),
    "pl_t1": ("{j}: leader command vs follower", "{j}：主手指令 vs 从手实际"),
    "pl_tauB": ("follower (contact) torque", "从手(接触)力矩"), "pl_tauA": ("leader feedback torque", "主手反馈力矩"),
    "pl_torque": ("Torque (mN·m)", "力矩 (mN·m)"), "pl_t2": ("Follower current → contact torque; leader feedback", "从手电流 → 接触力矩；主手反馈"),
    "pl_slope": ("slope", "斜率"), "pl_nocontact": ("(no contact)", "（没有接触）"),
    "pl_x3": ("|q_leader − q_follower| (°)", "位置差 |q主 − q从| (°)"), "pl_y3": ("|contact torque| (mN·m)", "|接触力矩| (mN·m)"),
    "pl_t3": ("In contact: torque vs position error\n(slope = follower stiffness)", "接触阶段：力矩 vs 位置差（斜率 = 从手有效刚度）"),
    "pl_t4": ("Follower stiffness K_t = f(τ_t):\nP gain drops as contact current rises", "从手刚度 K_t = f(τ_t)：接触电流升高，P 增益下降"),
    "pl_kp": ("P gain", "P 增益"), "pl_iB": ("|I_follower| (mA)", "|从手电流| (mA)"),
}


def T(key, **kw):
    v = _T[key][0 if LANG == "en" else 1]
    return v.format(**kw) if isinstance(v, str) and kw else v
