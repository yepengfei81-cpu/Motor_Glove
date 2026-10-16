// 组会 PPT：按用户模板风格（白底、左上角藏青段落标签、大黑标题、灰副标题、蓝色小标题栏、灰色说明框、右下页码）
const pptxgen = require("pptxgenjs");
const { applyTheme } = require("/home/ypf/.claude/skills/synced/668fbcf9-15c1-4185-8be8-c0111ae9591e_33794ff7-8559-491a-ad8e-be774677bddd/pptx/scripts/apply_theme.js");
const FIG = "/home/ypf/motor_glove/docs/figs/ppt/";
const FIGS = "/home/ypf/motor_glove/docs/figs/";
const PAP = "/home/ypf/motor_glove/docs/figs/papers/";
const OUT = "/home/ypf/motor_glove/docs/lab_meeting_2026-10-15.pptx";

const THEME = {
  name: "Glove Lab Meeting",
  headFontFace: "Arial", bodyFontFace: "Arial",
  colors: { dk1: "111111", lt1: "FFFFFF", dk2: "0B3C6E", lt2: "EDEEF0", accent1: "1A73C7", accent2: "7A7A7A",
            accent3: "2F855A", accent4: "D69E2E", accent5: "C53030", accent6: "E8F1FB", hlink: "1A73C7", folHlink: "7A7A7A" },
};
const pres = new pptxgen();
pres.layout = "LAYOUT_WIDE";            // 13.33 x 7.5 in
pres.theme = { headFontFace: THEME.headFontFace, bodyFontFace: THEME.bodyFontFace };
pres.author = "Pengfei Ye"; pres.title = "Isomorphic Force-Feedback Glove";
const C = pres.SchemeColor;
const NAVY = "0B3C6E", BLUE = "1A73C7", GREY = "7A7A7A", INK = "111111", BOX = "EDEEF0", GREEN = "2F855A", AMBER = "D69E2E", RED = "C53030", TINT = "E8F1FB";
const W = 13.333, H = 7.5, M = 0.45;

// ---------- 版式 ----------
pres.defineSlideMaster({
  title: "CONTENT", background: { color: "FFFFFF" },
  objects: [
    { line: { x: 0, y: 0.04, w: W, h: 0, line: { color: NAVY, width: 2 } } },
    { rect: { x: 0, y: 0.08, w: 2.9, h: 0.62, fill: { color: NAVY } } },
    { line: { x: 0, y: H - 0.04, w: W, h: 0, line: { color: NAVY, width: 2 } } },
  ],
  slideNumber: { x: 12.55, y: 6.98, w: 0.5, h: 0.3, fontSize: 11, color: GREY, align: "right" },
});
pres.defineSlideMaster({ title: "TITLE", background: { color: "FFFFFF" },
  objects: [{ line: { x: 0, y: 0.04, w: W, h: 0, line: { color: NAVY, width: 2 } } }, { line: { x: 0, y: H - 0.04, w: W, h: 0, line: { color: NAVY, width: 2 } } }] });

// ---------- 小部件 ----------
let n = 0;
function slide(section, title, subtitle, notes, titleW = W - 2 * M) {
  const s = pres.addSlide({ masterName: "CONTENT" });
  n += 1;
  s.addText(section, { x: 0.18, y: 0.08, w: 2.7, h: 0.62, fontSize: 20, bold: true, color: "FFFFFF", margin: 0, valign: "middle", isTextBox: true });
  s.addText(title, { x: M, y: 0.76, w: titleW, h: 0.9, fontSize: 25, bold: true, color: INK, margin: 0, valign: "bottom", isTextBox: true });
  if (subtitle) s.addText(subtitle, { x: M, y: 1.63, w: W - 2 * M, h: 0.36, fontSize: 14, color: GREY, margin: 0, valign: "middle", isTextBox: true });
  if (notes) s.addNotes(notes);
  return s;
}
function head(s, text, x, y, w) { s.addText(text, { x, y, w, h: 0.36, fontSize: 14, bold: true, color: BLUE, margin: 0, valign: "middle", isTextBox: true }); }
function bullets(s, items, x, y, w, h, size = 13) {
  const arr = items.map((t, i) => (typeof t === "string" ? { text: t, options: { bullet: { indent: 14 }, breakLine: i < items.length - 1, paraSpaceAfter: 5 } }
                                                         : { text: t.text, options: Object.assign({ bullet: { indent: 14 }, breakLine: i < items.length - 1, paraSpaceAfter: 5 }, t.options) }));
  s.addText(arr, { x, y, w, h, fontSize: size, color: "333333", margin: 0, valign: "top", isTextBox: true });
}
function caption(s, text, sub, x, y, w, h = 0.6) {
  s.addShape(pres.ShapeType.rect, { x, y, w, h, fill: { color: BOX }, line: { color: "D0D3D8", width: 0.75 } });
  s.addText([{ text, options: { bold: false, color: INK, fontSize: 12, breakLine: !!sub } }].concat(sub ? [{ text: sub, options: { color: GREY, fontSize: 10 } }] : []),
    { x, y, w, h, align: "center", valign: "middle", margin: 0.03, isTextBox: true });
}
function note(s, text) { s.addText(text, { x: M, y: 6.95, w: 10.5, h: 0.35, fontSize: 11, bold: true, color: NAVY, margin: 0, valign: "middle", isTextBox: true }); }
function cite(s, text, x, y, w) { s.addText(text, { x, y, w, h: 0.3, fontSize: 9.5, color: GREY, margin: 0, valign: "middle", isTextBox: true }); }
function img(s, path, x, y, w, h) { s.addImage({ path, x, y, w, h, sizing: { type: "contain", w, h } }); }
const fs = require("fs");
function imgOr(s, path, text, x, y, w, h) {            // 文件存在就放图，否则放占位框
  if (fs.existsSync(path)) img(s, path, x, y, w, h); else placeholderBox(s, text, x, y, w, h);
}
function placeholderBox(s, text, x, y, w, h) {
  s.addShape(pres.ShapeType.rect, { x, y, w, h, fill: { color: "F4F5F7" }, line: { color: "B8BCC4", width: 1, dashType: "dash" } });
  s.addText(text, { x, y, w, h, align: "center", valign: "middle", fontSize: 12, color: GREY, isTextBox: true, margin: 0.1 });
}
// 打分圆点：1 = 绿勾，0.5 = 琥珀半圆，0 = 红叉
function chip(s, x, y, d, score) {
  if (score >= 1) {
    s.addShape(pres.ShapeType.ellipse, { x, y, w: d, h: d, fill: { color: GREEN }, line: { color: GREEN } });
    s.addText("✓", { x, y, w: d, h: d, align: "center", valign: "middle", fontSize: d * 36, bold: true, color: "FFFFFF", margin: 0, isTextBox: true });
  } else if (score > 0) {
    s.addShape(pres.ShapeType.ellipse, { x, y, w: d, h: d, fill: { color: "FFFFFF" }, line: { color: AMBER, width: 2 } });
    s.addShape(pres.ShapeType.chord, { x, y, w: d, h: d, fill: { color: AMBER }, line: { color: AMBER }, angleRange: [90, 270] });
  } else {
    s.addShape(pres.ShapeType.ellipse, { x, y, w: d, h: d, fill: { color: RED }, line: { color: RED } });
    s.addText("✗", { x, y, w: d, h: d, align: "center", valign: "middle", fontSize: d * 36, bold: true, color: "FFFFFF", margin: 0, isTextBox: true });
  }
}
const CRIT = ["Dexterous", "No embodiment gap", "Operator feels contact", "Robot in the loop"];
const SCORES = { umi: [0, 1, 1, 0], dexumi: [0.5, 0.5, 1, 0], artglove: [1, 0, 1, 0], mile: [1, 1, 0, 1], ditto: [0.5, 1, 1, 1], dittox: [0.5, 0.5, 1, 1], ours: [1, 1, 1, 0.5] };
// 右上角小卡：2×2
function miniCard(s, key, label, x, y, w = 2.55) {
  const h = 1.0, ours = key === "ours";
  s.addShape(pres.ShapeType.roundRect, { x, y, w, h, rectRadius: 0.08, fill: { color: ours ? TINT : "FFFFFF" }, line: { color: ours ? BLUE : "B8BCC4", width: 1 } });
  s.addText(label, { x, y: y + 0.03, w, h: 0.26, align: "center", fontSize: 10, bold: true, color: ours ? BLUE : INK, margin: 0, isTextBox: true });
  const sc = SCORES[key];
  for (let i = 0; i < 4; i++) {
    const cx = x + 0.12 + (i % 2) * (w / 2), cy = y + 0.33 + Math.floor(i / 2) * 0.32;
    chip(s, cx, cy, 0.24, sc[i]);
    s.addText(`${"①②③④"[i]} ${CRIT[i]}`, { x: cx + 0.28, y: cy - 0.02, w: w / 2 - 0.4, h: 0.28, fontSize: 8, color: "333333", margin: 0, valign: "middle", isTextBox: true });
  }
}
function miniRow(s, key, label, x, y) {
  const w = 3.25, h = 0.42, ours = key === "ours";
  s.addShape(pres.ShapeType.roundRect, { x, y, w, h, rectRadius: 0.06, fill: { color: ours ? TINT : "FFFFFF" }, line: { color: ours ? BLUE : "B8BCC4", width: 1 } });
  s.addText(label, { x: x + 0.1, y, w: 0.95, h, fontSize: 9.5, bold: true, color: ours ? BLUE : INK, margin: 0, valign: "middle", isTextBox: true });
  SCORES[key].forEach((sc, i) => {
    const gx = x + 1.08 + i * 0.53;
    s.addText("①②③④"[i], { x: gx, y, w: 0.2, h, fontSize: 9, color: GREY, margin: 0, valign: "middle", isTextBox: true });
    chip(s, gx + 0.2, y + 0.09, 0.24, sc);
  });
}
function statTile(s, big, small, x, y, w, h = 0.95) {
  s.addShape(pres.ShapeType.rect, { x, y, w, h, fill: { color: TINT }, line: { color: TINT } });
  s.addText(big, { x, y: y + 0.06, w, h: h * 0.5, align: "center", valign: "middle", fontSize: 22, bold: true, color: NAVY, margin: 0, isTextBox: true });
  s.addText(small, { x: x + 0.08, y: y + h * 0.52, w: w - 0.16, h: h * 0.44, align: "center", valign: "top", fontSize: 9.5, color: "333333", margin: 0, isTextBox: true });
}
function arrow(s, x1, y1, x2, y2, color = NAVY) {
  s.addShape(pres.ShapeType.line, { x: Math.min(x1, x2), y: Math.min(y1, y2), w: Math.abs(x2 - x1) || 0.01, h: Math.abs(y2 - y1) || 0.01,
    line: { color, width: 1.75, endArrowType: "triangle" }, flipH: x2 < x1, flipV: y2 < y1 });
}
function box(s, title, lines, x, y, w, h, color) {
  s.addShape(pres.ShapeType.roundRect, { x, y, w, h, rectRadius: 0.1, fill: { color: "FFFFFF" }, line: { color, width: 1.75 } });
  s.addText(title, { x, y: y + 0.06, w, h: 0.32, align: "center", fontSize: 12, bold: true, color, margin: 0, isTextBox: true });
  s.addText(lines.map((t, i) => ({ text: t, options: { breakLine: i < lines.length - 1 } })),
    { x: x + 0.1, y: y + 0.4, w: w - 0.2, h: h - 0.45, align: "center", valign: "top", fontSize: 9.5, color: "333333", margin: 0, paraSpaceAfter: 2, isTextBox: true });
}

// ================= P1 标题 =================
{
  const s = pres.addSlide({ masterName: "TITLE" }); n += 1;
  s.addText("LAB MEETING · 15 OCTOBER 2026", { x: M, y: 0.55, w: 9, h: 0.4, fontSize: 16, bold: true, color: NAVY, margin: 0, isTextBox: true });
  s.addText("An Isomorphic Force-Feedback Glove–Hand Pair for Dexterous Teleoperation", { x: M, y: 0.95, w: 12.4, h: 1.3, fontSize: 32, bold: true, color: INK, margin: 0, valign: "top", isTextBox: true });
  s.addText("13 motor joints, joint-to-joint bilateral control, and what it lets us record for policy learning", { x: M, y: 2.25, w: 12, h: 0.5, fontSize: 18, color: GREY, margin: 0, isTextBox: true });
  s.addText("Pengfei Ye", { x: M, y: 3.05, w: 6, h: 0.4, fontSize: 20, bold: true, color: INK, margin: 0, isTextBox: true });
  s.addText("HKUST / MIT CSAIL · ongoing work with E. Adelson", { x: M, y: 3.45, w: 8, h: 0.4, fontSize: 16, color: INK, margin: 0, isTextBox: true });
  img(s, FIG + "cad_palm_oblique.png", M, 4.1, 4.0, 3.0);
  img(s, FIGS + "skeleton_zero_palm_front.png", 4.67, 4.1, 4.0, 3.0);
  img(s, FIG + "sim_grasp_hold.png", 8.88, 4.1, 4.0, 3.0);
  s.addNotes("一句话：一只按人手设计的手套，和一只完全相同的机械手，关节一一对应、带力反馈。今天讲三件事：为什么要机器人在环、我们做了什么、记录下来的力和刚度怎么用。");
}

// ================= P2 state vs action + 打分卡 =================
{
  const s = slide("Motivation", "Handheld devices record what the human did, not what the robot must do",
    "UMI-style collection has no motor in the loop: gripper width stops at contact, grasp force is never recorded",
    "全片的论点。无本体采集记录的是人在自己闭环下的运动学轨迹（state）；策略部署需要的是机器人本体的动作（action）：力、柔顺、执行器动力学、可行性，这些只有机器人在环时才会被记录。证据：DITTO 手持数据在\"抓住并抬起\"阶段 0%，加 10 条遥操作数据升到 70%；ForceMimic 把力加进动作相对提升 54.5%；ACP（UMI 作者同组）指出位置控制策略忽略柔顺。由此引出四条要求，后面每篇论文都用这张卡打分。");
  head(s, "STATE IS NOT ACTION", M, 2.05, 5.6);
  bullets(s, [
    "A policy needs robot actions: force, compliance, actuator dynamics, feasibility",
    "Human-held demos capture the human's closed loop, not the robot's actuators",
    { text: "DITTO: handheld-only data → 0 % at \"hold and lift\"; +10 teleop episodes → 70 %", options: { bold: true } },
    "ForceMimic: force in the action → +54.5 % relative over vision-only",
    "ACP (UMI's own group): compliance is ignored by position-only visuomotor policies",
    "UMI: kinematic feasibility unknown at collection time, filtered afterwards",
  ], M, 2.45, 5.7, 2.6, 12.5);
  statTile(s, "0 % → 70 %", "DITTO: handheld only vs +10 teleop episodes (hold & lift)", M, 5.2, 1.8);
  statTile(s, "+54.5 %", "ForceMimic: force as action vs vision-only", M + 1.95, 5.2, 1.8);
  statTile(s, "13 × 220 Hz", "what our pair logs: q*, q, τ, I, K per joint", M + 3.9, 5.2, 1.8);
  // 打分卡（右侧）
  head(s, "FOUR REQUIREMENTS FOR GOOD DEXTEROUS DATA", 6.55, 2.05, 6.4);
  const sys = [["UMI", "umi"], ["DexUMI", "dexumi"], ["ART-Glove", "artglove"], ["MILE", "mile"], ["DITTO", "ditto"], ["DITTO-X", "dittox"], ["Ours", "ours"]];
  const x0 = 8.65, cw = 0.6, y0 = 2.55, rh = 0.78;
  sys.forEach(([name, key], j) => {
    const cx = x0 + j * cw;
    s.addShape(pres.ShapeType.rect, { x: cx + 0.02, y: y0, w: cw - 0.04, h: 0.42, fill: { color: key === "ours" ? BLUE : BOX }, line: { color: key === "ours" ? BLUE : BOX } });
    s.addText(name, { x: cx, y: y0, w: cw, h: 0.42, align: "center", valign: "middle", fontSize: 7.5, bold: true, color: key === "ours" ? "FFFFFF" : INK, margin: 0, isTextBox: true });
  });
  const desc = ["enough independent DoF", "human motion runs 1:1 on the robot", "real or rendered force", "takeover, DAgger, actuator dynamics"];
  CRIT.forEach((c, i) => {
    const cy = y0 + 0.5 + i * rh;
    if (i % 2 === 0) s.addShape(pres.ShapeType.rect, { x: 6.55, y: cy - 0.04, w: 6.35, h: rh - 0.04, fill: { color: "F7F8FA" }, line: { color: "F7F8FA" } });
    s.addText([{ text: `${"①②③④"[i]} ${c}`, options: { bold: true, fontSize: 10.5, color: INK, breakLine: true } }, { text: desc[i], options: { fontSize: 8.5, color: GREY } }],
      { x: 6.62, y: cy, w: 2.1, h: rh - 0.1, valign: "middle", margin: 0, isTextBox: true });
    sys.forEach(([name, key], j) => chip(s, x0 + j * cw + cw / 2 - 0.15, cy + 0.17, 0.3, SCORES[key][i]));
  });
  note(s, "The scorecard returns on every paper slide and on our limitations slide");
}


// ================= P3 时间线：数据手套四十年 =================
{
  const s = slide("Motivation", "Forty years of data gloves: from input devices to linkages, motors and tactile skins",
    "Top: teleoperation (robot in the loop). Bottom: demonstration capture (no robot). Colour = what the glove carries",
    "两条轨：上面是遥操作（机器人在环），下面是演示采集（没有机器人）。颜色是手套带了什么：只有运动传感、带触觉皮肤、带力/触觉反馈、以及按某只机械手设计的外骨骼。这页只讲一句：两条轨到 2025–2026 年在外骨骼上汇合了，MILE 和 DITTO 已经做了同构的主从对，MILE 没有电机、DITTO 只有两根手指，我们在它们中间：四指、有电机。条目都核实过（arXiv 号在大纲里）。");
  const CAT = { motion: ["2B6CB0", "EBF4FF"], tactile: ["2F855A", "F0FFF4"], haptic: ["C05621", "FFFAF0"], exo: ["6B46C1", "FAF5FF"], ours: ["0B3C6E", "0B3C6E"] };
  const chipT = (x, y, w, name, inst, cat) => {
    const [ec, fc] = CAT[cat];
    s.addShape(pres.ShapeType.roundRect, { x, y, w, h: 0.32, rectRadius: 0.06, fill: { color: fc }, line: { color: ec, width: 1.25 } });
    s.addText([{ text: name, options: { bold: true, color: cat === "ours" ? "FFFFFF" : INK, fontSize: 9.5 } },
               { text: inst ? "  " + inst : "", options: { color: cat === "ours" ? "FFFFFF" : GREY, fontSize: 8 } }],
      { x: x + 0.08, y, w: w - 0.12, h: 0.32, valign: "middle", margin: 0, isTextBox: true });
  };
  const era = (x, y, w, h, label, items) => {
    s.addShape(pres.ShapeType.roundRect, { x, y, w, h, rectRadius: 0.1, fill: { color: "F7F8FA" }, line: { color: "D0D3D8", width: 1 } });
    s.addText(label, { x: x + 0.15, y: y - 0.16, w: 1.5, h: 0.3, fontSize: 10, bold: true, color: INK, fill: { color: "E2E8F0" }, align: "center", valign: "middle", margin: 0, isTextBox: true });
    items.forEach((it, i) => chipT(x + 0.15, y + 0.27 + i * 0.38, w - 0.3, ...it));
  };
  const xs = [0.45, 3.59, 6.73, 9.87], w = 3.0;
  const top = [
    ["1988–2002", [["Dexterous HandMaster", "A. D. Little, 1988", "motion"], ["CyberGrasp", "Virtual Technologies, 1999", "haptic"], ["Rutgers Master II", "Rutgers, 2002", "haptic"]]],
    ["2022–2024", [["NimbRo Avatar", "Bonn", "haptic"], ["iCub3 avatar", "IIT", "haptic"], ["BiDex", "CMU, 2024", "motion"]]],
    ["2025", [["DOGlove", "Tsinghua", "haptic"], ["HOMIE", "Shanghai AI Lab", "motion"], ["GEX", "11-DoF hand + 12-DoF exo", "exo"], ["MILE", "SJTU · isomorphic, passive", "exo"]]],
    ["2026", [["DITTO / DITTO-X", "Columbia · 7 DoF, force feedback", "haptic"], ["TAG (Feel Robot Feels)", "tactile arrays", "haptic"], ["Our glove", "isomorphic, motors in every joint", "ours"]]],
  ];
  const bot = [
    ["1977–1990", [["Sayre Glove", "UIC, 1977", "motion"], ["DataGlove", "VPL, 1987", "motion"], ["CyberGlove", "1990", "motion"]]],
    ["2019–2024", [["STAG", "MIT, 2019", "tactile"], ["DexCap", "Stanford, 2024", "motion"]]],
    ["2025", [["DexUMI", "Stanford", "exo"], ["ExoStart", "DeepMind · CMU · MIT", "exo"], ["DEXOP", "MIT", "exo"], ["OSMO", "Meta FAIR", "tactile"]]],
    ["2026", [["ART-Glove", "CMU · 22 DoF, 2048 taxels", "tactile"], ["DexEXO", "UCLA", "exo"], ["SEED-UMI", "PKU · shared exoskeleton", "exo"]]],
  ];
  top.forEach(([lab, items], i) => era(xs[i], 2.3, w, 1.85, lab, items));
  s.addShape(pres.ShapeType.line, { x: 0.45, y: 4.42, w: 12.43, h: 0, line: { color: "A0AEC0", width: 2 } });
  s.addText("▲ Teleoperation (robot in the loop)", { x: 0.45, y: 4.22, w: 4, h: 0.22, fontSize: 9.5, bold: true, color: NAVY, margin: 0, isTextBox: true });
  s.addText("▼ Demonstration capture (no robot)", { x: 8.88, y: 4.46, w: 4, h: 0.22, fontSize: 9.5, bold: true, color: NAVY, align: "right", margin: 0, isTextBox: true });
  bot.forEach(([lab, items], i) => era(xs[i], 4.9, w, 1.85, lab, items));
  const leg = [["Motion sensors", "motion"], ["Tactile skin", "tactile"], ["Haptic / force feedback", "haptic"], ["Exoskeleton matched to a robot hand", "exo"], ["Ours", "ours"]];
  let lx = 0.45;
  leg.forEach(([t, c]) => {
    s.addShape(pres.ShapeType.roundRect, { x: lx, y: 6.86, w: 0.3, h: 0.16, rectRadius: 0.04, fill: { color: CAT[c][1] }, line: { color: CAT[c][0], width: 1.25 } });
    s.addText(t, { x: lx + 0.36, y: 6.8, w: 2.6, h: 0.28, fontSize: 9, color: "333333", margin: 0, valign: "middle", isTextBox: true });
    lx += 0.36 + t.length * 0.075 + 0.35;
  });
}

// ================= P3 UMI =================
{
  const s = slide("Prior Work · UMI", "UMI: make the collection device identical to the robot's end-effector",
    "Same soft gripper on both sides, a wrist camera for observation, and the data deploys across robots",
    "UMI 的贡献是让无本体数据能直接训练出可部署的策略，而不是\"开创了无机器人采集\"（2020 年就有 Grasping in the Wild）。做法：采集设备和机器人末端长得一样，腕部 155° 鱼眼相机加侧面镜子，延迟匹配加相对轨迹动作，所以数据能跨机器人部署。局限：平行夹爪；只采 state；采集时不知道机器人运动学是否可行，只能事后过滤。过渡：夹爪做不了灵巧操作，换成手会怎样？", 9.1);
  img(s, PAP + "umi_teaser.png", M, 2.05, 8.0, 2.63);
  cite(s, "Chi et al., \"Universal Manipulation Interface,\" arXiv:2402.10329, Fig. 1", M, 4.7, 8);
  head(s, "INSIGHT", M, 5.05, 4);
  bullets(s, ["Identical soft gripper on the handheld device and the robot", "Wrist GoPro, 155° fisheye + side mirrors: observation aligned with action",
              "Latency matching + relative-trajectory actions → cross-robot deployment"], M, 5.42, 4.1, 1.4, 11.5);
  head(s, "LIMITATIONS", 4.75, 5.05, 3.8);
  bullets(s, ["Parallel gripper only: no dexterity", "State only: grasp force never recorded", "Kinematic feasibility unknown at collection time"], 4.75, 5.42, 3.8, 1.4, 11.5);
  miniRow(s, "umi", "UMI", 9.65, 0.2);
  statTile(s, "$73 + $298", "printed parts + GoPro kit", 8.9, 2.05, 2.0, 0.9);
  statTile(s, "3×", "faster than SpaceMouse teleop", 11.0, 2.05, 1.9, 0.9);
  statTile(s, "71.7 %", "in-the-wild cup task, 30 locations, 1400 demos", 8.9, 3.05, 4.0, 0.9);
  img(s, PAP + "umi_hardware.png", 8.9, 4.1, 4.0, 1.18);
  cite(s, "UMI hardware (Fig. 2): soft gripper, fisheye camera, mirrors", 8.9, 5.3, 4.0);
  note(s, "Transition: a gripper cannot do dexterous tasks. What if we replace it with a hand?");
}

// ================= P4 DexUMI vs ART-Glove =================
{
  const s = slide("Exoskeletons", "DexUMI fits the exoskeleton to a robot hand; ART-Glove fits it to the human hand",
    "Neither has the robot in the loop: the human, not the actuator, bears the contact",
    "左：DexUMI 是\"机器人优先\"，外骨骼按商用手的指尖工作空间优化，用编码器读关节角，再用 SAM2 和 ProPainter 把机器人手贴回图像，平均成功率 86%。但每只手要单独调参，人手力比机器人大所以触觉读数会漂移，渲染仍需真机。右：ART-Glove 是\"人手优先\"，22 个解剖关节全部编码器测量，2048 触点，但没有机器人、没有策略结果；作者未来工作要做\"按手套几何的驱动手\"，正是我们的路线。", 9.1);
  head(s, "DEXUMI: ROBOT-FIRST", M, 2.05, 6);
  img(s, PAP + "dexumi_teaser.png", M, 2.42, 6.0, 2.47);
  cite(s, "Xu et al., \"DexUMI,\" arXiv:2505.21864, Fig. 1", M, 4.9, 6);
  bullets(s, ["Hardware adaptation: exoskeleton optimized to the robot's fingertip workspace, joint encoders",
              "Software adaptation: segment the human hand (SAM2), inpaint, render the robot hand back",
              { text: "86 % average success on Inspire (6 active) and XHand (12 active)", options: { bold: true } },
              "Authors' limits: per-hand tuning; human force > robot force so tactile drifts; fixed camera"], M, 5.2, 6.0, 1.7, 10.5);
  head(s, "ART-GLOVE: HUMAN-FIRST", 6.9, 2.05, 6);
  img(s, PAP + "artglove_teaser.png", 6.9, 2.42, 6.0, 2.33);
  cite(s, "Lin & Zhao, \"ART-Glove,\" arXiv:2606.16370, Fig. 1", 6.9, 4.75, 6);
  bullets(s, ["16 rigid plates, 22 encoder-measured joints aligned to anatomical joints, 2048 taxels at 120 Hz",
              "Kapandji 10/10: the hand is almost unconstrained",
              "Measurement only: no robot, no policy; normal force only, one size",
              { text: "Future work in the paper: an actuated hand built on the glove geometry — our direction", options: { bold: true } }], 6.9, 5.05, 6.0, 1.85, 10.5);
  miniRow(s, "dexumi", "DexUMI", 9.65, 0.1);
  miniRow(s, "artglove", "ART-Glove", 9.65, 0.58);
  note(s, "Real contact in both, but the human feels the object; the robot never does");
}

// ================= P5 DITTO → DITTO-X =================
{
  const s = slide("DITTO / DITTO-X", "DITTO closes the loop with an equivalent leader; DITTO-X trades isomorphism for generality",
    "The closest work to ours: joint-level force feedback from follower current, then one exoskeleton for three hands",
    "DITTO：主从运动学等价，拇指 4 加食指 3 共 7 自由度，1:1 电机映射，用从手电流算关节级力反馈，200 Hz；有力反馈时用户接触力约减半（6.9 N 对 13.0 N）；手持数据 0% 对加 10 条遥操作 70%。DITTO-X：为通用性放弃同构，一个外骨骼驱动 Sharpa、Wuji、Inspire；食指中指 1:1，拇指任务空间重定向，力用雅可比伪逆估计；力反馈加振动才有 86.1% 的物体大小辨识率，去掉任一通道 61.1%；反向遥操作接管 78.3% 对 Manus 27.7%。老师读过这篇，我们的差异必须讲成：同构做到 4 指 13 自由度、拇指完整、主从一起设计。", 9.1);
  head(s, "DITTO (2026-09): 1:1 KINEMATICS, 7 DoF", M, 2.05, 6);
  img(s, PAP + "ditto_teaser.png", M, 2.42, 6.0, 2.2);
  cite(s, "DITTO, arXiv:2609.19196, Fig. 1", M, 4.63, 6);
  bullets(s, ["Leader and follower kinematically equivalent: thumb 4 + index 3, 1:1 motor mapping",
              "Joint-level force feedback from follower current, 200 Hz",
              { text: "With feedback, users apply about half the contact force", options: { bold: true } }], M, 4.93, 6.0, 0.9, 10.5);
  head(s, "DITTO-X (2026-10): ONE EXOSKELETON, THREE HANDS", 6.9, 2.05, 6);
  img(s, PAP + "dittox_teaser.png", 6.9, 2.42, 6.0, 1.75);
  cite(s, "DITTO-X, arXiv:2610.00781, Fig. 1", 6.9, 4.18, 6);
  bullets(s, ["Index and middle 1:1; thumb retargeted in task space, force via Jacobian pseudo-inverse; ring and pinky coupled",
              "Force feedback + fingertip vibration are complementary",
              "Reverse teleoperation: take over a running policy; DAgger improves over two rounds"], 6.9, 4.5, 6.0, 1.3, 10.5);
  statTile(s, "86.1 % vs 61.1 %", "size discrimination, both channels vs either alone", 6.9, 5.85, 2.9, 0.85);
  statTile(s, "78.3 % vs 27.7 %", "takeover success, DITTO-X vs Manus glove", 9.95, 5.85, 2.95, 0.85);
  miniRow(s, "ditto", "DITTO", 9.65, 0.1);
  miniRow(s, "dittox", "DITTO-X", 9.65, 0.58);
  statTile(s, "0 % → 70 %", "handheld-only data vs +10 teleop episodes at hold & lift", M, 5.85, 2.9, 0.85);
  statTile(s, "6.9 N vs 13.0 N", "contact force with vs without force feedback", 3.55, 5.85, 2.9, 0.85);
  note(s, "Generality cost isomorphism. We take the other end of that trade-off");
}

// ================= P6 定位 + 对比表 =================
{
  const s = slide("Our Position", "Design the glove around the human hand, then derive an identical robot hand",
    "Human-first and fully isomorphic: every one of the 13 joints maps 1:1, so force, takeover and data need no retargeting",
    "对比表后，讲通用性与保真度的取舍轴：DITTO-X 在通用那端，我们在同构那端。同构的四个好处：力反馈关节对关节、不用估计不用重定向；反向遥操作直接写关节角、不用逆运动学；仿真孪生就是机器人本身；采到的数据不需要重定向。代价：机械手要自己造，机器人手运动学被人手约束。类比 GELLO、HOMIE 的同构主臂。最接近的工作是 MILE（上海交大，2025-11）：先按人手设计 17 自由度的外骨骼，再按外骨骼做机械手，关节一一对应、不用重定向，指尖有视触觉，四任务 76% 对手套基线 28%。但 MILE 的外骨骼是纯编码器的、被动的，论文自己写了\"不向操作者提供任何主动力或触觉反馈\"，并把它列为未来工作。我们的手套里有电机，所以多了双边力反馈，而且刚度可以当动作。");
  const hdr = ["", "UMI", "DexUMI", "ART-Glove", "MILE", "DITTO", "DITTO-X", "Ours"];
  const rows = [
    ["Dexterity", "parallel gripper", "Inspire 6 active / XHand 12", "22 DoF measured", "4 fingers, 17 DoF", "2 fingers, 7 DoF", "3 fingers (~9 motors)", "4 fingers, 13 DoF"],
    ["Correspondence to the robot", "identical gripper", "fingertips only", "none (retargeting)", "1:1, exoskeleton first", "1:1", "index, middle 1:1; thumb retargeted", "all 13 joints 1:1"],
    ["What the operator feels", "real contact", "real contact", "real contact", "nothing (tactile on screen)", "rendered joint force", "joint force + fingertip vibration", "rendered joint force"],
    ["Robot in the loop", "no", "no", "no", "yes", "yes", "yes, reverse teleop", "yes (sim / glove #2; robot hand next)"],
    ["Policy results", "yes", "yes, 86 % avg.", "none", "yes, 76 % avg.", "yes", "yes, with DAgger", "not yet"],
  ];
  const OURS = 7;
  const tbl = [hdr.map((h, j) => ({ text: h, options: { bold: true, fontSize: 10, color: j === OURS ? "FFFFFF" : INK, fill: { color: j === OURS ? BLUE : BOX }, align: "center", valign: "middle" } }))]
    .concat(rows.map((r, i) => r.map((c, j) => ({ text: c, options: { fontSize: 9, bold: j === 0, color: INK, align: j === 0 ? "left" : "center", valign: "middle",
      fill: { color: j === OURS ? TINT : (i % 2 === 0 ? "F7F8FA" : "FFFFFF") } } }))));
  s.addTable(tbl, { x: M, y: 2.05, w: 12.43, colW: [1.7, 1.3, 1.6, 1.35, 1.45, 1.3, 1.85, 1.88], rowH: 0.46, border: { type: "solid", color: "D0D3D8", pt: 0.5 }, margin: 0.04 });
  // 取舍轴
  head(s, "GENERALITY ⟷ FIDELITY", M, 5.05, 5);
  s.addShape(pres.ShapeType.line, { x: 0.8, y: 5.95, w: 4.9, h: 0, line: { color: GREY, width: 1.5, beginArrowType: "triangle", endArrowType: "triangle" } });
  s.addShape(pres.ShapeType.ellipse, { x: 1.55, y: 5.84, w: 0.22, h: 0.22, fill: { color: GREY }, line: { color: GREY } });
  s.addShape(pres.ShapeType.ellipse, { x: 4.85, y: 5.84, w: 0.22, h: 0.22, fill: { color: BLUE }, line: { color: BLUE } });
  s.addText("DITTO-X\none exoskeleton, many hands", { x: 0.75, y: 6.12, w: 1.9, h: 0.55, fontSize: 9.5, color: GREY, align: "center", margin: 0, isTextBox: true });
  s.addText("Ours\none pair, all 13 joints 1:1", { x: 4.05, y: 6.12, w: 1.9, h: 0.55, fontSize: 9.5, bold: true, color: BLUE, align: "center", margin: 0, isTextBox: true });
  s.addText("generality", { x: 0.8, y: 5.55, w: 1.5, h: 0.28, fontSize: 9.5, color: GREY, margin: 0, isTextBox: true });
  s.addText("fidelity", { x: 4.7, y: 5.55, w: 1.0, h: 0.28, fontSize: 9.5, color: GREY, align: "right", margin: 0, isTextBox: true });
  head(s, "WHAT ISOMORPHISM BUYS", 6.3, 5.05, 6.5);
  bullets(s, ["Force feedback joint-to-joint: no estimation, no retargeting", "Reverse teleop is exact: write joint angles, no inverse kinematics",
              "The simulation twin is the robot itself; recorded data needs no retargeting",
              { text: "Closest pair: MILE (SJTU 2025) also builds the hand from a 17-DoF exoskeleton, but the exoskeleton is passive: no force to the operator. Ours puts motors in the glove → bilateral feedback, stiffness as action", options: { bold: true } }],
          6.3, 5.42, 6.6, 1.5, 10.5);
  note(s, "Same idea as isomorphic leader arms (GELLO, HOMIE), taken to a 13-joint hand. Cost: we must build the hand ourselves");
}


// ================= P7a 为什么手套难做 =================
{
  const s = slide("Design", "Why a glove is hard to wear: the hand leaves little room where the glove needs it most",
    "Four constraints that shaped the 13-joint layout",
    "四个约束：关节轴藏在手掌里（MCP 轴在皮肤下面，电机放不到轴上）；拇指扫过的范围很大（从张开到对掌，要 4 个关节）；手指之间没有空间（三个模块并排，连杆和电机不能互相碰）；手的尺寸差别大（DexEXO 按 140–217 mm 的手长设计；我们现在只有一个尺寸，这是局限）。");
  const panels = [
    [FIG + "rcm_cropped.png", "Joint axes are hidden", "The MCP axis lies under the skin, so the motor cannot sit on it. A double parallelogram puts a virtual axis there while the motor stays outside the finger", "Our middle-finger MCP, 0° / 30° / 60° overlay"],
    [FIG + "rom_overlay_thumb_side.png", "The thumb sweeps wide", "From open to opposition the thumb needs abduction, flexion and two more joints: four motors on the thumb alone, mounted to clear the palm", "Our CAD, open / half / fist overlay"],
    [FIG + "modules_cropped.png", "No room between fingers", "Three finger modules sit side by side on the dorsal base; linkages and motors must clear their neighbours through the whole range", "Our CAD, one colour per finger module"],
    [FIG + "cad_dorsal.png", "Hands differ in size", "DexEXO models hand lengths of 140–217 mm. Our glove is one size today; sizing is on the limitations slide", "DexEXO, arXiv 2603.17323"],
  ];
  panels.forEach(([pth, h1, body, c], i) => {
    const x = 0.45 + i * 3.15;
    s.addShape(pres.ShapeType.rect, { x, y: 2.05, w: 3.0, h: 2.55, fill: { color: "F7F8FA" }, line: { color: "F7F8FA" } });
    img(s, pth, x + 0.1, 2.12, 2.8, 2.4);
    s.addText(h1, { x, y: 4.72, w: 3.0, h: 0.36, fontSize: 13, bold: true, color: INK, margin: 0, valign: "middle", isTextBox: true });
    s.addText(body, { x, y: 5.1, w: 3.0, h: 1.2, fontSize: 10.5, color: "333333", margin: 0, valign: "top", isTextBox: true });
    cite(s, c, x, 6.4, 3.0);
  });
  note(s, "Every one of these shows up again on the next slide as a design decision");
}

// ================= P7 设计 =================
{
  const s = slide("Design", "13 joints, four fingers, every axis where the human joint is",
    "Thumb 4 + index / middle / ring 3 each; a double-parallelogram MCP puts the middle-finger axis inside the finger",
    "布局：拇指 4（CMC 侧摆、CMC 弯曲、MCP 弯曲、IP），食指、中指、无名指各 3（MCP 弯曲、MCP 侧摆、PIP），没有小指。中指 MCP 用双平行四边形做出虚拟转轴，电机不在转轴上，这是\"按人手关节轴设计\"的证据。远端 4 个关节用 77.5:1 的 XL330-M077 便于反拖，其余 288:1。OpenRB-150，1 Mbps，6 V，约 220 Hz。");
  img(s, FIGS + "skeleton_zero_palm_front_nomesh.png", M, 1.95, 5.6, 4.2);
  caption(s, "Skeleton of the glove: red = flexion axes, blue = abduction / opposition, numbers = motor IDs", "", M + 0.4, 6.15, 4.8, 0.55);
  img(s, FIG + "rcm_cropped.png", 6.3, 1.95, 3.5, 2.62);
  caption(s, "Middle-finger MCP: 0° / 30° / 60° overlay, the red virtual axis never moves", "", 6.3, 4.6, 3.5, 0.55);
  img(s, FIG + "modules_cropped.png", 9.95, 1.95, 3.0, 2.25);
  caption(s, "Thumb ID 1–4 · index 5–7 · middle 8–10 · ring 11–13", "", 9.95, 4.25, 3.0, 0.45);
  head(s, "ACTUATION AND ELECTRONICS", 6.3, 5.3, 6.6);
  bullets(s, ["13 × DYNAMIXEL XL330; distal PIP / IP joints use 77.5:1 (easier to back-drive), others 288:1",
              "OpenRB-150 bridge, 1 Mbps bus, 6 V pack, all 13 motors read and written at ≈ 220 Hz",
              "Onshape → URDF / MuJoCo export is automated, closed chains included"], 6.3, 5.68, 6.6, 1.2, 11);
  note(s, "Human-first: the glove is designed on the human hand, the robot hand is derived from it");
}

// ================= P8 进展 + 演示 =================
{
  const s = slide("Progress", "It works: sim-in-the-loop force feedback and a two-glove bilateral pair",
    "Position goes forward, force comes back, only in contact and only against penetration",
    "先放视频再讲参数。链路：Onshape 自动导出 URDF 和 MuJoCo，13 个电机标定完成。控制律：e = q_leader − q_follower，τ = −kp(|e| − 死区)·sign(e)，kp 0.4 N·m/rad，死区 ±2°，单关节上限 0.06 N·m（DITTO 0.08），只在接触时出力、只往外推，13 路电流总和有上限，约 220 Hz（DITTO 200 Hz）。视频 1：手套驱动 MuJoCo 虚拟手抓积木，碰到积木时人捏不下去，虚拟手抓起 7.8 cm。视频 2：两只手套，从手被挡住，主手被往回推。");
  // 流程框
  const by = 2.05, bh = 0.95;
  box(s, "Leader glove (worn)", ["13 × XL330, current mode", "renders feedback torque"], M, by, 2.6, bh, BLUE);
  box(s, "PC ≈ 220 Hz", ["MuJoCo twin or glove #2", "τ = −kp(|e| − db)·sign(e)"], 3.85, by, 2.6, bh, GREEN);
  box(s, "Follower hand", ["position mode + current limit", "blocked → error e grows"], 7.25, by, 2.6, bh, RED);
  const lab = (t, x, y, c) => s.addText(t, { x, y, w: 0.8, h: 0.22, fontSize: 8.5, bold: true, color: c, align: "center", margin: 0, isTextBox: true });
  arrow(s, 3.05, by + 0.3, 3.85, by + 0.3, BLUE); lab("q_L", 3.05, by + 0.04, BLUE);
  arrow(s, 6.45, by + 0.3, 7.25, by + 0.3, BLUE); lab("q* = q_L", 6.45, by + 0.04, BLUE);
  arrow(s, 7.25, by + 0.66, 6.45, by + 0.66, RED); lab("q_F, I_F", 6.45, by + 0.7, RED);
  arrow(s, 3.85, by + 0.66, 3.05, by + 0.66, RED); lab("I_L ← τ", 3.05, by + 0.7, RED);
  // 控制律图（原生折线图）
  const e = [], tau = [];
  for (let k = -30; k <= 30; k += 1) { e.push(k % 10 === 0 ? String(k) : ""); const t = Math.abs(k) > 2 ? -0.4 * (Math.abs(k) - 2) * Math.sign(k) * Math.PI / 180 * 1000 : 0; tau.push(Math.max(-60, Math.min(60, t))); }
  s.addChart(pres.ChartType.line, [{ name: "Feedback torque", labels: e, values: tau }], {
    x: M, y: 3.2, w: 5.6, h: 3.6, chartColors: [RED], lineSize: 2.5, lineDataSymbol: "none",
    showTitle: true, title: "Feedback torque vs position error (kp 0.4 N·m/rad, deadband ±2°, limit 0.06 N·m)", titleFontSize: 10, titleColor: INK, titleFontFace: "+mn-lt",
    showLegend: false, catAxisTitle: "e = q_leader − q_follower (deg)", showCatAxisTitle: true, catAxisTitleFontSize: 9, catAxisTitleColor: GREY, catAxisTitleFontFace: "+mn-lt",
    valAxisTitle: "torque (mN·m)", showValAxisTitle: true, valAxisTitleFontSize: 9, valAxisTitleColor: GREY, valAxisTitleFontFace: "+mn-lt",
    catAxisLabelFontSize: 8, catAxisLabelColor: GREY, catAxisLabelFontFace: "+mn-lt", valAxisLabelFontSize: 8, valAxisLabelColor: GREY, valAxisLabelFontFace: "+mn-lt",
    valAxisMinVal: -75, valAxisMaxVal: 75, valAxisMajorUnit: 25, valGridLine: { color: "E3E5E8", size: 0.5 }, catGridLine: { style: "none" },
  });
  // 仿真图 + 视频占位
  img(s, FIG + "sim_grasp_open.png", 6.3, 3.2, 3.2, 2.4);
  img(s, FIG + "sim_grasp_hold.png", 9.7, 3.2, 3.2, 2.4);
  caption(s, "MuJoCo twin driven by the glove: open, then grasp and lift the cube 7.8 cm", "", 6.3, 5.62, 6.6, 0.42);
  imgOr(s, FIGS + "photos/pinch_wide.jpg", "[VIDEO 1] glove drives the MuJoCo hand; the operator feels the cube", 6.3, 6.15, 3.2, 0.7);
  imgOr(s, FIGS + "photos/pinch_close.jpg", "[VIDEO 2] two gloves: follower pinches a cup → leader pushed back", 9.7, 6.15, 3.2, 0.7);
  statTile(s, "≈ 220 Hz", "13 motors read + written (DITTO: 200 Hz)", 10.15, by, 1.35, bh);
  statTile(s, "0.06 N·m", "per-joint limit (DITTO: 0.08)", 11.6, by, 1.3, bh);
  note(s, "Force only in contact, only outward, with a 13-channel current budget and a 100 ms bus watchdog");
}

// ================= P9 记录与方法 1–3 =================
{
  const s = slide("Force & Impedance", "Every cycle we log command, result and force; stiffness itself can be an action",
    "Three ways to turn the recorded force into supervision: observation, action, or impedance",
    "重点页。记录：q*（主手指令）、q（从手结果）、τ（从手电流 × Kt = 接触力矩）、I（渲染给人的电流）、K_t（从手当周期的刚度）。方法 1 力作观测：策略输入多一项 τ，部署时同一只手的电流同样可得。方法 2 力作动作：策略输出 (q*, τ*)，底层混合位置/力控制，Goal Current 当动作。方法 3 阻抗作动作：策略输出 (q*, K_t)，底层阻抗控制 τ = K(q* − q) + D(q̇* − q̇)；标签 K_t = f(τ_t)：力大就软、空中就硬（ACP 规则）；XL330 的 P、D 增益通电可改写，所以 (q*, Kp, Kd, 电流上限) 可整组作为动作。实测（食指 MCP，捏纸杯）：Kp 150 时刚度 0.14 N·m/rad、最大夹持力矩 25 mN·m；Kp 600 时 0.58 N·m/rad、57 mN·m，斜率比 4.1 对 Kp 比 4.0，K ≈ Kp/1030。自适应那段接触电流升到 100 mA 时 Kp 从 600 降到 380，规则跑通。同一个位置指令下机器人出多大力由 K 决定，所以 K 要进动作。");
  const y = 2.05, h = 2.0;
  box(s, "Logged, 13 joints × 220 Hz", ["q*  leader angle = command", "q   follower angle = result", "τ   follower current × Kt = contact torque", "I   current rendered to the operator", "K_t follower stiffness this cycle"], M, y, 3.1, h, BLUE);
  box(s, "① Force as observation", ["policy input += τ", "current is available at deployment · FACTR"], 3.95, y, 2.75, 0.95, GREEN);
  box(s, "② Force as action", ["action chunk = (q*, τ*)", "hybrid position / force control", "Goal Current is the action · Bi-ACT"], 3.95, y + 1.05, 2.75, 0.95, GREEN);
  box(s, "③ Impedance as action", ["action chunk = (q*, K_t)", "τ_cmd = K(q* − q) + D(q̇* − q̇)", "label K_t = f(τ_t): high force → soft (ACP rule)", "XL330 P / D gains writable online", "ACP, Comp-ACT, CompVLA"], 7.1, y, 3.2, h, RED);
  box(s, "Deployment", ["isomorphic robot hand", "same controller", "no retargeting"], 10.7, y, 2.2, h, NAVY);
  arrow(s, 3.55, y + 0.48, 3.95, y + 0.48, GREY); arrow(s, 3.55, y + 1.52, 3.95, y + 1.52, GREY);
  arrow(s, 6.7, y + 1.0, 7.1, y + 1.0, GREY); arrow(s, 10.3, y + 1.0, 10.7, y + 1.0, GREY);
  head(s, "MEASURED ON THE TWO-GLOVE PAIR: INDEX MCP PINCHING A PAPER CUP, Kp 150 vs 600, AND Kp SCHEDULED BY CONTACT CURRENT", M, 4.25, 12.4);
  imgOr(s, FIG + "stiffness_kp_p2.png", "[PLOT] soft vs firm grasp: follower current over time", M, 4.65, 4.05, 2.25);
  imgOr(s, FIG + "stiffness_kp_p3.png", "[PLOT] Kp 150 vs 600: τ-vs-e slope = follower stiffness K", 4.65, 4.65, 4.05, 2.25);
  imgOr(s, FIG + "stiffness_adaptive_p4.png", "[PLOT] K_t = f(τ_t): P gain drops as contact current rises", 8.85, 4.65, 4.05, 2.25);
  note(s, "Same pinch, same q*: 25 mN·m with Kp 150 vs 57 mN·m with Kp 600 — the robot's force is set by K, so K belongs in the action");
}

// ================= P10 局限 / 下一步 / 讨论 =================
{
  const s = slide("Outlook", "What is missing, and what we do next", "The scorecard again, honestly: one half circle, and the plan to fill it",
    "用小卡讲那个半圆：机械手还没造，力反馈来自仿真或第二只手套；没有用户实验和策略结果；没有指尖振动（DITTO-X 消融说两个通道都需要）；没有小指，手套只有一个尺寸；288:1 的反拖阻力要量化（DITTO 特意选 181:1）；6 V 略超 XL330 规格，拇指 MCP 那个 12 V 电机欠压运行。下一步：造机械手 → 量化并补偿反拖 → 方法 1 到 3 逐级验证 → 用户实验 → 反向遥操作和 DAgger。讨论题：减速比怎么选、拇指几个自由度、要不要兼顾非同构的手。");
  head(s, "LIMITATIONS", M, 2.05, 4);
  bullets(s, ["No physical robot hand yet: feedback comes from the MuJoCo twin or glove #2", "No user study, no policy results",
              "No fingertip vibration (DITTO-X ablation: both channels needed)", "No pinky, one glove size",
              "Back-drive friction of 288:1 joints not quantified (DITTO chose 181:1)", "Power: 5-cell NiMH drifts 5.0–7.0 V around the 3.7–6.0 V XL330 spec; one 12 V-class motor runs under-volted"], M, 2.42, 4.0, 3.6, 11.5);
  head(s, "NEXT STEPS", 4.75, 2.05, 4);
  const steps = ["Build the isomorphic robot hand", "Quantify and compensate back-drive friction", "Validate methods ① → ② → ③ on the real pair", "User study: force feedback on / off", "Reverse teleoperation + DAgger"];
  steps.forEach((t, i) => {
    const yy = 2.5 + i * 0.68;
    s.addShape(pres.ShapeType.ellipse, { x: 4.75, y: yy, w: 0.42, h: 0.42, fill: { color: NAVY }, line: { color: NAVY } });
    s.addText(String(i + 1), { x: 4.75, y: yy, w: 0.42, h: 0.42, align: "center", valign: "middle", fontSize: 12, bold: true, color: "FFFFFF", margin: 0, isTextBox: true });
    s.addText(t, { x: 5.3, y: yy, w: 3.5, h: 0.42, valign: "middle", fontSize: 12, color: "333333", margin: 0, isTextBox: true });
  });
  head(s, "QUESTIONS FOR THE GROUP", 9.1, 2.05, 3.8);
  bullets(s, ["Gear ratio: transparency vs torque?", "How many thumb DoF are worth it?", "Should the pair also drive non-isomorphic hands?"], 9.1, 2.42, 3.8, 1.5, 12);
  miniCard(s, "ours", "Ours, today", 9.1, 4.25, 3.8);
  s.addText("④ becomes ✓ the day the robot hand exists", { x: 9.1, y: 5.35, w: 3.8, h: 0.3, fontSize: 10, italic: true, color: GREY, margin: 0, isTextBox: true });
  img(s, FIG + "cad_dorsal.png", 9.1, 5.65, 3.8, 1.25);
  note(s, "Ongoing work with E. Adelson · exploring broader collaboration");
}

(async () => {
  await pres.writeFile({ fileName: OUT });
  await applyTheme(OUT, THEME);
  console.log("wrote", OUT, "slides", n);
})();
