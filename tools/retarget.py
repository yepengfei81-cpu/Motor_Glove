"""
主手关节角 -> 从手目标角 的映射链（bilateral.py 和 glove_live.py 共用）：

    经验比例 (--scale NAME=K)  ->  map_AB (a·q+b, fit_map.py 拟合)  ->  手套->机械手 (符号 × 多项式, compare_kin.py)  ->  限幅

    jm = JointMap(joints, scale=["thumb_mcp_flex_motor=1.3"], map_path="config/map_AB.json", to_hand=True)
    q_follower = jm(q_leader)
"""
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import glove_model as gm  # noqa: E402

G2H_DEFAULT = os.path.join(gm.ROOT, "config", "glove_to_hand.json")


class JointMap:
    def __init__(self, joints, scale=(), map_path=None, to_hand=False, g2h_path=None, range_deg=110.0):
        self.joints = list(joints)
        self.scale, self.qmap, self.g2h = {}, {}, {}
        for item in scale:
            name, _, k = item.partition("=")
            if name not in self.joints:
                print(f"!! --scale {item}: 没有这个关节（或没在同步）")
                continue
            self.scale[name] = float(k)
        if map_path:
            self.qmap = {j: v for j, v in json.load(open(map_path)).items() if j in self.joints}
        if to_hand:
            path = g2h_path or G2H_DEFAULT
            self.g2h = {j: v for j, v in json.load(open(path)).items() if j in self.joints}
        self.rng = math.radians(range_deg)
        self.describe(map_path, g2h_path or G2H_DEFAULT if to_hand else None)

    def describe(self, map_path, g2h_path):
        if self.scale:
            print("* 主手关节角经验比例：" + "，".join(f"{j.replace('_motor', '')} ×{k:.2f}" for j, k in self.scale.items()))
        if self.qmap:
            print(f"* 关节映射 {os.path.relpath(map_path)}：" + "，".join(
                f"{j.replace('_motor', '')} a={v['a']:.2f} b={math.degrees(v['b']):+.1f}°" for j, v in self.qmap.items()))
        if self.g2h:
            odd = [f"{j.replace('_motor', '')} {'-' if v['sign'] < 0 else '+'}{v['ratio']:.2f}θ" for j, v in self.g2h.items()
                   if v["sign"] < 0 or abs(v["ratio"] - 1) > 0.01]
            print(f"* 手套->机械手映射 {os.path.relpath(g2h_path)}：" + "，".join(odd) + "，其余 1:1")

    def direction(self, j):
        """d(从手角)/d(主手角) 的符号：力反馈从从手空间换回主手空间要乘它（映射里有符号相反的关节，如拇指 IP、中指 PIP）"""
        d = 1.0
        if self.scale.get(j, 1.0) < 0:
            d = -d
        if j in self.qmap and self.qmap[j]["a"] < 0:
            d = -d
        if j in self.g2h and self.g2h[j]["sign"] < 0:
            d = -d
        return d

    def hand_to_glove(self, q):
        """只做手套->机械手那一段的反变换（符号 / 传动比，线性近似），fit_map.py 用来在手套空间里拟合 a·q+b"""
        out = dict(q)
        for j, v in self.g2h.items():
            if j in out:
                out[j] = v["sign"] * out[j] / v["ratio"]
        return out

    def inverse(self, q):
        """从手角 -> 主手角（整条链反过来：手套->机械手用线性近似，map_AB 和比例精确反解）"""
        out = dict(q)
        for j, v in self.g2h.items():
            if j in out:
                out[j] = v["sign"] * out[j] / v["ratio"]
        for j, v in self.qmap.items():
            if j in out and abs(v["a"]) > 1e-6:
                out[j] = (out[j] - v["b"]) / v["a"]
        for j, k in self.scale.items():
            if j in out and abs(k) > 1e-6:
                out[j] /= k
        return out

    def __call__(self, q):
        q = dict(q)
        for j, k in self.scale.items():
            if j in q:
                q[j] *= k
        for j, v in self.qmap.items():
            if j in q:
                q[j] = v["a"] * q[j] + v["b"]
        for j, v in self.g2h.items():
            if j in q:
                c = v["poly_phi_of_theta"]; t = q[j]
                q[j] = v["sign"] * (c[0] * t ** 3 + c[1] * t ** 2 + c[2] * t + c[3])
        return {j: max(-self.rng, min(self.rng, v)) for j, v in q.items()}
