"""
积木 / 手的滑块控制面板（tkinter 小窗口，和 MuJoCo 窗口一起用）。
主循环里定期调用 panel.update()，再用 panel.take_cube_move() 取出积木的新位置。
"""
import math
import tkinter as tk


class CubePanel:
    def __init__(self, x0, y0, assist=True, auto=True, span=0.10, size=4.0, friction=1.0):
        self.default = (x0, y0)
        self._resized = False
        self.root = tk.Tk()
        self.root.title("积木 / 手 控制")
        self.root.geometry("+40+80")
        self.closed = False
        self.root.protocol("WM_DELETE_WINDOW", self._close)
        self._moved = True                       # 启动时先放一次
        font = ("Noto Sans CJK TC", 11)

        def slider(text, frm, to, init, res):
            var = tk.DoubleVar(value=init)
            tk.Scale(self.root, label=text, from_=frm, to=to, resolution=res, orient=tk.HORIZONTAL, length=320,
                     variable=var, font=font, command=lambda _v: self._mark()).pack(padx=10, pady=2)
            return var

        self.x = slider("积木 X (cm)", (x0 - span) * 100, (x0 + span) * 100, x0 * 100, 0.5)
        self.y = slider("积木 Y (cm)", (y0 - span) * 100, (y0 + span) * 100, y0 * 100, 0.5)
        self.yaw = slider("积木转角 (°)", -90, 90, 0, 5)
        self.size = tk.DoubleVar(value=size)
        tk.Scale(self.root, label="积木大小 (cm)", from_=2, to=8, resolution=0.5, orient=tk.HORIZONTAL, length=320,
                 variable=self.size, font=font, command=lambda _v: self._mark_size()).pack(padx=10, pady=2)
        self.friction = tk.DoubleVar(value=friction)
        tk.Scale(self.root, label="摩擦系数（小 = 滑，大 = 粘）", from_=0.1, to=2.5, resolution=0.1, orient=tk.HORIZONTAL,
                 length=320, variable=self.friction, font=font).pack(padx=10, pady=2)
        self.hand = tk.DoubleVar(value=0.0)
        tk.Scale(self.root, label="手的高度 (cm，在自动抬手之外再加)", from_=-3, to=12, resolution=0.5, orient=tk.HORIZONTAL,
                 length=320, variable=self.hand, font=font).pack(padx=10, pady=2)
        self.assist = tk.BooleanVar(value=assist)
        self.auto = tk.BooleanVar(value=auto)
        tk.Checkbutton(self.root, text="抓取辅助（捏住就粘在手上）", variable=self.assist, font=font).pack(anchor="w", padx=10)
        tk.Checkbutton(self.root, text="抓住后自动抬手 8 cm", variable=self.auto, font=font).pack(anchor="w", padx=10)
        row = tk.Frame(self.root)
        row.pack(pady=4)
        tk.Button(row, text="积木放回滑块位置", font=font, command=self._mark).pack(side=tk.LEFT, padx=4)
        tk.Button(row, text="回到默认位置", font=font, command=self._reset).pack(side=tk.LEFT, padx=4)
        self.status = tk.StringVar(value="")
        tk.Label(self.root, textvariable=self.status, font=font, justify=tk.LEFT).pack(padx=10, pady=4, anchor="w")

    def _mark(self):
        self._moved = True

    def _mark_size(self):
        self._resized = True

    def _reset(self):
        self.x.set(self.default[0] * 100)
        self.y.set(self.default[1] * 100)
        self.yaw.set(0)
        self._moved = True

    def _close(self):
        self.closed = True
        self.root.destroy()

    def update(self, status=None):
        if self.closed:
            return
        if status is not None:
            self.status.set(status)
        self.root.update()

    def take_cube_move(self):
        """积木要移动时返回 (x, y, yaw弧度)，否则 None"""
        if self.closed or not self._moved:
            return None
        self._moved = False
        return self.x.get() / 100, self.y.get() / 100, math.radians(self.yaw.get())

    def take_cube_size(self):
        """积木大小改了就返回新的边长（米），否则 None"""
        if self.closed or not self._resized:
            return None
        self._resized = False
        return self.size.get() / 100

    @property
    def hand_height(self):
        return self.hand.get() / 100

    def close(self):
        if not self.closed:
            self._close()
