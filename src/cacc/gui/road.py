"""Animated top-down road view of a simulated platoon (replays a SimResult)."""

from __future__ import annotations

import numpy as np
from PySide6.QtCore import QRectF, Qt, QTimer, Signal
from PySide6.QtGui import QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import QWidget

from cacc.gui.common import MUTED


class RoadView(QWidget):
    """Draws the cars at one time sample; :meth:`play` animates the replay."""

    frame_changed = Signal(int)

    def __init__(self):
        super().__init__()
        self.setMinimumHeight(150)
        self.res = None
        self.k = 0
        self.span = 1.0  # metres shown, leader front to last car's rear
        self.speed = 4.0  # playback speed (x real time)
        self.colors: list[QColor] = []
        self.timer = QTimer(self)
        self.timer.setInterval(33)
        self.timer.timeout.connect(self._tick)

    def set_result(self, res, colors) -> None:
        self.res = res
        self.colors = [QColor.fromRgbF(*c[:3]) if not isinstance(c, str) else QColor(c)
                       for c in colors]
        self.k = min(self.k, len(res.t) - 1)
        self.span = max(float(np.max(res.pos[:, 0] - res.pos[:, -1]))
                        + res.config.vehicle.length, 1.0)
        self.update()

    def set_frame(self, k: int) -> None:
        if self.res is not None:
            self.k = int(np.clip(k, 0, len(self.res.t) - 1))
            self.update()

    def play(self, on: bool) -> None:
        if self.res is None:
            return
        if on and self.k >= len(self.res.t) - 1:
            self.k = 0
        self.timer.start() if on else self.timer.stop()

    def _tick(self) -> None:
        res = self.res
        dt = float(res.t[1] - res.t[0])
        step = max(1, int(round(self.speed * self.timer.interval() / 1000.0 / dt)))
        self.k = min(self.k + step, len(res.t) - 1)
        if self.k >= len(res.t) - 1:
            self.timer.stop()
        self.frame_changed.emit(self.k)
        self.update()

    # ------------------------------------------------------------ painting
    def paintEvent(self, _event):  # noqa: N802 (Qt API)
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        p.fillRect(0, 0, w, h, QColor("#eef0ea"))
        road_top, road_h = h * 0.28, h * 0.44
        p.fillRect(QRectF(0, road_top, w, road_h), QColor("#3d4046"))
        if self.res is None:
            p.setPen(QColor(MUTED))
            p.drawText(self.rect(), Qt.AlignCenter, "Run a simulation to see the platoon")
            return
        res, k = self.res, self.k
        L = res.config.vehicle.length
        pos = res.pos[k]
        # fixed zoom for the whole replay (the leader is pinned at the right),
        # so gaps visibly open and close instead of the view rescaling
        span = self.span
        tail = float(pos[0]) - span
        margin = 40.0
        scale = (w - 2 * margin) / span  # px per metre
        def x_px(x): return margin + (x - tail) * scale

        # lane dashes scroll with the leader so motion is visible
        p.setPen(QPen(QColor("#d9d7cf"), 2, Qt.DashLine))
        mid = road_top + road_h / 2
        dash = 6.0 * scale
        off = (tail % 12.0) * scale
        x = -off
        while x < w:
            p.drawLine(int(x), int(mid), int(x + dash), int(mid))
            x += 12.0 * scale
        car_h = road_h * 0.34
        font = QFont()
        font.setPointSizeF(8)
        p.setFont(font)
        for i in range(pos.size):
            x1 = x_px(float(pos[i]))
            x0 = x1 - L * scale
            rect = QRectF(x0, mid - car_h / 2, max(L * scale, 6), car_h)
            p.setPen(Qt.NoPen)
            p.setBrush(self.colors[i] if i < len(self.colors) else QColor("#888"))
            p.drawRoundedRect(rect, 3, 3)
            p.setPen(QColor("#1a1a19"))
            p.drawText(QRectF(x0 - 20, road_top - 34, L * scale + 40, 16), Qt.AlignCenter,
                       "Lead" if i == 0 else f"F{i}")
            p.drawText(QRectF(x0 - 20, road_top - 18, L * scale + 40, 16), Qt.AlignCenter,
                       f"{res.vel[k, i] * 3.6:.0f} km/h")
            if i > 0:  # gap to the car ahead, under the road
                gap = float(pos[i - 1]) - L - float(pos[i])
                bad = gap < 2.0
                p.setPen(QColor("#b02e2e") if bad else QColor("#52514e"))
                p.drawText(QRectF(x1, road_top + road_h + 2, x_px(float(pos[i - 1])) - L * scale - x1,
                                  16), Qt.AlignCenter, f"{gap:.1f} m")
        p.setPen(QColor("#1a1a19"))
        p.drawText(8, h - 8, f"t = {res.t[k]:.1f} s   ·   playback {self.speed:g}×")
