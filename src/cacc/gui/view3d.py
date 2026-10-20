"""3-D replay of a simulated run: the platoon on a highway, in perspective.

A small software renderer on QPainter (no OpenGL, so it runs in the frozen
.exe, over remote desktop and headless in tests): boxes for car bodies,
cabins and wheels, flat-shaded with back-face culling and the painter's
algorithm, polygons clipped at the near plane. On top of the physics it
shows what an engineer looks for in a replay: brake lights when a car
decelerates, the V2V links (cyan; red when the trust gate distrusts them,
absent behind a human or a car that cut in), cars cutting in from the next
lane, and a red flash and label at the moment of a collision.

Cameras: *Drone* (orbit: drag to turn, wheel to zoom), *Chase* (behind the
last car) and *Side*. :class:`Replay3DPanel` adds play / scrub / speed and
emits the frame so the trace plots can follow.
"""

from __future__ import annotations

import math

import numpy as np
from PySide6.QtCore import QPointF, Qt, QTimer, Signal
from PySide6.QtGui import QBrush, QColor, QFont, QLinearGradient, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import (QComboBox, QHBoxLayout, QPushButton, QSlider, QVBoxLayout,
                               QWidget)

from cacc.gui.common import MUTED, label, vehicle_colors

LANE = 3.6  # lane width [m]
NEAR = 0.5  # near clipping plane [m]
WIDTH, HEIGHT = 1.85, 1.45  # car body [m]
LIGHT = np.array([-0.35, -0.45, 0.82]) / np.linalg.norm([-0.35, -0.45, 0.82])
HUMAN, CUTIN = "#8d8d8d", "#8a3ffc"

# unit box faces (vertex indices) with outward normals
_V = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0],
               [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]], float)
_F = (((0, 1, 2, 3), (0, 0, -1)), ((4, 7, 6, 5), (0, 0, 1)),
      ((0, 4, 5, 1), (0, -1, 0)), ((3, 2, 6, 7), (0, 1, 0)),
      ((1, 5, 6, 2), (1, 0, 0)), ((0, 3, 7, 4), (-1, 0, 0)))


class Camera:
    def __init__(self, eye, target):
        self.eye = np.asarray(eye, float)
        f = np.asarray(target, float) - self.eye
        self.f = f / np.linalg.norm(f)
        r = np.cross(self.f, (0.0, 0.0, 1.0))
        self.r = r / (np.linalg.norm(r) or 1.0)
        self.u = np.cross(self.r, self.f)

    def to_cam(self, pts: np.ndarray) -> np.ndarray:
        d = pts - self.eye
        return np.column_stack([d @ self.r, d @ self.u, d @ self.f])


def _clip(poly: np.ndarray) -> np.ndarray:
    """Sutherland-Hodgman clip of a camera-space polygon at z = NEAR."""
    out = []
    n = len(poly)
    for k in range(n):
        a, b = poly[k], poly[(k + 1) % n]
        ina, inb = a[2] >= NEAR, b[2] >= NEAR
        if ina:
            out.append(a)
        if ina != inb:
            s = (NEAR - a[2]) / (b[2] - a[2])
            out.append(a + s * (b - a))
    return np.array(out)


class View3D(QWidget):
    """Draws one frame of a :class:`SimResult` in 3-D."""

    def __init__(self):
        super().__init__()
        self.setMinimumHeight(260)
        self.res, self.k = None, 0
        self.mode = "Drone"
        self.yaw, self.pitch, self.zoom = -125.0, 30.0, 1.0
        self._drag = None
        self.crash: dict[int, int] = {}

    # ------------------------------------------------------------ data
    def set_result(self, res) -> None:
        from cacc.traffic import front_tracks

        self.res = res
        self.k = min(self.k, len(res.t) - 1)
        n = res.vel.shape[1] - 1
        self.cols = [QColor(c) if isinstance(c, str) else QColor.fromRgbF(*c[:3])
                     for c in vehicle_colors(n)]
        tr = res.traffic or {}
        self.humans = set(tr.get("humans", []))
        self.cutins = tr.get("cut_ins", [])
        gaps, _ = front_tracks(res)
        self.crash = {}
        for j in range(gaps.shape[1]):
            hit = np.flatnonzero(gaps[:, j] <= 0.0)
            if hit.size:
                self.crash[j] = int(hit[0])
        self.v2v = res.controller != "acc"
        self.update()

    def set_frame(self, k: int) -> None:
        if self.res is not None:
            self.k = int(np.clip(k, 0, len(self.res.t) - 1))
            self.update()

    # ------------------------------------------------------------ camera
    def _camera(self) -> Camera:
        res, k = self.res, self.k
        L = res.config.vehicle.length
        front, back = float(res.pos[k, 0]), float(res.pos[k, -1]) - L
        mid = 0.5 * (front + back)
        span = max(front - back, 20.0)
        if self.mode == "Chase":
            return Camera((back - 14.0, 0.0, 5.0), (back + 35.0, 0.0, 0.8))
        if self.mode == "Side":
            d = (0.47 * span + 6.0) * self.zoom
            return Camera((mid, -d, 0.22 * d), (mid, 0.0, 0.0))
        d = (0.36 * span + 12.0) * self.zoom
        yaw, pitch = math.radians(self.yaw), math.radians(self.pitch)
        eye = (mid + d * math.cos(pitch) * math.cos(yaw),
               d * math.cos(pitch) * math.sin(yaw), d * math.sin(pitch) + 1.0)
        return Camera(eye, (mid, 0.0, 0.0))

    def mousePressEvent(self, e):  # noqa: N802 (Qt API)
        self._drag = e.position()

    def mouseMoveEvent(self, e):  # noqa: N802
        if self._drag is not None and self.mode == "Drone":
            d = e.position() - self._drag
            self._drag = e.position()
            self.yaw -= d.x() * 0.4
            self.pitch = float(np.clip(self.pitch + d.y() * 0.3, 4.0, 85.0))
            self.update()

    def mouseReleaseEvent(self, _e):  # noqa: N802
        self._drag = None

    def wheelEvent(self, e):  # noqa: N802
        self.zoom = float(np.clip(self.zoom * (0.9 if e.angleDelta().y() > 0 else 1.1),
                                  0.3, 4.0))
        self.update()

    # ------------------------------------------------------------ geometry
    def _project(self, cam: Camera, pts: np.ndarray):
        c = cam.to_cam(pts)
        c = _clip(c)
        if len(c) < 3:
            return None, 0.0
        w, h = self.width(), self.height()
        f = h / (2 * math.tan(math.radians(28)))
        poly = QPolygonF([QPointF(w / 2 + f * x / z, h / 2 - f * y / z) for x, y, z in c])
        return poly, float(c[:, 2].mean())

    def _point(self, cam: Camera, p) -> QPointF | None:
        x, y, z = cam.to_cam(np.asarray([p], float))[0]
        if z < NEAR:
            return None
        w, h = self.width(), self.height()
        f = h / (2 * math.tan(math.radians(28)))
        return QPointF(w / 2 + f * x / z, h / 2 - f * y / z)

    def _box(self, cam, faces, origin, size, color: QColor, tag=None):
        v = _V * size + origin
        for idx, nrm in _F:
            c = v[list(idx)].mean(axis=0)
            if np.dot(nrm, cam.eye - c) <= 0:
                continue  # back face
            poly, depth = self._project(cam, v[list(idx)])
            if poly is None:
                continue
            shade = 0.55 + 0.45 * max(0.0, float(np.dot(nrm, LIGHT)))
            col = QColor.fromRgbF(min(color.redF() * shade, 1), min(color.greenF() * shade, 1),
                                  min(color.blueF() * shade, 1), color.alphaF())
            faces.append((depth, poly, col, tag))

    def _car(self, cam, faces, x_front: float, y: float, color: QColor,
             braking: bool, L: float):
        x0 = x_front - L
        self._box(cam, faces, (x0, y - WIDTH / 2, 0.28), (L, WIDTH, 0.62), color)
        cab = QColor(color).darker(140)
        self._box(cam, faces, (x0 + 0.22 * L, y - 0.8, 0.9), (0.5 * L, 1.6, 0.55), cab)
        for dx in (0.15 * L, 0.78 * L):
            for dy in (-WIDTH / 2 - 0.02, WIDTH / 2 - 0.26):
                self._box(cam, faces, (x0 + dx, y + dy, 0.0), (0.62, 0.28, 0.62),
                          QColor("#1c1c1c"))
        lamp = QColor("#ff2a1a") if braking else QColor("#5a1410")
        for dy in (-0.75, 0.45):  # rear lamps, just behind the body
            self._box(cam, faces, (x0 - 0.04, y + dy, 0.62), (0.05, 0.3, 0.16), lamp,
                      "lamp" if braking else None)

    # ------------------------------------------------------------ painting
    def paintEvent(self, _event):  # noqa: N802
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        sky = QLinearGradient(0, 0, 0, h)
        sky.setColorAt(0.0, QColor("#9cc3e6"))
        sky.setColorAt(1.0, QColor("#e9f1f7"))
        p.fillRect(0, 0, w, h, QBrush(sky))
        if self.res is None:
            p.setPen(QColor(MUTED))
            p.drawText(self.rect(), Qt.AlignCenter, "Open a cell to replay it in 3-D")
            return
        res, k = self.res, self.k
        L = res.config.vehicle.length
        cam = self._camera()
        mid = float(res.pos[k, 0])
        p.setPen(Qt.NoPen)

        def ground(x0, x1, y0, y1, color, z=0.0):
            pts = np.array([[x0, y0, z], [x1, y0, z], [x1, y1, z], [x0, y1, z]])
            poly, _ = self._project(cam, pts)
            if poly is not None:
                p.setBrush(QColor(color))
                p.drawPolygon(poly)

        far = 1500.0
        ground(mid - far, mid + far, -far, far, "#9db57d")
        ground(mid - far, mid + far, -1.5 * LANE - 1.0, 1.5 * LANE + 1.0, "#4a4d52", 0.01)
        for y in (-1.5 * LANE, 1.5 * LANE):  # solid edge lines
            ground(mid - far, mid + far, y - 0.08, y + 0.08, "#e8e6df", 0.02)
        x_last = float(res.pos[k, -1])
        start = math.floor((x_last - 120.0) / 12.0) * 12.0
        for x in np.arange(start, mid + 260.0, 12.0):  # dashes move with the road
            for y in (-0.5 * LANE, 0.5 * LANE):
                ground(x, x + 3.0, y - 0.07, y + 0.07, "#f0eee6", 0.02)

        faces = []
        n = res.vel.shape[1] - 1
        for i in range(n + 1):
            color = QColor(HUMAN) if i in self.humans else self.cols[i]
            braking = res.acc[k, i] < -0.5
            self._car(cam, faces, float(res.pos[k, i]), 0.0, color, braking, L)
        for car in self.cutins:
            if np.isfinite(car["pos"][k]):
                self._car(cam, faces, float(car["pos"][k]), float(car["lateral"][k]),
                          QColor(CUTIN), car["acc"][k] < -0.5, L)
        faces.sort(key=lambda f: -f[0])
        for _d, poly, col, tag in faces:
            p.setBrush(col)
            p.drawPolygon(poly)
            if tag == "lamp":
                r = poly.boundingRect()
                s = max(1.5, min(8.0, 1.6 * max(r.width(), r.height())))
                p.setBrush(QColor(255, 60, 30, 70))
                p.drawEllipse(r.center(), s, 0.8 * s)

        self._links(p, cam, k, n)
        self._labels(p, cam, k, n, L)
        self._hud(p, k, n, w, h)

    def _links(self, p, cam, k, n):
        if not self.v2v:
            return
        res = self.res
        cut = {c["ahead_of"] for c in self.cutins if np.isfinite(c["pos"][k])}
        for i in range(1, n + 1):
            if (i - 1) in self.humans or i in self.humans or i in cut:
                continue
            a = self._point(cam, (res.pos[k, i - 1] - 1.8, 0.0, 2.3))
            b = self._point(cam, (res.pos[k, i] - 1.8, 0.0, 2.3))
            if a is None or b is None:
                continue
            g = 1.0 if res.trust is None else float(res.trust[k, i - 1])
            pen = QPen(QColor(0, 180, 220, 190) if g > 0.5 else QColor(220, 40, 40, 220),
                       2.0, Qt.SolidLine if g > 0.5 else Qt.DashLine)
            p.setPen(pen)
            p.drawLine(a, b)
        p.setPen(Qt.NoPen)

    def _labels(self, p, cam, k, n, L):
        res = self.res
        p.setFont(QFont("Segoe UI", 8, QFont.Bold))
        for i in range(n + 1):
            q = self._point(cam, (res.pos[k, i] - L / 2, 0.0, 2.9))
            if q is None:
                continue
            txt = "lead" if i == 0 else (f"{i} · human" if i in self.humans else str(i))
            crashed = (i - 1) in self.crash and k >= self.crash[i - 1] if i else False
            aeb = i and res.aeb is not None and res.aeb[k, i - 1] > 0
            if aeb:
                txt += " · AEB"
            p.setPen(QColor("#c4161c") if crashed else QColor("#d97a00") if aeb
                     else QColor("#1f2328"))
            p.drawText(q + QPointF(-14, 0), txt)
        for car in self.cutins:
            if np.isfinite(car["pos"][k]):
                q = self._point(cam, (car["pos"][k] - L / 2, car["lateral"][k], 2.9))
                if q is not None:
                    p.setPen(QColor(CUTIN))
                    p.drawText(q + QPointF(-20, 0), "cut-in")

    def _hud(self, p, k, n, w, h):
        res = self.res
        t = res.t[k]
        crashed = [j for j, kk in self.crash.items() if k >= kk]
        p.setPen(QColor("#1f2328"))
        p.setFont(QFont("Segoe UI", 10, QFont.Bold))
        p.drawText(12, 22, f"t = {t:5.1f} s    lead {res.vel[k, 0] * 3.6:5.1f} km/h")
        p.setFont(QFont("Segoe UI", 8))
        p.drawText(12, 40, "   ".join(f"{i}: {res.vel[k, i] * 3.6:.0f}"
                                     for i in range(1, n + 1)) + "  km/h")
        if crashed:
            j = min(crashed, key=lambda c: self.crash[c])
            who = f"car {j + 1}" if j < n else "cut-in car"
            fresh = (k - self.crash[j]) * res.config.dt < 1.5
            if fresh:
                p.fillRect(0, 0, w, h, QColor(220, 0, 0, 45))
            p.setPen(QColor("#c4161c"))
            p.setFont(QFont("Segoe UI", 15, QFont.Bold))
            p.drawText(self.rect().adjusted(0, 14, -14, 0), Qt.AlignTop | Qt.AlignRight,
                       f"COLLISION · {who} at t = {res.t[self.crash[j]]:.1f} s")


class Replay3DPanel(QWidget):
    """The 3-D view with play / scrub / speed / camera controls."""

    frame_changed = Signal(float)  # simulation time of the shown frame

    def __init__(self):
        super().__init__()
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        self.view = View3D()
        lay.addWidget(self.view, 1)
        row = QHBoxLayout()
        self.play_btn = QPushButton("▶ Play")
        self.play_btn.setEnabled(False)
        self.play_btn.clicked.connect(self.toggle)
        self.slider = QSlider(Qt.Horizontal)
        self.slider.valueChanged.connect(self._scrub)
        self.speed = QComboBox()
        for s in ("0.25×", "0.5×", "1×", "2×", "4×"):
            self.speed.addItem(s, float(s[:-1]))
        self.speed.setCurrentIndex(2)
        self.cam = QComboBox()
        self.cam.addItems(("Drone", "Chase", "Side"))
        self.cam.currentTextChanged.connect(self._camera)
        self.jump = QPushButton("Jump to crash")
        self.jump.setEnabled(False)
        self.jump.clicked.connect(self._to_crash)
        row.addWidget(self.play_btn)
        row.addWidget(self.slider, 1)
        for text, w in (("speed", self.speed), ("camera", self.cam)):
            row.addWidget(label(text, wrap=False))
            row.addWidget(w)
        row.addWidget(self.jump)
        lay.addLayout(row)
        self.timer = QTimer(self, interval=33)
        self.timer.timeout.connect(self._tick)

    def set_result(self, res) -> None:
        self.timer.stop()
        self.play_btn.setText("▶ Play")
        self.view.set_result(res)
        self.slider.blockSignals(True)
        self.slider.setRange(0, len(res.t) - 1)
        self.slider.setValue(0)
        self.slider.blockSignals(False)
        self.view.set_frame(0)
        self.play_btn.setEnabled(True)
        self.jump.setEnabled(bool(self.view.crash))

    def toggle(self) -> None:
        if self.view.res is None:
            return
        if self.timer.isActive():
            self.timer.stop()
            self.play_btn.setText("▶ Play")
        else:
            if self.slider.value() >= self.slider.maximum():
                self.slider.setValue(0)
            self.timer.start()
            self.play_btn.setText("❚❚ Pause")

    def _tick(self) -> None:
        res = self.view.res
        dt = float(res.t[1] - res.t[0])
        step = max(1, int(round(self.speed.currentData() * 0.033 / dt)))
        k = min(self.slider.value() + step, self.slider.maximum())
        self.slider.setValue(k)
        if k >= self.slider.maximum():
            self.toggle()

    def _scrub(self, k: int) -> None:
        self.view.set_frame(k)
        if self.view.res is not None:
            self.frame_changed.emit(float(self.view.res.t[k]))

    def _camera(self, mode: str) -> None:
        self.view.mode = mode
        self.view.update()

    def _to_crash(self) -> None:
        if self.view.crash:
            k = min(self.view.crash.values())
            dt = self.view.res.config.dt
            self.slider.setValue(max(0, k - int(3.0 / dt)))
            if not self.timer.isActive():
                self.toggle()
