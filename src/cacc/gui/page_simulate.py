"""Live platoon page: move a slider, the platoon re-simulates and replays."""

from __future__ import annotations

import math

import numpy as np
from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (QComboBox, QFormLayout, QHBoxLayout, QHeaderView, QLineEdit,
                               QMessageBox, QPushButton, QSlider, QSplitter, QTableWidget,
                               QTableWidgetItem, QTabWidget, QVBoxLayout, QWidget)

from cacc.gui.common import (MUTED, Page, PlotCanvas, VerdictBadge, card, label,
                             primary_button, run_job, vehicle_colors)
from cacc.gui.road import RoadView

#: controller family -> (label, vehicle lag tau [s], gains, default time gap [s])
FAMILIES = {
    "acc": ("ACC — radar only (Ploeg 2014 gains)", 0.1, {"kp": 0.2, "kd": 0.7}, 1.2),
    "cacc": ("CACC — radar + V2V (Ploeg 2014 gains)", 0.1, {"kp": 0.2, "kd": 0.7}, 0.7),
    "cthp": ("CTHP CACC — Ma 2025 case A", 0.5, {"kp": 0.009, "kv": 0.63, "ka": 0.5}, 0.95),
}
LEADERS = {
    "Emergency brake (−4 m/s² for 2 s)": {"profile": "brake", "t_start": 5.0,
                                          "duration": 2.0, "decel": -4.0},
    "Speed wave (sine, 0.1 Hz)": {"profile": "sine", "t_start": 5.0, "amplitude": 1.0,
                                  "freq_hz": 0.1, "duration": 20.0},
    "Stop-and-go bursts": {"profile": "bursts", "bursts": [[5, 10, 1.5, 0.1],
                                                          [25, 10, 1.5, 0.1]]},
}
KEY_METRICS = ("min_gap", "min_time_gap", "min_ttc", "peak_spacing_error",
               "l2_amplification_max", "peak_decel", "rms_accel")


def build_config(family: str, n: int, v0: float, h: float, delay: float, loss: float):
    from cacc import ControllerParams, PlatoonConfig
    from cacc.vehicle import VehicleParams

    _, tau, gains, _ = FAMILIES.get(family, FAMILIES["cacc"])
    return PlatoonConfig(
        n_followers=n, v0=v0, vehicle=VehicleParams(tau=tau),
        control=ControllerParams(h=h, r=2.5, **gains), delay=delay,
        loss_prob=loss, msg_rate=10.0 if loss > 0 else None, dt=0.01, t_final=45.0)


def simulate(cfg, controller: str, leader_spec: dict):
    from cacc import PlatoonSim
    from cacc.criteria import METRICS
    from cacc.platoon import make_leader_profile

    res = PlatoonSim(cfg, controller, make_leader_profile(leader_spec)).run()
    return res, {m: METRICS[m].fn(res) for m in KEY_METRICS}


def sweep(cfg, controller: str):
    """Black-box multisine sweep on 3-4 followers (hops 1->2 ... measured)."""
    from cacc.stringstab import SweepConfig, measure_string_stability

    n = max(3, min(cfg.n_followers, 4))
    return measure_string_stability(cfg, controller, SweepConfig(n_followers=n))


class SimulatePage(Page):
    def __init__(self):
        super().__init__(
            "Live platoon",
            "Change the controller, time gap, V2V latency or packet loss. The platoon "
            "re-simulates instantly. Watch whether a disturbance from the lead car "
            "shrinks (string-stable) or grows (string-unstable) down the line.")
        self.res, self.busy, self.pending = None, False, False

        split = QSplitter(Qt.Horizontal)
        side, form_box = card()
        form = QFormLayout()
        self.family = QComboBox()
        for key, (text, *_rest) in FAMILIES.items():
            self.family.addItem(text, key)
        self.family.addItem("My controller (plugin file.py:Class)", "plugin")
        self.family.setCurrentIndex(1)
        self.plugin = QLineEdit()
        self.plugin.setPlaceholderText(r"C:\path\my_controller.py:MyController")
        self.plugin.setEnabled(False)
        self.leader = QComboBox()
        self.leader.addItems(list(LEADERS))
        self.n = self._slider(2, 12, 5, "{} cars")
        self.v0 = self._slider(5, 35, 20, "{} m/s")
        self.h = self._slider(10, 300, 70, "{:.2f} s", scale=0.01)
        self.delay = self._slider(0, 500, 100, "{} ms")
        self.loss = self._slider(0, 50, 0, "{} %")
        form.addRow("Controller", self.family)
        form.addRow("Plugin", self.plugin)
        form.addRow("Lead car", self.leader)
        for text, (s, lab) in (("Followers", self.n), ("Cruise speed", self.v0),
                               ("Time gap h", self.h), ("V2V latency", self.delay),
                               ("Packet loss", self.loss)):
            row = QHBoxLayout()
            row.addWidget(s, 1)
            row.addWidget(lab)
            form.addRow(text, row)
        form_box.addLayout(form)
        self.sweep_btn = primary_button("Measure string stability")
        self.sweep_btn.clicked.connect(self._sweep)
        self.badge = VerdictBadge()
        self.sweep_note = label("", "subtitle")
        form_box.addWidget(self.sweep_btn)
        form_box.addWidget(self.badge)
        form_box.addWidget(self.sweep_note)
        self.metrics = QTableWidget(len(KEY_METRICS), 2)
        self.metrics.setHorizontalHeaderLabels(("Metric", "Value"))
        self.metrics.verticalHeader().hide()
        self.metrics.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        form_box.addWidget(self.metrics, 1)
        side.setMinimumWidth(330)
        split.addWidget(side)

        right = QWidget()
        rl = QVBoxLayout(right)
        rl.setContentsMargins(0, 0, 0, 0)
        self.road = RoadView()
        controls = QHBoxLayout()
        self.play_btn = QPushButton("▶  Play")
        self.play_btn.setCheckable(True)
        self.play_btn.toggled.connect(self._play)
        self.scrub = QSlider(Qt.Horizontal)
        self.scrub.valueChanged.connect(self.road.set_frame)
        self.road.frame_changed.connect(self._on_frame)
        self.road.timer.timeout.connect(self._maybe_stop)
        self.speed = QComboBox()
        self.speed.addItems(["1×", "2×", "4×", "8×"])
        self.speed.setCurrentIndex(2)
        self.speed.currentTextChanged.connect(
            lambda s: setattr(self.road, "speed", float(s.rstrip("×"))))
        controls.addWidget(self.play_btn)
        controls.addWidget(self.scrub, 1)
        controls.addWidget(self.speed)
        self.tabs = QTabWidget()
        self.plot = PlotCanvas(rows=2, height=4.0)
        self.gain_plot = PlotCanvas(rows=1, height=3.0)
        self.tabs.addTab(self.plot, "Speeds and spacing errors")
        self.tabs.addTab(self.gain_plot, "String-stability sweep")
        rl.addWidget(self.road)
        rl.addLayout(controls)
        rl.addWidget(self.tabs, 1)
        split.addWidget(right)
        split.setSizes([340, 800])
        self.body.addWidget(split, 1)

        self.debounce = QTimer(self)
        self.debounce.setSingleShot(True)
        self.debounce.setInterval(200)
        self.debounce.timeout.connect(self._simulate)
        self.family.currentIndexChanged.connect(self._family_changed)
        self.plugin.editingFinished.connect(self.debounce.start)
        self.leader.currentIndexChanged.connect(self.debounce.start)
        self._family_changed()

    def _slider(self, lo, hi, val, fmt, scale=1.0):
        s = QSlider(Qt.Horizontal)
        s.setRange(lo, hi)
        s.setValue(val)
        lab = label(fmt.format(val * scale if scale != 1.0 else val), wrap=False)
        lab.setMinimumWidth(62)
        s.valueChanged.connect(
            lambda v: lab.setText(fmt.format(v * scale if scale != 1.0 else v)))
        s.valueChanged.connect(lambda _v: self.debounce.start())
        return s, lab

    # ------------------------------------------------------------ state
    def _controller(self) -> str:
        key = self.family.currentData()
        return self.plugin.text().strip().strip('"') if key == "plugin" else key

    def _config(self):
        key = self.family.currentData()
        return build_config("cacc" if key == "plugin" else key, self.n[0].value(),
                            float(self.v0[0].value()), self.h[0].value() / 100.0,
                            self.delay[0].value() / 1000.0, self.loss[0].value() / 100.0)

    def _family_changed(self) -> None:
        key = self.family.currentData()
        self.plugin.setEnabled(key == "plugin")
        if key in FAMILIES:
            self.h[0].setValue(int(round(FAMILIES[key][3] * 100)))
        self.badge.set(None)
        self.sweep_note.setText("")
        self.debounce.start()

    # ------------------------------------------------------------ simulate
    def _simulate(self) -> None:
        if self.family.currentData() == "plugin" and ":" not in self._controller():
            return
        if self.busy:
            self.pending = True
            return
        self.busy = True
        run_job(simulate, self._config(), self._controller(),
                LEADERS[self.leader.currentText()],
                on_done=self._sim_done, on_error=self._sim_failed)

    def _sim_failed(self, msg: str) -> None:
        self.busy = False
        QMessageBox.warning(self, "Simulation failed", msg)

    def _sim_done(self, out) -> None:
        self.busy = False
        self.res, vals = out
        res = self.res
        colors = vehicle_colors(res.config.n_followers)
        self.road.set_result(res, colors)
        self.scrub.blockSignals(True)
        self.scrub.setRange(0, len(res.t) - 1)
        self.scrub.setValue(self.road.k)
        self.scrub.blockSignals(False)
        a1, a2 = self.plot.clear()
        for i in range(res.vel.shape[1]):
            a1.plot(res.t, res.vel[:, i], color=colors[i], lw=1.8 if i == 0 else 1.2,
                    label="lead" if i == 0 else f"F{i}")
        for i in range(res.err.shape[1]):
            a2.plot(res.t, res.err[:, i], color=colors[i + 1], lw=1.2)
        a1.set_ylabel("speed [m/s]", fontsize=9, color=MUTED)
        a2.set_ylabel("spacing error [m]", fontsize=9, color=MUTED)
        a2.set_xlabel("time [s]", fontsize=9, color=MUTED)
        a1.legend(frameon=False, fontsize=7, ncol=min(7, res.vel.shape[1]), loc="upper right")
        peaks = np.max(np.abs(res.err), axis=0)
        grows = res.err.shape[1] > 1 and peaks[-1] > peaks[0] * 1.02
        a2.set_title("disturbance GROWS down the platoon" if grows else
                     "disturbance shrinks down the platoon", fontsize=9, loc="left",
                     color="#b02e2e" if grows else "#067a06")
        self.plot.draw()
        from cacc.criteria import METRICS
        for r, m in enumerate(KEY_METRICS):
            v = vals[m]
            self.metrics.setItem(r, 0, QTableWidgetItem(m.replace("_", " ")))
            self.metrics.setItem(r, 1, QTableWidgetItem(
                f"{v:.3f} {METRICS[m].unit}" if math.isfinite(v) else str(v)))
        if self.pending:
            self.pending = False
            self._simulate()

    # ------------------------------------------------------------ replay
    def _play(self, on: bool) -> None:
        self.play_btn.setText("❚❚  Pause" if on else "▶  Play")
        self.road.play(on)

    def _on_frame(self, k: int) -> None:
        self.scrub.blockSignals(True)
        self.scrub.setValue(k)
        self.scrub.blockSignals(False)

    def _maybe_stop(self) -> None:
        if not self.road.timer.isActive() and self.play_btn.isChecked():
            self.play_btn.setChecked(False)

    # ------------------------------------------------------------ sweep
    def _sweep(self) -> None:
        self.sweep_btn.setEnabled(False)
        self.sweep_note.setText("Driving the lead car with a multi-tone speed signal and "
                                "measuring each hop's gain… (5–30 s)")
        run_job(sweep, self._config(), self._controller(),
                on_done=self._sweep_done, on_error=self._sweep_failed)

    def _sweep_failed(self, msg: str) -> None:
        self.sweep_btn.setEnabled(True)
        self.sweep_note.setText("")
        QMessageBox.warning(self, "Sweep failed", msg)

    def _sweep_done(self, r) -> None:
        self.sweep_btn.setEnabled(True)
        stable = r.string_stable(1.0)
        verdict = "marginal" if r.marginal(1.0) else (
            "string-stable" if stable else "string-unstable")
        self.badge.set(verdict, f"peak |Γ| = {r.peak_gain:.3f}")
        self.sweep_note.setText(f"Worst amplification at {r.peak_omega:.3f} rad/s. "
                                "A gain above 1 means speed waves grow car by car.")
        (ax,) = self.gain_plot.clear()
        colors = vehicle_colors(max(r.hops) + 1)
        ax.axhline(1.0, color=MUTED, lw=1.0, ls=(0, (4, 3)))
        for row, i in zip(r.gain, r.hops, strict=True):
            ax.semilogx(r.omega, row, "-o", ms=3.5, color=colors[i], lw=1.4,
                        label=f"hop F{i - 1} → F{i}")
        ax.set_xlabel("frequency ω [rad/s]", fontsize=9, color=MUTED)
        ax.set_ylabel("measured |Γ(jω)|", fontsize=9, color=MUTED)
        ax.legend(frameon=False, fontsize=8)
        self.gain_plot.draw()
        self.tabs.setCurrentWidget(self.gain_plot)
