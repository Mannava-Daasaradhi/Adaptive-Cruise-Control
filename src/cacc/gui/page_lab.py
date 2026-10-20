"""Controller Lab: build a controller from transfer functions and see inside it.

Type one transfer function per measured signal (spacing error, its rate,
relative speed, the V2V feedforward). Every edit re-draws, live:

* **String stability** — |Γ(jω)| of the whole platoon hop with this design,
  plus each block's own gain |K(jω)|;
* **Poles** — the follower's closed loop in the s-plane;
* **Physics** — the equations the engine is solving, filled in with your
  blocks and the plant (vehicle lag, dead time, V2V delay);
* **Inside the controller** — a platoon run of one maneuver, with the
  follower's command split into what each block contributed, and where the
  actuator clipped it.

"Auto-tune" searches the numbers in your blocks for the shortest headway
that is string-stable and passes the key stress cells (cacc.autotune);
"Stress test this" sends the design to the Stress test page.
"""

from __future__ import annotations

import numpy as np
from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (QComboBox, QDoubleSpinBox, QFileDialog, QGridLayout,
                               QHBoxLayout, QLineEdit, QMessageBox, QPlainTextEdit,
                               QPushButton, QSpinBox, QSplitter, QTabWidget, QVBoxLayout,
                               QWidget)

from cacc import tfctrl as tc
from cacc.gui.common import (BLUE, GREEN, INK, MUTED, ORANGE, RED, AQUA, Page,
                             PlotCanvas, VerdictBadge, card, label, primary_button,
                             run_job, vehicle_colors)

BLOCK_COLORS = {"e": BLUE, "e_dot": AQUA, "dv": ORANGE, "u_ff": "#8a3ffc"}
MANEUVERS = ("hard-brake", "emergency-stop", "traffic-wave", "stop-and-go",
             "full-throttle")


def simulate(design: tc.Design, maneuver: str, tau: float, act_delay: float,
             delay: float, follower: int):
    """Background job: one clean-link platoon run + per-block breakdown."""
    import yaml

    from cacc import campaign as cp
    from cacc.platoon import PlatoonSim
    from cacc.scenario import scenario_from_dict

    base = yaml.safe_load(cp.BASE.read_text(encoding="utf-8"))
    tree = cp.compose(base, maneuver, "nominal", tc.REF)
    tree["controller"] = {**tree.get("controller", {}), **design.controller_block}
    tree["vehicle"].update(tau=tau, act_delay=act_delay)
    tree["network"] = {"delay": delay}
    sc = scenario_from_dict(tree, default_name=maneuver)
    res = PlatoonSim(sc.config, tc.REF, sc.leader, design.options).run()
    return res, tc.contributions(res, design, follower)


def run_tune(design: tc.Design, tau: float, act_delay: float, theta: float):
    """Background job: :func:`cacc.autotune.tune` on all but one core."""
    import os

    from cacc.autotune import tune

    return tune(design, tau=tau, act_delay=act_delay,
                thetas=tuple(sorted({theta, max(theta, 0.15)})),
                jobs=max(1, (os.cpu_count() or 2) - 1))


def _spin(lo, hi, val, step, dec=2, suffix=""):
    w = QDoubleSpinBox()
    w.setRange(lo, hi)
    w.setDecimals(dec)
    w.setSingleStep(step)
    w.setValue(val)
    w.setSuffix(suffix)
    return w


class LabPage(Page):
    stress_design = Signal(object)  # a tc.Design for the Stress test page

    def __init__(self):
        super().__init__(
            "Controller Lab",
            "Build your own controller from transfer functions in s, one per "
            "signal the car measures, and watch the physics live: string "
            "stability of the platoon, the closed-loop poles, and, in a "
            "simulated maneuver, exactly what each block of your controller "
            "commanded. Names h and r read the spacing policy; lag(T), "
            "lead(Tz, Tp), pi(kp, ki) and lowpass2(wn, zeta) are built in.")
        self.design, self.report = None, None
        self._timer = QTimer(self, singleShot=True, interval=250)
        self._timer.timeout.connect(self.refresh)

        top, tl = card()
        row = QHBoxLayout()
        self.preset = QComboBox()
        for key, (desc, *_r) in tc.PRESETS.items():
            self.preset.addItem(desc, key)
        load = QPushButton("Load preset")
        load.clicked.connect(lambda: self.load(tc.Design.preset(self.preset.currentData())))
        self.ff = QComboBox()
        self.ff.addItem("V2V sends commanded accel u", "u")
        self.ff.addItem("V2V sends realized accel a", "a")
        self.h = _spin(0.0, 5.0, 0.7, 0.05, suffix=" s")
        self.r = _spin(0.0, 20.0, 2.5, 0.5, 1, " m")
        self.tau = _spin(0.01, 2.0, 0.2, 0.05, suffix=" s")
        self.act = _spin(0.0, 1.0, 0.0, 0.01, suffix=" s")
        self.delay = _spin(0.0, 1.0, 0.05, 0.01, suffix=" s")
        for text, w in (("Start from", self.preset), (None, load), ("V2V", self.ff),
                        ("headway h", self.h), ("standstill r", self.r)):
            if text:
                row.addWidget(label(text, wrap=False))
            row.addWidget(w)
        row.addStretch(1)
        tl.addLayout(row)
        row2 = QHBoxLayout()
        for text, w in (("Car: lag τ", self.tau), ("dead time", self.act),
                        ("V2V delay θ", self.delay)):
            row2.addWidget(label(text, wrap=False))
            row2.addWidget(w)
        row2.addStretch(1)
        self.badge = VerdictBadge()
        row2.addWidget(self.badge)
        for text, fn in (("Open…", self._open), ("Save…", self._save),
                         ("Export MATLAB…", self._matlab)):
            b = QPushButton(text)
            b.clicked.connect(fn)
            row2.addWidget(b)
        self.tune_btn = QPushButton("Auto-tune…")
        self.tune_btn.setToolTip("Search the numbers in your blocks for the shortest "
                                 "headway that stays string-stable (up to 150 ms "
                                 "V2V delay) and passes hard brake, wet emergency "
                                 "stop, 300 ms latency and a cut-in")
        self.tune_btn.clicked.connect(self._tune)
        row2.addWidget(self.tune_btn)
        self.stress_btn = primary_button("Stress test this →")
        self.stress_btn.clicked.connect(self._send)
        row2.addWidget(self.stress_btn)
        tl.addLayout(row2)
        self.body.addWidget(top)

        split = QSplitter(Qt.Horizontal)
        left = QWidget()
        ll = QVBoxLayout(left)
        ll.setContentsMargins(0, 0, 0, 0)
        ll.addWidget(label("u(s) = Σ K(s) · signal", "h2"))
        grid = QGridLayout()
        self.edits, self.state = {}, {}
        for r, sig in enumerate(tc.SIGNALS):
            ed = QLineEdit()
            ed.setFont(QFont("Consolas", 10))
            ed.textChanged.connect(self._changed)
            st = label("", "subtitle", wrap=False)
            grid.addWidget(label(f"K for {tc.SIGNAL_LABELS[sig]}", wrap=False), 2 * r, 0, 1, 2)
            grid.addWidget(ed, 2 * r + 1, 0)
            grid.addWidget(st, 2 * r + 1, 1)
            self.edits[sig], self.state[sig] = ed, st
        ll.addLayout(grid)
        ll.addWidget(label("Physics", "h2"))
        self.physics = QPlainTextEdit()
        self.physics.setReadOnly(True)
        self.physics.setFont(QFont("Consolas", 9))
        ll.addWidget(self.physics, 1)
        left.setMinimumWidth(360)
        split.addWidget(left)

        self.tabs = QTabWidget()
        self.bode = PlotCanvas(rows=2, height=5.0)
        self.poles = PlotCanvas(rows=1, height=5.0, sharex=False)
        inside = QWidget()
        il = QVBoxLayout(inside)
        il.setContentsMargins(6, 6, 6, 6)
        ir = QHBoxLayout()
        self.man = QComboBox()
        for m in MANEUVERS:
            self.man.addItem(m)
        self.follower = QSpinBox()
        self.follower.setRange(1, 5)
        self.follower.setValue(2)
        self.run_btn = QPushButton("Run maneuver")
        self.run_btn.clicked.connect(self.run_inside)
        self.inside_status = label("", "subtitle")
        for text, w in (("Maneuver", self.man), ("show follower", self.follower)):
            ir.addWidget(label(text, wrap=False))
            ir.addWidget(w)
        ir.addWidget(self.run_btn)
        ir.addWidget(self.inside_status, 1)
        il.addLayout(ir)
        self.trace = PlotCanvas(rows=3, height=6.0)
        il.addWidget(self.trace, 1)
        self.tabs.addTab(self.bode, "String stability")
        self.tabs.addTab(self.poles, "Poles")
        self.tabs.addTab(inside, "Inside the controller")
        split.addWidget(self.tabs)
        split.setSizes([420, 900])
        self.body.addWidget(split, 1)

        for w in (self.h, self.r, self.tau, self.act, self.delay):
            w.valueChanged.connect(self._changed)
        self.ff.currentIndexChanged.connect(self._changed)
        self.load(tc.Design.preset("cacc"))

    # ------------------------------------------------------------ design io
    def load(self, d: tc.Design) -> None:
        for w in (*self.edits.values(), self.h, self.r, self.ff):
            w.blockSignals(True)
        for sig, ed in self.edits.items():
            ed.setText(d.blocks.get(sig, "0"))
        self.h.setValue(d.h)
        self.r.setValue(d.r)
        self.ff.setCurrentIndex(0 if d.ff_signal == "u" else 1)
        for w in (*self.edits.values(), self.h, self.r, self.ff):
            w.blockSignals(False)
        self._name = d.name
        self.refresh()

    def current(self) -> tc.Design:
        return tc.Design({s: e.text().strip() or "0" for s, e in self.edits.items()},
                         self.ff.currentData(), self.h.value(), self.r.value(),
                         getattr(self, "_name", "my-controller"))

    def _changed(self) -> None:
        self._timer.start()

    # ------------------------------------------------------------ analysis
    def refresh(self) -> None:
        d = self.current()
        ok = True
        for sig, st in self.state.items():
            try:
                tf = tc.parse(d.blocks[sig], {"h": d.h, "r": d.r})
                if not tf.proper:
                    raise ValueError("improper: more zeros than poles")
                st.setText("off" if tf.is_zero else
                           f"✓ order {tf.order}" if tf.order else "✓ gain")
                st.setStyleSheet(f"color:{GREEN if not tf.is_zero else MUTED};")
                st.setToolTip("")
            except ValueError as exc:
                ok = False
                st.setText("✕ error")
                st.setStyleSheet(f"color:{RED};")
                st.setToolTip(str(exc))
        self.stress_btn.setEnabled(ok)
        if not ok:
            self.design = None
            self.badge.set("ERROR", "fix the red block (hover for why)")
            return
        tau, act, theta = self.tau.value(), self.act.value(), self.delay.value()
        w, g, rep = tc.analyse(d, tau, theta, act)
        self.design, self.report = d, rep
        verdict = ("string-stable" if rep.string_stable and rep.internally_stable
                   else "string-unstable")
        self.badge.set(verdict, f"peak |Γ| {rep.peak_gain:.3f}"
                       + ("" if rep.internally_stable else " · loop UNSTABLE"))
        self._draw_bode(d, w, g, rep)
        self._draw_poles(rep)
        self.physics.setPlainText(self._physics_text(d, rep, tau, act, theta))

    def _draw_bode(self, d, w, g, rep) -> None:
        ax, bx = self.bode.clear()
        ax.semilogx(w, g, color=INK, lw=1.6, label="|Γ(jω)| per hop")
        ax.axhline(1.0, color=RED, ls="--", lw=1, label="string-stability limit")
        ax.plot([rep.peak_omega], [rep.peak_gain], "o", color=RED if rep.peak_gain > 1 + 1e-6
                else GREEN)
        ax.set_ylabel("|Γ(jω)|")
        ax.set_title("A wave the car ahead makes, as this car repeats it "
                     "(above 1 = amplified down the platoon)", fontsize=9, color=MUTED)
        ax.legend(fontsize=7, frameon=False)
        s = 1j * w
        for sig, tf in d.tfs().items():
            if not tf.is_zero:
                bx.loglog(w, np.abs(tf(s)), color=BLOCK_COLORS[sig],
                          label=tc.SIGNAL_LABELS[sig])
        bx.set_ylabel("|K(jω)|")
        bx.set_xlabel("ω [rad/s]")
        bx.legend(fontsize=7, frameon=False)
        self.bode.draw()

    def _draw_poles(self, rep) -> None:
        (ax,) = self.poles.clear()
        p = rep.poles
        ax.axvline(0, color=RED, lw=1)
        ax.axhline(0, color="#c3c2b7", lw=0.8)
        ax.plot(p.real, p.imag, "x", ms=9, mew=2,
                color=GREEN if rep.internally_stable else RED)
        span = max(1.0, float(np.max(np.abs(p))) * 1.2) if p.size else 1.0
        ax.set_xlim(-span, max(0.2 * span, float(p.real.max()) * 1.2 if p.size else 0))
        ax.set_ylim(-span / 2, span / 2)
        ax.set_xlabel("Re s [1/s]")
        ax.set_ylabel("Im s [rad/s]")
        ax.set_title("Closed-loop poles: all left of the red line = the car settles; "
                     "slowest pole sets how long it takes", fontsize=9, color=MUTED)
        self.poles.draw()

    @staticmethod
    def _physics_text(d, rep, tau, act, theta) -> str:
        b = d.blocks
        phi = "s²·A_{i-1}  (realized accel)" if d.ff_signal == "a" else "U_{i-1}  (commanded accel)"
        slow = min((-p.real for p in rep.poles if p.real < 0), default=float("nan"))
        return "\n".join([
            "CONTROLLER (what you typed)",
            f"  u = [{b['e']}]·e + [{b['e_dot']}]·ė",
            f"      + [{b['dv']}]·Δv + [{b['u_ff']}]·u_ff",
            f"  {rep.n_states} internal state(s), realized in state space",
            "",
            "SIGNALS the car measures",
            f"  e   = gap − (r + h·v)       spacing error, h = {d.h:g} s, r = {d.r:g} m",
            "  ė   = Δv − h·a              its rate (radar + IMU)",
            "  Δv  = v_ahead − v           relative speed (radar)",
            f"  u_ff = e^(−θs)·{phi}, θ = {theta:g} s",
            "",
            "PLANT (the car)",
            f"  a = e^(−{act:g}s)/({tau:g}s + 1) · u     powertrain lag + dead time",
            "  then a → v → position (two integrators); the actuator clips u",
            "  to the EV's grip / motor-power limits",
            "",
            "PLATOON (per hop)",
            "  Γ(s) = G(K_E + sK_Δv + K_ff e^(−θs)φ) / (1 + G(K_E(1+hs) + sK_Δv))",
            "  G = e^(−θa s)/(s²(τs+1)),  K_E = K_e + s·K_ė",
            f"  peak |Γ| = {rep.peak_gain:.4f} at ω = {rep.peak_omega:.3g} rad/s"
            f"  → {'damped' if rep.string_stable else 'AMPLIFIED'} down the line",
            f"  slowest pole {slow:.3g} 1/s → settles in ≈ {4 / slow:.0f} s"
            if slow == slow and slow > 0 else "  loop has a pole on/right of the axis",
        ])

    # ------------------------------------------------------------ inside
    def run_inside(self) -> None:
        if self.design is None:
            return
        self.run_btn.setEnabled(False)
        self.inside_status.setText("simulating…")
        run_job(simulate, self.design, self.man.currentText(), self.tau.value(),
                self.act.value(), self.delay.value(), self.follower.value(),
                on_done=self.show_inside, on_error=self._sim_failed)

    def _sim_failed(self, msg: str) -> None:
        self.run_btn.setEnabled(True)
        self.inside_status.setText("")
        QMessageBox.critical(self, "Simulation failed", msg)

    def show_inside(self, out) -> None:
        res, parts = out
        self.run_btn.setEnabled(True)
        i = self.follower.value()
        gap_ax, v_ax, u_ax = self.trace.clear()
        t, n = res.t, res.vel.shape[1] - 1
        cols = vehicle_colors(n + 1)
        gaps = -np.diff(res.pos, axis=1) - res.config.vehicle.length
        for k in range(n):
            lw = 2.0 if k + 1 == i else 0.9
            gap_ax.plot(t, gaps[:, k], color=cols[k + 1], lw=lw)
        for k in range(n + 1):
            v_ax.plot(t, res.vel[:, k] * 3.6, color=cols[k], lw=2.0 if k == i else 0.9)
        for sig in tc.SIGNALS:
            if sig in parts:
                u_ax.plot(t, parts[sig], color=BLOCK_COLORS[sig], lw=1,
                          label=tc.SIGNAL_LABELS[sig])
        u_ax.plot(t, parts["sum"], color=INK, ls="--", lw=1, label="controller asks")
        u_ax.plot(t, parts["u"], color=INK, lw=2, label="actuator gets")
        clip = np.abs(parts["sum"] - parts["u"]) > 1e-3
        if clip.any():
            u_ax.fill_between(t, parts["sum"], parts["u"], where=clip, color=RED,
                              alpha=0.25, label="clipped (grip / power)")
        gap_ax.set_ylabel("gap [m]")
        v_ax.set_ylabel("speed [km/h]")
        u_ax.set_ylabel(f"car {i} command [m/s²]")
        u_ax.set_xlabel("time [s]")
        u_ax.legend(fontsize=7, frameon=False, ncol=3)
        self.trace.draw()
        self.inside_status.setText(
            f"min gap {gaps.min():.2f} m · car {i} peak brake {-parts['u'].min():.2f} m/s²"
            + (f" · clipped {clip.mean():.0%} of the time" if clip.any() else ""))

    # ------------------------------------------------------------ files
    def _open(self) -> None:
        p, _ = QFileDialog.getOpenFileName(self, "Open design", "", "Design (*.yaml *.yml)")
        if p:
            try:
                self.load(tc.Design.load(p))
            except (ValueError, OSError) as exc:
                QMessageBox.warning(self, "Open design", str(exc))

    def _save(self) -> str | None:
        p, _ = QFileDialog.getSaveFileName(self, "Save design", "my_controller.yaml",
                                           "Design (*.yaml)")
        if p:
            self.current().save(p)
        return p or None

    def _matlab(self) -> None:
        if self.design is None:
            return
        p, _ = QFileDialog.getSaveFileName(self, "Export to MATLAB",
                                           "my_controller.m", "MATLAB (*.m)")
        if p:
            from pathlib import Path

            Path(p).write_text(tc.to_matlab(self.design, self.tau.value(),
                                            self.delay.value()), encoding="utf-8")

    def _tune(self) -> None:
        if self.design is None:
            return
        self.tune_btn.setEnabled(False)
        self.tune_btn.setText("tuning…")
        run_job(run_tune, self.design, self.tau.value(), self.act.value(),
                self.delay.value(), on_done=self._tuned, on_error=self._tune_failed)

    def _tune_failed(self, msg: str) -> None:
        self.tune_btn.setEnabled(True)
        self.tune_btn.setText("Auto-tune…")
        QMessageBox.critical(self, "Auto-tune", msg)

    def _tuned(self, res) -> None:
        self.tune_btn.setEnabled(True)
        self.tune_btn.setText("Auto-tune…")
        if res.passed or res.feasible:
            self.load(res.design)
        QMessageBox.information(self, "Auto-tune", res.summary()
                                + ("" if res.passed else
                                   "\n\n" + "\n".join(res.log)))

    def _send(self) -> None:
        if self.design is not None:
            self.stress_design.emit(self.design)

