"""Telemetry panel of the Stress test page: one campaign cell, replayed.

Five stacked channels over time, like a race engineer's trace view: gap to
the car ahead, speed, acceleration, battery power and the trust weight of
the V2V gate (when it runs). A run can be *pinned*; the next cell you open
is drawn on top of it with the pinned run dashed, so two conditions (or two
controllers) compare trace by trace.
"""

from __future__ import annotations

import math

import numpy as np
from PySide6.QtWidgets import QGridLayout, QHBoxLayout, QPushButton, QVBoxLayout, QWidget

from cacc.gui.common import MUTED, RED, PlotCanvas, card, label, vehicle_colors

#: KPI tiles: key -> (title, unit, format)
KPIS = (("min_gap", "Min gap", "m", "{:.2f}"),
        ("min_ttc", "Min TTC", "s", "{:.2f}"),
        ("peak_decel", "Peak braking", "m/s²", "{:.2f}"),
        ("kwh_100", "Energy", "kWh/100 km", "{:.1f}"),
        ("regen", "Regen recovered", "%", "{:.0f}"),
        ("draft", "Drafting saved", "%", "{:.1f}"))


def kpis(res) -> dict:
    """Headline numbers of one run (worst follower)."""
    from cacc.criteria import METRICS
    from cacc.ev import energy_report

    rep = energy_report(res)
    solo, real = rep["kwh_solo"].sum(), rep["kwh"].sum()
    return {"min_gap": METRICS["min_gap"].fn(res),
            "min_ttc": METRICS["min_ttc"].fn(res),
            "peak_decel": METRICS["peak_decel"].fn(res),
            "kwh_100": float(np.nanmax(rep["kwh_per_100km"])),
            "regen": 100.0 * float(rep["regen_share"].min()),
            "draft": 100.0 * (solo - real) / solo if solo > 0 else 0.0}


class Telemetry(QWidget):
    def __init__(self):
        super().__init__()
        self.res, self.pinned, self.title = None, None, ""
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        head = QHBoxLayout()
        self.heading = label("Click a cell of the grid to replay it here.", "h2")
        self.pin_btn = QPushButton("Pin for comparison")
        self.pin_btn.setToolTip("Keep this run; the next cell you open is drawn "
                                "over it (pinned run dashed)")
        self.pin_btn.setEnabled(False)
        self.pin_btn.clicked.connect(self._pin)
        self.unpin_btn = QPushButton("Unpin")
        self.unpin_btn.setEnabled(False)
        self.unpin_btn.clicked.connect(self._unpin)
        head.addWidget(self.heading, 1)
        head.addWidget(self.pin_btn)
        head.addWidget(self.unpin_btn)
        lay.addLayout(head)

        tiles = QGridLayout()
        self.tiles = {}
        for i, (key, title, unit, _) in enumerate(KPIS):
            box, bl = card()
            bl.setSpacing(0)
            t = label(f"{title} [{unit}]", wrap=False)
            t.setStyleSheet(f"color:{MUTED}; font-size:8pt;")
            v = label("—", wrap=False)
            v.setStyleSheet("font-size:14pt; font-weight:600;")
            bl.addWidget(t)
            bl.addWidget(v)
            self.tiles[key] = v
            tiles.addWidget(box, 0, i)
        lay.addLayout(tiles)
        self.plot = PlotCanvas(rows=5, height=7.0)
        lay.addWidget(self.plot, 1)

    # ---------------------------------------------------------------- state
    def show_run(self, res, title: str) -> None:
        self.res, self.title = res, title
        self.pin_btn.setEnabled(True)
        k = kpis(res)
        ref = kpis(self.pinned[0]) if self.pinned else None
        for key, _t, _u, fmt in KPIS:
            v = k[key]
            txt = "never" if math.isinf(v) else fmt.format(v)
            if ref is not None and math.isfinite(v) and math.isfinite(ref[key]):
                txt += f"  ({v - ref[key]:+.2f})"
            self.tiles[key].setText(txt)
        self.tiles["min_gap"].setStyleSheet(
            "font-size:14pt; font-weight:600;"
            + (f"color:{RED};" if k["min_gap"] <= 0 else ""))
        self.heading.setText(title + (f"   vs pinned: {self.pinned[1]}"
                                      if self.pinned else ""))
        self._draw()

    def _pin(self) -> None:
        if self.res is not None:
            self.pinned = (self.res, self.title)
            self.unpin_btn.setEnabled(True)
            self.show_run(self.res, self.title)

    def _unpin(self) -> None:
        self.pinned = None
        self.unpin_btn.setEnabled(False)
        if self.res is not None:
            self.show_run(self.res, self.title)

    # ---------------------------------------------------------------- plots
    def _draw(self) -> None:
        from cacc.ev import EVParams, battery_power

        axes = self.plot.clear(rows=5)
        runs = [(self.res, "-", 1.0)]
        if self.pinned and self.pinned[0] is not self.res:
            runs.append((self.pinned[0], "--", 0.55))
        for res, ls, alpha in runs:
            n = res.vel.shape[1] - 1
            cols = vehicle_colors(n)
            gaps = -np.diff(res.pos, axis=1) - res.config.vehicle.length
            ev = res.config.vehicle.ev or EVParams()
            draft = ev.draft_scale(gaps)
            p_kw = battery_power(ev, res.vel[:, 1:], res.acc[:, 1:], draft) / 1e3
            for i in range(n + 1):
                kw = dict(color=cols[i], lw=1.0 if i else 1.3, ls=ls, alpha=alpha)
                axes[1].plot(res.t, res.vel[:, i] * 3.6, **kw)
                axes[2].plot(res.t, res.acc[:, i], **kw)
                if i:
                    axes[0].plot(res.t, gaps[:, i - 1], **kw)
                    axes[3].plot(res.t, p_kw[:, i - 1], **kw)
                    if res.trust is not None:
                        axes[4].plot(res.t, res.trust[:, i - 1], **kw)
            if res.trust is None and ls == "-":
                axes[4].text(0.5, 0.5, "trust gate off in this run",
                             transform=axes[4].transAxes, ha="center",
                             va="center", color=MUTED, fontsize=8)
        axes[0].axhline(0.0, color=RED, lw=1.0)
        for ax, ylab in zip(axes, ("gap [m]", "speed [km/h]", "accel [m/s²]",
                                   "battery [kW]", "V2V trust"), strict=True):
            ax.set_ylabel(ylab, fontsize=8, color=MUTED)
        axes[4].set_ylim(-0.05, 1.05)
        axes[-1].set_xlabel("time [s]", fontsize=8, color=MUTED)
        self.plot.draw()
