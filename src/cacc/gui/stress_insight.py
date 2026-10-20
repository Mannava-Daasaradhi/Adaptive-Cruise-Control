"""Stress page panels: *Why it failed* and *Breaking point*.

``WhyPanel`` shows :func:`cacc.explain.explain` for the opened cell: what
broke, the physics behind it, and candidate fixes re-simulated on the same
seed. ``BreakPanel`` runs :mod:`cacc.envelope` for the controller under
test: the breaking point of every stress axis (a tornado chart of how much
of each axis the controller survives) or a two-axis pass/fail map.
"""

from __future__ import annotations

import html
import math

import numpy as np
from PySide6.QtWidgets import (QComboBox, QHBoxLayout, QMessageBox, QPushButton, QSpinBox,
                               QTextBrowser, QVBoxLayout, QWidget)

from cacc.gui.common import GREEN, MUTED, RED, PlotCanvas, label, run_job


def run_explain(report, case_name, controller, options, seed, res, jobs):
    from cacc.explain import explain

    return explain(report, case_name, controller, options, seed, res=res, jobs=jobs)


def run_profile(controller, options, block, maneuver, criteria, jobs):
    from cacc.envelope import profile

    return profile(controller, maneuver, options=options, controller_block=block,
                   criteria=criteria, jobs=jobs)


def run_map(controller, options, block, maneuver, ax, ay, n, criteria, jobs):
    from cacc.envelope import envelope

    return envelope(controller, maneuver, ax, ay, n, criteria, options=options,
                    controller_block=block, jobs=jobs)


class WhyPanel(QWidget):
    def __init__(self):
        super().__init__()
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        self.text = QTextBrowser()
        self.text.setOpenExternalLinks(False)
        lay.addWidget(self.text)
        self.text.setHtml(f"<p style='color:{MUTED}'>Open a failed cell: this tab "
                          "explains what broke, why, and which change fixes it "
                          "(each fix re-simulated on the same seed).</p>")

    def busy(self, name: str) -> None:
        self.text.setHtml(f"<h3>{html.escape(name)}</h3><p style='color:{MUTED}'>"
                          "working out the root cause and trying fixes…</p>")

    def passed(self, cell: dict) -> None:
        m = cell["margin"]
        self.text.setHtml(
            f"<h3>{html.escape(cell['name'])}: passes</h3><p>Tightest criterion: "
            f"{html.escape(cell['binding'])}, margin "
            f"{m:+.0%}.</p>" if math.isfinite(m) else "<p>passes</p>")

    def show(self, ex: dict) -> None:
        e = html.escape
        parts = [f"<h3 style='color:{RED}'>{e(ex['headline'])}</h3>",
                 f"<p style='color:{MUTED}'>{e(ex['case'])} · seed {ex['seed']}</p>"]
        if ex["findings"]:
            parts.append(f"<p>{e(ex['findings'][0]['detail'])}</p><h4>Why</h4><ul>")
            for f in ex["findings"][1:]:
                parts.append(f"<li><b>{e(f['title'])}</b><br>{e(f['detail'])}</li>")
            parts.append("</ul>")
        if ex["fixes"]:
            parts.append("<h4>Fixes tried on the same seed</h4><table cellpadding=4>")
            for f in ex["fixes"]:
                ok = f["verdict"] == "PASS"
                m = f["margin"]
                parts.append(f"<tr><td style='color:{GREEN if ok else RED}'>"
                             f"{'✓ fixes it' if ok else '✕ not enough'}</td>"
                             f"<td>{e(f['change'])}</td><td style='color:{MUTED}'>"
                             + (f"{m:+.0%}" if math.isfinite(m) else "") + "</td></tr>")
            parts.append("</table>")
        self.text.setHtml("".join(parts))


class BreakPanel(QWidget):
    """``context()`` returns (controller, options, block, criteria, jobs)
    for the controller under test, or None (the page shows why)."""

    def __init__(self, context):
        from cacc import campaign as cp
        from cacc.envelope import AXES

        super().__init__()
        self.context = context
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        row = QHBoxLayout()
        self.man = QComboBox()
        for k, v in cp.MANEUVERS.items():
            self.man.addItem(v[0], k)
        self.man.setCurrentIndex(self.man.findData("hard-brake"))
        self.all_btn = QPushButton("Find breaking points (all axes)")
        self.all_btn.clicked.connect(self.find_all)
        self.ax, self.ay = QComboBox(), QComboBox()
        for k, a in AXES.items():
            self.ax.addItem(a.label, k)
            self.ay.addItem(a.label, k)
        self.ay.setCurrentIndex(self.ay.findData("grip"))
        self.n = QSpinBox()
        self.n.setRange(3, 15)
        self.n.setValue(7)
        self.map_btn = QPushButton("Map envelope")
        self.map_btn.clicked.connect(self.map)
        row.addWidget(label("Maneuver", wrap=False))
        row.addWidget(self.man)
        row.addWidget(self.all_btn)
        row.addStretch(1)
        row.addWidget(self.ax)
        row.addWidget(label("vs", wrap=False))
        row.addWidget(self.ay)
        row.addWidget(self.n)
        row.addWidget(self.map_btn)
        lay.addLayout(row)
        self.status = label("Drives each stress axis from benign to harsh and "
                            "bisects to the exact point where the controller "
                            "first fails.", "subtitle")
        lay.addWidget(self.status)
        self.plot = PlotCanvas(rows=1, height=4.0, sharex=False)
        lay.addWidget(self.plot, 1)
        self.rows = None
        self.grid = None

    def _busy(self, on: bool, text: str = "") -> None:
        self.all_btn.setEnabled(not on)
        self.map_btn.setEnabled(not on)
        if text:
            self.status.setText(text)

    def _failed(self, msg: str) -> None:
        self._busy(False, "")
        QMessageBox.critical(self, "Breaking point", msg)

    def find_all(self) -> None:
        ctx = self.context()
        if ctx is None:
            return
        ctrl, opts, block, crit, jobs = ctx
        self._busy(True, "searching every axis (coarse scan + bisection)…")
        run_job(run_profile, ctrl, opts, block, self.man.currentData(), crit, jobs,
                on_done=self.show_profile, on_error=self._failed)

    def map(self) -> None:
        ctx = self.context()
        if ctx is None or self.ax.currentData() == self.ay.currentData():
            return
        ctrl, opts, block, crit, jobs = ctx
        n = self.n.value()
        self._busy(True, f"mapping {n}×{n} = {n * n} cases…")
        run_job(run_map, ctrl, opts, block, self.man.currentData(),
                self.ax.currentData(), self.ay.currentData(), n, crit, jobs,
                on_done=self.show_map, on_error=self._failed)

    def show_profile(self, rows: list[dict]) -> None:
        self.rows = rows
        self._busy(False, "")
        (ax,) = self.plot.clear(rows=1)
        y = np.arange(len(rows))[::-1]
        fr = [r["fraction"] for r in rows]
        cols = [RED if r["status"] == "fails" else GREEN if r["status"] == "holds"
                else "#e0a030" for r in rows]
        ax.barh(y, fr, color=cols, height=0.6)
        ax.set_yticks(y, [r["label"] for r in rows], fontsize=8)
        ax.set_xlim(0, 1.0)
        ax.set_xlabel("share of the axis range survived (benign → harsh)", fontsize=8)
        for yy, r in zip(y, rows, strict=True):
            txt = ("holds" if r["status"] == "holds" else "fails at start"
                   if r["status"] == "fails" else
                   f"breaks at {r['value']:.3g} {r['unit']}".strip())
            ax.text(min(r["fraction"], 0.98) + 0.01 if r["fraction"] < 0.75 else 0.02,
                    yy, txt, va="center", fontsize=8)
        ax.set_title(f"Breaking points · {self.man.currentText()}", fontsize=9,
                     color=MUTED)
        self.plot.draw()
        weakest = rows[0]
        self.status.setText(f"Weakest: {weakest['label']} — {weakest['text']}")

    def show_map(self, env: dict) -> None:
        self.grid = env
        self._busy(False, "")
        (ax,) = self.plot.clear(rows=1)
        m = np.clip(np.where(np.isfinite(env["margin"]), env["margin"], -1.0), -1, 1)
        x, y = np.asarray(env["x"]), np.asarray(env["y"])
        im = ax.pcolormesh(x, y, m, cmap="RdYlGn", vmin=-1, vmax=1, shading="nearest")
        if (m > 0).any() and (m < 0).any():
            ax.contour(x, y, m, levels=[0.0], colors="#1f2328", linewidths=1.5)
        ax.set_xlabel(env["x_label"], fontsize=8)
        ax.set_ylabel(env["y_label"], fontsize=8)
        ax.set_title(f"Safety envelope · {self.man.currentText()} · black line = "
                     "breaking boundary", fontsize=9, color=MUTED)
        self.plot.fig.colorbar(im, ax=ax, label="margin of the tightest criterion")
        self.plot.draw()
        self.status.setText(f"{env['pass_share']:.0%} of the map passes")
