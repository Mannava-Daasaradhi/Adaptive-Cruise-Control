"""Audit page: drive log in -> per-car verdict, twin fit, V2V what-if, report."""

from __future__ import annotations

from pathlib import Path

import numpy as np
from PySide6.QtWidgets import (QAbstractItemView, QDoubleSpinBox, QFormLayout, QHBoxLayout,
                               QHeaderView, QMessageBox, QPushButton, QSplitter,
                               QTableWidget, QTableWidgetItem, QTabWidget)
from PySide6.QtCore import Qt

from cacc.gui.common import (BLUE, MUTED, ORANGE, VERDICT_STYLE, AQUA, FilePicker, Page,
                             PlotCanvas, VerdictBadge, card, label, open_path, output_dir,
                             primary_button, resource, run_job)

COLUMNS = ("Vehicle", "Verdict", "Time gap in use", "Needs at least", "‖Γ‖∞",
           "Recommended gap", "Best V2V (ka, latency budget)")


def run_audit(log: str, min_duration: float, ka: float, delay: float, out: Path):
    """Background job: calibrate every follower and write the report files."""
    from cacc.audit import audit_log, write_audit

    res = audit_log(log, min_duration, v2v_ka=ka, v2v_delay=delay)
    paths = write_audit(res, out) if res.vehicles else {}
    return res, paths


def _fmt(x, fmt="{:.2f} s", none="—") -> str:
    return none if x is None else fmt.format(x)


class AuditPage(Page):
    def __init__(self):
        super().__init__(
            "String-Stability Audit",
            "Load a car-following drive log (OpenACC CSV or a generic platoon log). "
            "Each ACC car gets a digital twin, a string-stability verdict with "
            "uncertainty, and a what-if for V2V communication. A customer-ready "
            "HTML report is written next to the twins and release gates.")
        self.result, self.paths = None, {}

        box, form_lay = card()
        form = QFormLayout()
        self.log = FilePicker("Drive log", "Drive logs (*.csv);;All files (*)")
        sample = QPushButton("Use sample log")
        sample.clicked.connect(self._use_sample)
        sample.setEnabled(resource("examples", "data") is not None)
        row = QHBoxLayout()
        row.addWidget(self.log, 1)
        row.addWidget(sample)
        form.addRow("Drive log", row)
        self.min_dur = self._spin(30.0, 5.0, 600.0, 5.0, " s")
        self.ka = self._spin(0.5, 0.0, 1.0, 0.1, "")
        self.delay = self._spin(100.0, 0.0, 1000.0, 10.0, " ms", decimals=0)
        opts = QHBoxLayout()
        for text, w in (("Shortest ACC segment", self.min_dur),
                        ("What-if V2V gain ka", self.ka), ("V2V latency", self.delay)):
            opts.addWidget(label(text, wrap=False))
            opts.addWidget(w)
            opts.addSpacing(12)
        opts.addStretch(1)
        form.addRow("Options", opts)
        form_lay.addLayout(form)
        actions = QHBoxLayout()
        self.run_btn = primary_button("Run audit")
        self.run_btn.clicked.connect(self._run)
        self.status = label("", "subtitle")
        self.badge = VerdictBadge()
        self.report_btn = QPushButton("Open report")
        self.folder_btn = QPushButton("Open folder")
        self.report_btn.clicked.connect(lambda: open_path(self.paths["report"]))
        self.folder_btn.clicked.connect(lambda: open_path(self.paths["report"].parent))
        for b in (self.report_btn, self.folder_btn):
            b.setEnabled(False)
        actions.addWidget(self.run_btn)
        actions.addWidget(self.status, 1)
        actions.addWidget(self.badge)
        actions.addWidget(self.report_btn)
        actions.addWidget(self.folder_btn)
        form_lay.addLayout(actions)
        self.body.addWidget(box)

        split = QSplitter(Qt.Vertical)
        self.table = QTableWidget(0, len(COLUMNS))
        self.table.setHorizontalHeaderLabels(COLUMNS)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.verticalHeader().hide()
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.itemSelectionChanged.connect(self._show_vehicle)
        split.addWidget(self.table)
        self.tabs = QTabWidget()
        self.fit_plot = PlotCanvas(rows=2, height=3.2)
        self.gain_plot = PlotCanvas(rows=1, height=3.0)
        self.tabs.addTab(self.gain_plot, "String stability (gain)")
        self.tabs.addTab(self.fit_plot, "Twin vs. measured")
        self.detail = label("", "subtitle")
        split.addWidget(self.tabs)
        split.setSizes([160, 380])
        self.body.addWidget(split, 1)
        self.body.addWidget(self.detail)

    @staticmethod
    def _spin(val, lo, hi, step, suffix, decimals=2) -> QDoubleSpinBox:
        s = QDoubleSpinBox()
        s.setRange(lo, hi)
        s.setSingleStep(step)
        s.setDecimals(decimals)
        s.setValue(val)
        s.setSuffix(suffix)
        return s

    def _use_sample(self) -> None:
        p = resource("examples", "data", "synthetic_openacc_platoon.csv")
        if p:
            self.log.set_path(p)

    # ------------------------------------------------------------ run
    def _run(self) -> None:
        log = self.log.path()
        if not log or not Path(log).is_file():
            QMessageBox.warning(self, "Audit", "Choose a drive-log CSV first.")
            return
        out = output_dir("audits", Path(log).stem)
        self.run_btn.setEnabled(False)
        self.badge.set(None)
        self.status.setText("Calibrating digital twins… (about 10–30 s per log)")
        run_job(run_audit, log, self.min_dur.value(), self.ka.value(),
                self.delay.value() / 1000.0, out,
                on_done=self._done, on_error=self._failed)

    def _failed(self, msg: str) -> None:
        self.run_btn.setEnabled(True)
        self.status.setText("Audit failed.")
        QMessageBox.critical(self, "Audit failed", msg)

    def _done(self, out) -> None:
        self.result, self.paths = out
        res = self.result
        self.run_btn.setEnabled(True)
        n_bad = sum(v.twin.verdict == "string-unstable" for v in res.vehicles)
        if not res.vehicles:
            self.status.setText("No ACC-engaged segment long enough to calibrate. "
                                + " · ".join(res.skipped))
            return
        self.status.setText(f"{len(res.vehicles)} car(s) audited · report saved")
        self.status.setToolTip(str(self.paths["report"]))
        self.badge.set("string-unstable" if n_bad else "string-stable",
                       f"{n_bad} of {len(res.vehicles)} unstable" if n_bad else "")
        for b in (self.report_btn, self.folder_btn):
            b.setEnabled(True)
        self._fill_table()

    def _fill_table(self) -> None:
        vs = self.result.vehicles
        self.table.setRowCount(len(vs))
        for i, v in enumerate(vs):
            r = v.twin
            best = v.best_v2v
            cells = (r.follower, VERDICT_STYLE[r.verdict][2], _fmt(r.model.T),
                     _fmt(r.min_time_gap, none="unstable at any gap"),
                     f"{r.hinf:.3f}", _fmt(v.recommended_T),
                     "none works" if best is None else
                     f"ka {best[0]:g}, up to {best[1] * 1000:.0f} ms")
            for j, text in enumerate(cells):
                item = QTableWidgetItem(text)
                if j == 1:
                    item.setForeground(Qt.GlobalColor.darkRed if r.verdict == "string-unstable"
                                       else Qt.GlobalColor.darkGreen)
                self.table.setItem(i, j, item)
        self.table.selectRow(0)

    # ------------------------------------------------------------ plots
    def _show_vehicle(self) -> None:
        rows = self.table.selectionModel().selectedRows()
        if not rows or self.result is None:
            return
        v = self.result.vehicles[rows[0].row()]
        self._plot_gain(v)
        self._plot_fit(v)
        r = v.twin
        m = r.model
        self.detail.setText(
            f"{r.follower} behind {r.leader}: {r.duration_s:.0f} s of ACC driving in "
            f"{r.segments} segment(s), mean speed {r.v_mean:.1f} m/s. Twin fit: gap RMSE "
            f"{r.gap_rmse:.2f} m, speed RMSE {r.speed_rmse:.2f} m/s. Fitted gains "
            f"k_s={m.k_s:.3g}, k_v={m.k_v:.3g}, lag τ={m.tau:.2f} s.")

    def _plot_gain(self, v) -> None:
        from cacc import gamma_magnitude

        r, m = v.twin, v.twin.model
        (ax,) = self.gain_plot.clear()
        w = np.logspace(-2, np.log10(3.0), 400)
        ka, th = r.what_if["ka"], r.what_if["delay_s"]
        ax.axhline(1.0, color=MUTED, lw=1.0, ls=(0, (4, 3)), label="limit |Γ| = 1")
        ax.semilogx(w, m.gain(w), color=BLUE, lw=1.8, label="as driven (radar only)")
        ax.semilogx(w, gamma_magnitude(w, "cthp", m.T, kp=m.k_s, kv=m.k_v, ka_eff=ka,
                                       tau=m.tau, theta=th),
                    color=ORANGE, lw=1.8, label=f"with V2V ka={ka:g}, {th * 1000:.0f} ms")
        emp = r.empirical
        if emp.get("peak_gain") is not None:
            ax.plot([emp["peak_omega_rad_s"]], [emp["peak_gain"]], "D", color=AQUA,
                    ms=7, label="model-free peak (from data)")
        ax.set_xlabel("frequency ω [rad/s]", fontsize=9, color=MUTED)
        ax.set_ylabel("speed gain |Γ(jω)|", fontsize=9, color=MUTED)
        ax.set_title("Above the dashed line, this car amplifies speed waves from the "
                     "car ahead", fontsize=9, color=MUTED, loc="left")
        ax.legend(frameon=False, fontsize=8)
        self.gain_plot.draw()

    def _plot_fit(self, v) -> None:
        tr = v.trace
        a1, a2 = self.fit_plot.clear()
        t = tr["t"] - tr["t"][0]
        for ax, meas, twin, unit in ((a1, tr["gap"], tr["gap_twin"], "gap [m]"),
                                     (a2, tr["speed"], tr["speed_twin"], "speed [m/s]")):
            ax.plot(t, meas, color=BLUE, lw=1.3, label="measured")
            ax.plot(t, twin, color=ORANGE, lw=1.3, label="digital twin")
            ax.set_ylabel(unit, fontsize=9, color=MUTED)
        a1.legend(frameon=False, fontsize=8, ncol=2)
        a2.set_xlabel("time in longest ACC segment [s]", fontsize=9, color=MUTED)
        self.fit_plot.draw()
