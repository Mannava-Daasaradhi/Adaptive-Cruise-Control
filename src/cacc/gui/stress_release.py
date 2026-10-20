"""Stress page panels: *Energy & comfort* and *Versions*.

``EnergyPanel`` scores the opened cell (:func:`cacc.energy.score`): range
each car loses against the lead car, regen vs friction braking, jerk.
``VersionsPanel`` saves campaign results as named versions
(:mod:`cacc.history`) and shows every cell's verdict across them, with the
diff of the last two: regressions first.
"""

from __future__ import annotations

import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (QAbstractItemView, QHBoxLayout, QLineEdit, QPlainTextEdit,
                               QPushButton, QSplitter, QTableWidget, QTableWidgetItem,
                               QVBoxLayout, QWidget)

from cacc.gui.common import BLUE, GREEN, MUTED, ORANGE, RED, PlotCanvas, label

CELL_BG = {"PASS": "#dff2df", "FAIL": "#f6d5d5", "ERROR": "#eeeeee", "": "#ffffff"}


class EnergyPanel(QWidget):
    def __init__(self):
        super().__init__()
        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 4, 4, 4)
        self.summary = label("Open a cell: range lost per car against the lead car, "
                             "regen vs friction braking and ride comfort.", "subtitle")
        lay.addWidget(self.summary)
        self.plot = PlotCanvas(rows=3, height=5.0, sharex=True)
        lay.addWidget(self.plot, 1)

    def set_result(self, res) -> None:
        from cacc.energy import score, summary

        s, tot = score(res), summary(res)
        n = res.vel.shape[1]
        x = np.arange(n)
        names = ["lead"] + [str(i) for i in range(1, n)]
        a1, a2, a3 = self.plot.clear(rows=3)
        lost = s["range_lost_pct"]
        a1.bar(x, lost, color=[MUTED] + [RED if v > 0 else GREEN for v in lost[1:]])
        a1.axhline(0, color=MUTED, lw=0.8)
        a1.set_ylabel("extra kWh vs lead [%]", fontsize=8)
        a1.set_title(f"{tot['kwh_per_100km']:.1f} kWh/100 km · range "
                     f"{tot['range_km']:.0f} km · worst car "
                     f"{tot['range_lost_pct']:+.1f} % vs the lead car",
                     fontsize=9, color=MUTED)
        a2.bar(x - 0.2, s["regen_kwh"] * 1e3, 0.4, color=BLUE, label="regen recovered")
        a2.bar(x + 0.2, s["friction_kwh"] * 1e3, 0.4, color=ORANGE,
               label="burnt in friction brakes")
        a2.set_ylabel("braking energy [Wh]", fontsize=8)
        a2.legend(fontsize=7, frameon=False)
        a3.bar(x - 0.2, s["rms_jerk"], 0.4, color=BLUE, label="RMS jerk [m/s³]")
        a3.bar(x + 0.2, 10 * s["harsh_share"], 0.4, color=RED,
               label="time beyond ISO 15622 comfort [×10 %]")
        a3.set_xticks(x, names, fontsize=8)
        a3.set_xlabel("car", fontsize=8)
        a3.legend(fontsize=7, frameon=False)
        self.plot.draw()
        self.summary.setText(
            f"Regen sent back {tot['regen_kwh'] * 1e3:.0f} Wh, the friction brakes "
            f"burnt {tot['friction_kwh'] * 1e3:.0f} Wh. Worst RMS jerk "
            f"{tot['rms_jerk']:.2f} m/s³; {tot['harsh_share']:.0%} of the run beyond "
            "the ISO 15622 comfort limits.")


class VersionsPanel(QWidget):
    def __init__(self, store_fn):
        super().__init__()
        self.store_fn = store_fn  # () -> history folder
        self.report = None
        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 4, 4, 4)
        row = QHBoxLayout()
        self.name = QLineEdit()
        self.name.setPlaceholderText("version label, e.g. v1.3 or gains-retune")
        self.save_btn = QPushButton("Save this run as a version")
        self.save_btn.setEnabled(False)
        self.save_btn.clicked.connect(self.save)
        row.addWidget(self.name, 1)
        row.addWidget(self.save_btn)
        lay.addLayout(row)
        split = QSplitter(Qt.Vertical)
        self.table = QTableWidget(0, 0)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.diff = QPlainTextEdit()
        self.diff.setReadOnly(True)
        split.addWidget(self.table)
        split.addWidget(self.diff)
        lay.addWidget(split, 1)
        self.refresh()

    def set_report(self, report) -> None:
        self.report = report
        self.save_btn.setEnabled(report is not None)

    def save(self) -> None:
        from cacc import history as hi

        if self.report is None:
            return
        label_ = self.name.text().strip() or f"v{len(hi.versions(self.store_fn())) + 1}"
        hi.save(self.report, label_, self.store_fn())
        self.name.clear()
        self.refresh()

    def refresh(self) -> None:
        from cacc import history as hi

        vs, names, grid = hi.table(self.store_fn())
        self.table.clear()
        self.table.setRowCount(len(names))
        self.table.setColumnCount(len(vs))
        self.table.setHorizontalHeaderLabels([v["label"] for v in vs])
        self.table.setVerticalHeaderLabels(names)
        for r, row in enumerate(grid):
            for c, verdict in enumerate(row):
                it = QTableWidgetItem({"PASS": "✓", "FAIL": "✕"}.get(verdict, verdict))
                it.setTextAlignment(Qt.AlignCenter)
                it.setBackground(QColor(CELL_BG.get(verdict, "#ffffff")))
                self.table.setItem(r, c, it)
        if len(vs) >= 2:
            rows = hi.diff(vs[-2], vs[-1])
            self.diff.setPlainText(hi.format_diff(vs[-2], vs[-1], rows))
        else:
            self.diff.setPlainText("Save two versions to see what changed between them: "
                                   "regressions (pass → fail) first.")
