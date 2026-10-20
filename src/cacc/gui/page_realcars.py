"""Real cars page: how production ACC cars behave in recorded drives.

Point it at a folder of car-following logs (OpenACC, microSIM-ACC or the
generic CSV) or download the public microSIM-ACC runs, and
:mod:`cacc.fieldstudy` measures every follower model-free: did it brake
harder than the car ahead (amplification), how far its speed undershot, its
smallest gap, grouped by vehicle or ACC gap setting.
"""

from __future__ import annotations

import os
from pathlib import Path

from PySide6.QtWidgets import (QAbstractItemView, QFileDialog, QHBoxLayout, QHeaderView,
                               QLineEdit, QMessageBox, QProgressBar, QPushButton,
                               QSplitter, QTableWidget, QTableWidgetItem)
from PySide6.QtCore import QObject, Qt, Signal

from cacc.gui.common import (BLUE, GREEN, RED, Page, PlotCanvas, card, label, open_path,
                             output_dir, primary_button, run_job)

COLS = (("group", "Group"), ("runs", "Runs"),
        ("amplified_share", "Braked harder"),
        ("median_amplification", "Median amplification"),
        ("worst_amplification", "Worst"), ("median_undershoot_mps", "Speed undershoot [m/s]"),
        ("min_gap_m", "Min gap [m]"))


def run_study(folder, fetch, jobs, out, progress):
    from cacc import fieldstudy as fs

    source = str(folder)
    if fetch:
        fs.fetch_microsim(folder)
        source = "microSIM-ACC ICE (FAU): 2-car commercial ACC runs"
    rows, problems = fs.study(folder, jobs=jobs, twin=False, progress=progress)
    if not rows:
        raise ValueError(f"no usable runs under {folder}")
    agg = fs.aggregate(rows)
    fs.write(rows, agg, problems, out, source)
    return rows, agg, out


class _Progress(QObject):
    tick = Signal(int, int)


class RealCarsPage(Page):
    def __init__(self):
        super().__init__(
            "Real cars",
            "How do production ACC cars really behave? Load a folder of recorded "
            "car-following drives (OpenACC, microSIM-ACC or a generic CSV), or "
            "download the public microSIM-ACC runs (358 runs of a commercial ACC "
            "car braking behind a lead car). Every follower is measured "
            "model-free: did it brake harder than the car ahead?")
        self.out = None
        self.progress = _Progress()
        self.progress.tick.connect(self._tick)
        top, tl = card()
        row = QHBoxLayout()
        self.folder = QLineEdit()
        self.folder.setPlaceholderText("folder of drive logs (*.csv)")
        browse = QPushButton("Browse…")
        browse.clicked.connect(self._browse)
        row.addWidget(label("Logs", wrap=False))
        row.addWidget(self.folder, 1)
        row.addWidget(browse)
        tl.addLayout(row)
        act = QHBoxLayout()
        self.run_btn = primary_button("Analyse folder")
        self.run_btn.clicked.connect(lambda: self._run(False))
        self.fetch_btn = QPushButton("Download microSIM-ACC runs and analyse")
        self.fetch_btn.clicked.connect(lambda: self._run(True))
        self.bar = QProgressBar()
        self.bar.hide()
        self.status = label("", "subtitle")
        self.open_btn = QPushButton("Open report")
        self.open_btn.setEnabled(False)
        self.open_btn.clicked.connect(lambda: open_path(self.out / "summary.md"))
        for w in (self.run_btn, self.fetch_btn):
            act.addWidget(w)
        act.addWidget(self.bar, 1)
        act.addWidget(self.status, 1)
        act.addWidget(self.open_btn)
        tl.addLayout(act)
        self.body.addWidget(top)
        split = QSplitter(Qt.Vertical)
        self.table = QTableWidget(0, len(COLS))
        self.table.setHorizontalHeaderLabels([c[1] for c in COLS])
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.plot = PlotCanvas(rows=1, height=3.0)
        split.addWidget(self.table)
        split.addWidget(self.plot)
        split.setSizes([260, 420])
        self.body.addWidget(split, 1)

    def _browse(self) -> None:
        d = QFileDialog.getExistingDirectory(self, "Folder of drive logs")
        if d:
            self.folder.setText(d)

    def _run(self, fetch: bool) -> None:
        folder = self.folder.text().strip().strip('"')
        if fetch:
            folder = str(output_dir("data", "microsim_acc"))
            self.folder.setText(folder)
        elif not folder or not Path(folder).is_dir():
            QMessageBox.warning(self, "Real cars", "Choose a folder of drive logs, or "
                                "download the microSIM-ACC runs.")
            return
        for b in (self.run_btn, self.fetch_btn):
            b.setEnabled(False)
        self.bar.setRange(0, 0)
        self.bar.show()
        self.status.setText("downloading…" if fetch else "reading logs…")
        self.out = output_dir("field_study", Path(folder).name)
        run_job(run_study, folder, fetch, max(1, min(8, (os.cpu_count() or 2) - 1)),
                self.out, self.progress.tick.emit, on_done=self.show_result,
                on_error=self._failed)

    def _tick(self, done: int, total: int) -> None:
        self.bar.setRange(0, total)
        self.bar.setValue(done)
        self.status.setText(f"{done} / {total} logs")

    def _failed(self, msg: str) -> None:
        for b in (self.run_btn, self.fetch_btn):
            b.setEnabled(True)
        self.bar.hide()
        self.status.setText("")
        QMessageBox.critical(self, "Field study failed", msg)

    def show_result(self, out) -> None:
        from cacc.fieldstudy import headline

        rows, agg, self.out = out
        for b in (self.run_btn, self.fetch_btn):
            b.setEnabled(True)
        self.open_btn.setEnabled(True)
        self.bar.hide()
        self.status.setText(headline(agg))
        self.table.setRowCount(len(agg))
        for r, a in enumerate(agg):
            for c, (key, _) in enumerate(COLS):
                v = a[key]
                it = QTableWidgetItem(f"{v:.0%}" if key == "amplified_share" else str(v))
                it.setTextAlignment(Qt.AlignCenter)
                self.table.setItem(r, c, it)
        (ax,) = self.plot.clear(rows=1)
        amp = sorted(r["decel_amplification"] for r in rows)
        ax.hist(amp, bins=30, color=BLUE)
        ax.axvline(1.0, color=RED, lw=1.5)
        ax.text(1.01, ax.get_ylim()[1] * 0.9, "braked harder than the car ahead →",
                color=RED, fontsize=8)
        ax.set_xlabel("follower's peak braking / lead car's peak braking", fontsize=8)
        ax.set_ylabel("runs", fontsize=8)
        ax.set_title(headline(agg), fontsize=9, color=GREEN if agg[-1]["amplified_share"]
                     < 0.5 else RED)
        self.plot.draw()
