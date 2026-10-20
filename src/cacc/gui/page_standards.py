"""Standards page: pre-assess a controller against ACC / CACC / AEB rules.

One click runs :func:`cacc.standards.check`: every requirement row (ISO
15622 comfort limits, ISO 20035 string stability and V2V degradation,
UNECE R157 and Euro NCAP AEB scenarios, fault reaction) gets a verdict and
the run that proves it. Ticking "+ AEB" reruns it with the emergency-braking
layer, so the effect of the safety layer reads row by row.
"""

from __future__ import annotations

import os

from PySide6.QtCore import QObject, Qt, Signal
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (QAbstractItemView, QCheckBox, QComboBox, QHBoxLayout,
                               QHeaderView, QLineEdit, QMessageBox, QProgressBar,
                               QPushButton, QSpinBox, QTableWidget, QTableWidgetItem)

from cacc.gui.common import (Page, card, label, open_path, output_dir, primary_button,
                             run_job)

STATUS_BG = {"PASS": "#dff2df", "FAIL": "#f6d5d5", "REVIEW": "#fbf1d0", "N/A": "#eeeeee"}
STATUS_TXT = {"PASS": "✓ PASS", "FAIL": "✕ FAIL", "REVIEW": "? REVIEW", "N/A": "– N/A"}


def run_standards(controller, options, aeb, seeds, jobs, out, progress):
    from cacc import standards as st

    res = st.check(controller, options, aeb=aeb, seeds=seeds, jobs=jobs,
                   progress=progress)
    st.write(res, out)
    return res, out


class _Progress(QObject):
    tick = Signal(int, int)


class StandardsPage(Page):
    def __init__(self):
        super().__init__(
            "Standards",
            "Pre-assess a controller against the rules a carmaker will ask about: "
            "ISO 15622 ACC comfort limits, ISO 20035 CACC string stability and V2V "
            "loss, UNECE R157 and Euro NCAP AEB crash scenarios, and fault "
            "reaction. Requirements are paraphrased and scenario rows are judged on "
            "no contact: a pre-assessment, not a certification.")
        self.out = None
        self.progress = _Progress()
        self.progress.tick.connect(self._tick)
        top, tl = card()
        row = QHBoxLayout()
        self.ctrl = QComboBox()
        for key, text in (("cacc", "CACC (Ploeg 2014)"), ("cthp", "CTHP CACC (Ma 2025)"),
                          ("acc", "ACC (radar only)"), ("plugin", "My controller (plugin)")):
            self.ctrl.addItem(text, key)
        self.plugin = QLineEdit()
        self.plugin.setPlaceholderText(r"C:\path\my_controller.py:MyController")
        self.plugin.setEnabled(False)
        self.ctrl.currentIndexChanged.connect(
            lambda: self.plugin.setEnabled(self.ctrl.currentData() == "plugin"))
        self.aeb = QCheckBox("+ AEB")
        self.seeds = QSpinBox()
        self.seeds.setRange(1, 10)
        self.jobs = QSpinBox()
        self.jobs.setRange(1, max(1, os.cpu_count() or 1))
        self.jobs.setValue(max(1, min(8, (os.cpu_count() or 2) - 1)))
        for text, w in (("Controller", self.ctrl), (None, self.plugin), (None, self.aeb),
                        ("Seeds", self.seeds), ("Workers", self.jobs)):
            if text:
                row.addWidget(label(text, wrap=False))
            row.addWidget(w, 2 if w is self.plugin else 0)
        tl.addLayout(row)
        act = QHBoxLayout()
        self.run_btn = primary_button("Run checklist")
        self.run_btn.clicked.connect(self._run)
        self.bar = QProgressBar()
        self.bar.hide()
        self.status = label("", "subtitle")
        self.open_btn = QPushButton("Open report")
        self.open_btn.setEnabled(False)
        self.open_btn.clicked.connect(lambda: open_path(self.out / "standards.md"))
        act.addWidget(self.run_btn)
        act.addWidget(self.bar, 1)
        act.addWidget(self.status, 1)
        act.addWidget(self.open_btn)
        tl.addLayout(act)
        self.body.addWidget(top)
        self.table = QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(["Status", "Standard", "Requirement",
                                              "Evidence"])
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setWordWrap(True)
        hh = self.table.horizontalHeader()
        for c, mode in enumerate((QHeaderView.ResizeToContents, QHeaderView.ResizeToContents,
                                  QHeaderView.Stretch, QHeaderView.Stretch)):
            hh.setSectionResizeMode(c, mode)
        self.body.addWidget(self.table, 1)

    def _controller(self):
        key = self.ctrl.currentData()
        if key != "plugin":
            return key
        ref = self.plugin.text().strip().strip('"')
        if ":" not in ref.replace(":\\", ""):
            QMessageBox.warning(self, "Standards", "Give the plugin as "
                                "path\\to\\file.py:ClassName.")
            return None
        return ref

    def _run(self) -> None:
        ctrl = self._controller()
        if ctrl is None:
            return
        self.run_btn.setEnabled(False)
        self.bar.setRange(0, 0)
        self.bar.show()
        self.status.setText("running the checklist scenarios…")
        self.out = output_dir("standards", ctrl.split(":")[-1])
        run_job(run_standards, ctrl, {}, self.aeb.isChecked(), self.seeds.value(),
                self.jobs.value(), self.out, self.progress.tick.emit,
                on_done=self._done, on_error=self._failed)

    def _tick(self, done: int, total: int) -> None:
        self.bar.setRange(0, total)
        self.bar.setValue(done)

    def _failed(self, msg: str) -> None:
        self.run_btn.setEnabled(True)
        self.bar.hide()
        self.status.setText("")
        QMessageBox.critical(self, "Checklist failed to run", msg)

    def _done(self, out) -> None:
        res, self.out = out
        self.run_btn.setEnabled(True)
        self.open_btn.setEnabled(True)
        self.bar.hide()
        c = res["counts"]
        self.status.setText(f"{c['PASS']} pass · {c['FAIL']} fail · {c['REVIEW']} "
                            f"review · {c['N/A']} n/a" + (" · AEB on" if res["aeb"] else ""))
        self.show_result(res)

    def show_result(self, res: dict) -> None:
        rows = res["rows"]
        self.table.setRowCount(len(rows))
        for r, row in enumerate(rows):
            vals = (STATUS_TXT[row["status"]], row["standard"],
                    f"{row['requirement']}\n{row['rule']}", row["evidence"])
            for c, v in enumerate(vals):
                it = QTableWidgetItem(v)
                if c == 0:
                    it.setBackground(QColor(STATUS_BG[row["status"]]))
                    it.setTextAlignment(Qt.AlignCenter)
                self.table.setItem(r, c, it)
        self.table.resizeRowsToContents()
