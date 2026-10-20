"""Stress test page: a controller vs a maneuver x condition grid (EV platoon).

Pick the controller, tick maneuvers and conditions, run. The grid fills in
with a verdict per cell and the value of the criterion that came closest to
failing; click a cell to replay its worst seed in the telemetry panel. "Set
as baseline" keeps the current grid, and the next run marks every cell that
got better (▲) or worse (▼) against it, so a gain change or a different
controller reads as a diff.
"""

from __future__ import annotations

import math
import os
from pathlib import Path

from PySide6.QtCore import QObject, Qt, Signal
from PySide6.QtGui import QColor, QFont
from PySide6.QtWidgets import (QAbstractItemView, QComboBox, QHBoxLayout, QHeaderView,
                               QLineEdit, QListWidget, QListWidgetItem, QMessageBox,
                               QPlainTextEdit, QProgressBar, QPushButton, QSpinBox,
                               QSplitter, QTableWidget, QTableWidgetItem, QVBoxLayout,
                               QWidget)

from cacc.gui.common import (Page, VerdictBadge, card, label, open_path, output_dir,
                             primary_button, run_job)
from cacc.gui.stress_telemetry import Telemetry

CONTROLLERS = (("cthp", "CTHP CACC — Ma 2025 gains"),
               ("cacc", "CACC — radar + V2V (Ploeg 2014)"),
               ("acc", "ACC — radar only"),
               ("plugin", "My controller (plugin file.py:Class)"),
               ("lab", "Controller Lab design"))
#: cell background by verdict and margin
PASS_BG, TIGHT_BG, FAIL_BG, ERR_BG = "#dff2df", "#fbf1d0", "#f6d5d5", "#eeeeee"
TIGHT = 0.15  # a pass with less than 15 % margin is drawn amber


def run_stress(controller, options, maneuvers, conditions, seeds, criteria,
               jobs, out, progress, controller_block=None):
    """Background job: build the campaign plan, run it, write the reports."""
    from cacc import campaign as cp
    from cacc.reporting import write_all

    plan = cp.build_plan(controller, maneuvers, conditions, seeds=seeds,
                         criteria=criteria, options=options,
                         controller_block=controller_block)
    report = cp.run_campaign(plan, jobs=jobs, progress=progress)
    write_all(report, out)
    return report, out


def replay_cell(case: dict, controller: str, seed: int, options: dict):
    from cacc.campaign import replay

    return replay(case, controller, seed, options)


def _cell_text(cell: dict) -> str:
    w = cell["worst"]
    if w is None:
        return "error"
    val = ("never" if isinstance(w, float) and math.isinf(w) else
           f"{w:.3g}" if isinstance(w, float) else str(w))
    mark = {"PASS": "✓", "FAIL": "✕"}.get(cell["verdict"], "!")
    return f"{mark} {cell['metric']}\n{val}"


class _Progress(QObject):
    tick = Signal(int, int)


class StressPage(Page):
    def __init__(self):
        super().__init__(
            "Stress test",
            "Throw every maneuver at your controller under every condition: radio "
            "latency, packet loss, spoofed messages, wet and snowy roads, a derated "
            "battery, slow brakes. Each cell is one case over several seeds; the grid "
            "shows where the car stays safe and where it breaks. Click a cell to "
            "replay its worst run.")
        from cacc import campaign as cp

        self.cp = cp
        self.report = self.baseline = self.out = None
        self.controller, self.options = "cthp", {}
        self.lab_design = None  # set by the Controller Lab ("Stress test this")
        self.progress = _Progress()
        self.progress.tick.connect(self._tick)

        top, tl = card()
        row = QHBoxLayout()
        self.ctrl = QComboBox()
        for key, text in CONTROLLERS:
            self.ctrl.addItem(text, key)
        self.plugin = QLineEdit()
        self.plugin.setPlaceholderText(r"C:\path\my_controller.py:MyController")
        self.plugin.setEnabled(False)
        self.ctrl.currentIndexChanged.connect(
            lambda: self.plugin.setEnabled(self.ctrl.currentData() == "plugin"))
        self.preset = QComboBox()
        self.preset.addItems(("Quick (21 cells)", "Full (90 cells)"))
        self.preset.currentIndexChanged.connect(self._apply_preset)
        self.seeds = QSpinBox()
        self.seeds.setRange(1, 20)
        self.seeds.setValue(2)
        self.jobs = QSpinBox()
        self.jobs.setRange(1, max(1, os.cpu_count() or 1))
        self.jobs.setValue(max(1, min(8, (os.cpu_count() or 2) - 1)))
        for text, w in (("Controller", self.ctrl), (None, self.plugin),
                        ("Preset", self.preset), ("Seeds", self.seeds),
                        ("Workers", self.jobs)):
            if text:
                row.addWidget(label(text, wrap=False))
            row.addWidget(w, 2 if w is self.plugin else 0)
        tl.addLayout(row)
        actions = QHBoxLayout()
        self.run_btn = primary_button("Run stress campaign")
        self.run_btn.clicked.connect(self._run)
        self.bar = QProgressBar()
        self.bar.setTextVisible(True)
        self.bar.setFormat("%v / %m runs")
        self.bar.hide()
        self.status = label("", "subtitle")
        self.badge = VerdictBadge()
        self.base_btn = QPushButton("Set as baseline")
        self.base_btn.setToolTip("Compare the next run against this grid")
        self.base_btn.clicked.connect(self._set_baseline)
        self.folder_btn = QPushButton("Open reports")
        self.folder_btn.clicked.connect(lambda: open_path(self.out))
        for b in (self.base_btn, self.folder_btn):
            b.setEnabled(False)
        actions.addWidget(self.run_btn)
        actions.addWidget(self.bar, 1)
        actions.addWidget(self.status, 1)
        actions.addWidget(self.badge)
        actions.addWidget(self.base_btn)
        actions.addWidget(self.folder_btn)
        tl.addLayout(actions)
        self.body.addWidget(top)

        split = QSplitter(Qt.Horizontal)
        left = QWidget()
        ll = QVBoxLayout(left)
        ll.setContentsMargins(0, 0, 0, 0)
        ll.addWidget(label("Maneuvers (lead car)", "h2"))
        self.man_list = self._checklist(
            (k, v[0]) for k, v in cp.MANEUVERS.items())
        ll.addWidget(self.man_list, 2)
        ll.addWidget(label("Conditions", "h2"))
        self.cond_list = self._checklist(
            (k, f"[{v[1]}] {v[0]}") for k, v in cp.CONDITIONS.items())
        ll.addWidget(self.cond_list, 3)
        ll.addWidget(label("Pass criteria (one per line)", "h2"))
        self.criteria = QPlainTextEdit("\n".join(cp.DEFAULT_CRITERIA))
        self.criteria.setFont(QFont("Consolas", 9))
        self.criteria.setMaximumHeight(90)
        self.criteria.setToolTip("metric op value, e.g. min_ttc >= 2.0 — the "
                                 "metric list is on the Evaluate page")
        ll.addWidget(self.criteria)
        left.setMinimumWidth(260)
        split.addWidget(left)

        right = QSplitter(Qt.Vertical)
        self.grid = QTableWidget(0, 0)
        self.grid.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.grid.setSelectionMode(QAbstractItemView.SingleSelection)
        self.grid.setWordWrap(True)
        self.grid.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.grid.verticalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.grid.cellClicked.connect(self._open_cell)
        self.telemetry = Telemetry()
        right.addWidget(self.grid)
        right.addWidget(self.telemetry)
        right.setSizes([330, 520])
        split.addWidget(right)
        split.setSizes([280, 1000])
        self.body.addWidget(split, 1)
        self._apply_preset()

    # ------------------------------------------------------------ inputs
    @staticmethod
    def _checklist(items) -> QListWidget:
        w = QListWidget()
        for key, text in items:
            it = QListWidgetItem(text)
            it.setData(Qt.UserRole, key)
            it.setFlags(it.flags() | Qt.ItemIsUserCheckable)
            it.setCheckState(Qt.Unchecked)
            w.addItem(it)
        return w

    @staticmethod
    def _checked(w: QListWidget) -> list[str]:
        return [w.item(i).data(Qt.UserRole) for i in range(w.count())
                if w.item(i).checkState() == Qt.Checked]

    def _apply_preset(self) -> None:
        man, cond = self.cp.PRESETS["full" if self.preset.currentIndex() else "quick"]
        for w, keys in ((self.man_list, man), (self.cond_list, cond)):
            for i in range(w.count()):
                it = w.item(i)
                it.setCheckState(Qt.Checked if it.data(Qt.UserRole) in keys
                                 else Qt.Unchecked)

    def use_design(self, design) -> None:
        """Select a Controller Lab design as the controller under test."""
        self.lab_design = design
        i = self.ctrl.findData("lab")
        self.ctrl.setItemText(i, f"Controller Lab design: {design.name}")
        self.ctrl.setCurrentIndex(i)

    def _controller(self) -> str | None:
        key = self.ctrl.currentData()
        if key == "lab":
            if self.lab_design is None:
                QMessageBox.warning(self, "Stress test", "Build a design on the "
                                    "Controller Lab page and press \"Stress test "
                                    "this\" first.")
                return None
            from cacc.tfctrl import REF

            return REF
        if key != "plugin":
            return key
        ref = self.plugin.text().strip().strip('"')
        if ":" not in ref.replace(":\\", ""):
            QMessageBox.warning(self, "Stress test", "Give the plugin as "
                                "path\\to\\file.py:ClassName.")
            return None
        return ref

    # ------------------------------------------------------------ run
    def _run(self) -> None:
        ctrl = self._controller()
        man, cond = self._checked(self.man_list), self._checked(self.cond_list)
        crit = [ln.strip() for ln in self.criteria.toPlainText().splitlines()
                if ln.strip()]
        if ctrl is None:
            return
        if not man or not cond or not crit:
            QMessageBox.warning(self, "Stress test", "Tick at least one maneuver "
                                "and one condition, and give at least one criterion.")
            return
        lab = self.ctrl.currentData() == "lab"
        self.controller = ctrl
        self.options = self.lab_design.options if lab else {}
        block = self.lab_design.controller_block if lab else None
        self.run_btn.setEnabled(False)
        self.badge.set(None)
        self.status.setText("")
        self.bar.setRange(0, len(man) * len(cond) * self.seeds.value())
        self.bar.setValue(0)
        self.bar.show()
        self.out = output_dir("stress", self.lab_design.name if lab
                              else Path(ctrl.split(":")[-1]).stem or ctrl)
        run_job(run_stress, ctrl, self.options, man, cond, self.seeds.value(), crit,
                self.jobs.value(), self.out, self.progress.tick.emit, block,
                on_done=self._done, on_error=self._failed)

    def _tick(self, done: int, total: int) -> None:
        self.bar.setRange(0, total)
        self.bar.setValue(done)

    def _failed(self, msg: str) -> None:
        self.run_btn.setEnabled(True)
        self.bar.hide()
        QMessageBox.critical(self, "Stress campaign failed to run", msg)

    def _done(self, out) -> None:
        self.report, self.out = out
        rep = self.report
        self.run_btn.setEnabled(True)
        self.bar.hide()
        for b in (self.base_btn, self.folder_btn):
            b.setEnabled(True)
        c = rep["counts"]
        self.badge.set(rep["verdict"], f"{c['PASS']} pass · {c['FAIL']} fail"
                       + (f" · {c['ERROR']} error" if c["ERROR"] else ""))
        self.status.setText(f"{len(rep['cases'])} cells · {len(rep['seeds'])} "
                            f"seed(s) · {rep['elapsed_s']:.1f} s")
        self._fill_grid()

    def _set_baseline(self) -> None:
        self.baseline = self.report
        self.status.setText(self.status.text() + " · baseline set")
        self._fill_grid()

    # ------------------------------------------------------------ grid
    def _baseline_cell(self, name: str) -> dict | None:
        if not self.baseline or self.baseline is self.report:
            return None
        for row in self.baseline["campaign"]["grid"]:
            for cell in row:
                if cell and cell["name"] == name:
                    return cell
        return None

    def _fill_grid(self) -> None:
        camp = self.report["campaign"]
        self.grid.clear()
        self.grid.setRowCount(len(camp["maneuvers"]))
        self.grid.setColumnCount(len(camp["conditions"]))
        self.grid.setHorizontalHeaderLabels([c["label"] for c in camp["conditions"]])
        self.grid.setVerticalHeaderLabels([m["label"] for m in camp["maneuvers"]])
        for r, row in enumerate(camp["grid"]):
            for col, cell in enumerate(row):
                if cell is None:
                    continue
                text = _cell_text(cell)
                base = self._baseline_cell(cell["name"])
                if base is not None:
                    d = cell["margin"] - base["margin"]
                    if base["verdict"] != cell["verdict"]:
                        text += "  ▲" if cell["verdict"] == "PASS" else "  ▼"
                    elif math.isfinite(d) and abs(d) > 0.02:
                        text += "  ▲" if d > 0 else "  ▼"
                item = QTableWidgetItem(text)
                item.setTextAlignment(Qt.AlignCenter)
                bg = (ERR_BG if cell["verdict"] == "ERROR" else
                      FAIL_BG if cell["verdict"] == "FAIL" else
                      TIGHT_BG if cell["margin"] < TIGHT else PASS_BG)
                item.setBackground(QColor(bg))
                item.setToolTip(f"{cell['name']}\nbinding: {cell['binding']}\n"
                                f"worst value {cell['worst']} (seed "
                                f"{cell['worst_seed']})\nmargin "
                                f"{cell['margin']:+.0%}"
                                if math.isfinite(cell["margin"]) else cell["name"])
                self.grid.setItem(r, col, item)

    def _open_cell(self, r: int, col: int) -> None:
        if self.report is None:
            return
        cell = self.report["campaign"]["grid"][r][col]
        if cell is None:
            return
        case = next(c for c in self.report["cases"] if c["name"] == cell["name"])
        seed = cell["worst_seed"] or self.report["seeds"][0]
        title = f"{cell['name']}  ·  seed {seed}  ·  {cell['verdict']}"
        self.telemetry.heading.setText(f"Replaying {title}…")
        run_job(replay_cell, case, self.controller, seed, self.options,
                on_done=lambda res: self.telemetry.show_run(res, title),
                on_error=lambda m: QMessageBox.critical(self, "Replay failed", m))
