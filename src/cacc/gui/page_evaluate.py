"""Evaluate page: a controller + a test plan -> PASS/FAIL matrix + evidence."""

from __future__ import annotations

import os
from pathlib import Path

import yaml
from PySide6.QtCore import Qt
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (QAbstractItemView, QHBoxLayout, QHeaderView, QMessageBox,
                               QPlainTextEdit, QPushButton, QSpinBox, QSplitter,
                               QTableWidget, QTableWidgetItem, QTabWidget)

from cacc.gui.common import (FilePicker, Page, VerdictBadge, card, label, open_path,
                             output_dir, primary_button, resource, run_job)


def run_plan(text: str, root: Path, source: Path | None, jobs: int, out: Path):
    """Background job: parse the (possibly edited) plan, run it, write reports."""
    from cacc.evaluate import evaluate, plan_from_dict
    from cacc.reporting import write_all

    plan = plan_from_dict(yaml.safe_load(text) or {}, root=root, source=source)
    report = evaluate(plan, jobs=jobs)
    return report, write_all(report, out)


def _fmt_value(v) -> str:
    return "—" if v is None else (f"{v:.4g}" if isinstance(v, float) else str(v))


class EvaluatePage(Page):
    def __init__(self):
        super().__init__(
            "Evaluate a controller",
            "A test plan names the controller under test (built-in acc / cacc / cthp, or "
            "your own Python class as a plugin), the scenarios and matrix to run, the "
            "seeds, and the pass criteria. You get a PASS/FAIL verdict per case with the "
            "worst seed, plus report.json, junit.xml and summary.md for CI.")
        self.report, self.paths = None, {}

        box, lay = card()
        row = QHBoxLayout()
        self.plan = FilePicker("Test plan", "Test plans (*.yaml *.yml);;All files (*)")
        self.plan.changed.connect(self._load_text)
        row.addWidget(label("Test plan", wrap=False))
        row.addWidget(self.plan, 1)
        for text, name in (("Sample: passing CACC", "smoke.yaml"),
                           ("Sample: commercial-style ACC", "idm_acc.yaml")):
            b = QPushButton(text)
            p = resource("examples", "plans", name)
            b.setEnabled(p is not None)
            b.clicked.connect(lambda _=False, p=p: self.plan.set_path(p))
            row.addWidget(b)
        lay.addLayout(row)
        actions = QHBoxLayout()
        self.jobs = QSpinBox()
        self.jobs.setRange(1, max(1, os.cpu_count() or 1))
        self.jobs.setValue(max(1, min(4, (os.cpu_count() or 2) - 1)))
        self.run_btn = primary_button("Run test plan")
        self.run_btn.clicked.connect(self._run)
        self.status = label("", "subtitle")
        self.badge = VerdictBadge()
        self.summary_btn = QPushButton("Open summary")
        self.folder_btn = QPushButton("Open folder")
        self.summary_btn.clicked.connect(lambda: open_path(self.paths["markdown"]))
        self.folder_btn.clicked.connect(lambda: open_path(self.paths["markdown"].parent))
        for b in (self.summary_btn, self.folder_btn):
            b.setEnabled(False)
        actions.addWidget(self.run_btn)
        actions.addWidget(label("Parallel workers", wrap=False))
        actions.addWidget(self.jobs)
        actions.addWidget(self.status, 1)
        actions.addWidget(self.badge)
        actions.addWidget(self.summary_btn)
        actions.addWidget(self.folder_btn)
        lay.addLayout(actions)
        self.body.addWidget(box)

        split = QSplitter(Qt.Horizontal)
        self.editor = QPlainTextEdit()
        self.editor.setFont(QFont("Consolas", 9))
        self.editor.setLineWrapMode(QPlainTextEdit.NoWrap)
        self.editor.setPlaceholderText("The plan's YAML appears here. Edit thresholds or "
                                       "the matrix, then run; the file is not changed.")
        tabs = QTabWidget()
        tabs.addTab(self.editor, "Plan (editable)")
        tabs.addTab(self._metrics_help(), "Criteria you can use")
        split.addWidget(tabs)
        right = QSplitter(Qt.Vertical)
        self.table = QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(("Case", "Verdict", "Failed checks",
                                              "Peak |Γ| (sweep)"))
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.verticalHeader().hide()
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.itemSelectionChanged.connect(self._show_case)
        self.detail = QPlainTextEdit()
        self.detail.setReadOnly(True)
        self.detail.setFont(QFont("Consolas", 9))
        right.addWidget(self.table)
        right.addWidget(self.detail)
        split.addWidget(right)
        split.setSizes([420, 620])
        self.body.addWidget(split, 1)

    @staticmethod
    def _metrics_help() -> QPlainTextEdit:
        from cacc.criteria import METRICS

        w = QPlainTextEdit()
        w.setReadOnly(True)
        w.setFont(QFont("Consolas", 9))
        lines = ['Write criteria as "metric op value", e.g. "min_ttc >= 2.0".', ""]
        for name, m in METRICS.items():
            better = ">=" if m.worse == "low" else "<="
            lines.append(f"{name:22s} [{m.unit}]  use {better}\n    {m.doc}")
        w.setPlainText("\n".join(lines))
        return w

    def open_plan(self, path: str | Path) -> None:
        self.plan.set_path(path)

    def _load_text(self, path: str) -> None:
        p = Path(path.strip().strip('"'))
        if p.is_file():
            try:
                self.editor.setPlainText(p.read_text(encoding="utf-8"))
            except OSError as exc:
                self.status.setText(f"Cannot read plan: {exc}")

    # ------------------------------------------------------------ run
    def _run(self) -> None:
        text = self.editor.toPlainText()
        if not text.strip():
            QMessageBox.warning(self, "Evaluate", "Choose a test plan first.")
            return
        src = Path(self.plan.path()) if self.plan.path() else None
        root = src.parent if src else Path.cwd()
        try:
            name = (yaml.safe_load(text) or {}).get("name", "plan")
        except yaml.YAMLError as exc:
            QMessageBox.critical(self, "Invalid YAML", str(exc))
            return
        self.run_btn.setEnabled(False)
        self.badge.set(None)
        self.status.setText("Running simulations… (a few seconds to a few minutes)")
        run_job(run_plan, text, root, src, self.jobs.value(),
                output_dir("evaluations", str(name)),
                on_done=self._done, on_error=self._failed)

    def _failed(self, msg: str) -> None:
        self.run_btn.setEnabled(True)
        self.status.setText("Run failed.")
        QMessageBox.critical(self, "Test plan failed to run", msg)

    def _done(self, out) -> None:
        self.report, self.paths = out
        rep = self.report
        self.run_btn.setEnabled(True)
        c = rep["counts"]
        self.badge.set(rep["verdict"], f"{c['PASS']} pass · {c['FAIL']} fail"
                       + (f" · {c['ERROR']} error" if c["ERROR"] else ""))
        ctrl = rep["under_test"]["controller"]
        file, _, cls = ctrl.rpartition(":")  # 'C:\...\x.py:Class' -> 'x.py:Class'
        short = f"{Path(file).name}:{cls}" if file.endswith(".py") else ctrl
        self.status.setText(f"{rep['plan']} · {short} · {rep['elapsed_s']:.1f} s")
        self.status.setToolTip(f"{ctrl}\nreports: {self.paths['markdown'].parent}")
        for b in (self.summary_btn, self.folder_btn):
            b.setEnabled(True)
        self.table.setRowCount(len(rep["cases"]))
        for i, case in enumerate(rep["cases"]):
            failed = [ch["label"] for ch in case["checks"] if not ch["passed"]]
            ss = case["string_stability"]
            if ss is not None and not ss["passed"]:
                failed.append("string stability")
            cells = (case["name"], case["verdict"], ", ".join(failed) or "—",
                     "—" if ss is None else f"{ss['peak_gain']:.4f}"
                     + (" (marginal)" if ss.get("marginal") else ""))
            for j, text in enumerate(cells):
                item = QTableWidgetItem(text)
                if j == 1:
                    item.setForeground(Qt.GlobalColor.darkGreen if text == "PASS"
                                       else Qt.GlobalColor.darkRed)
                self.table.setItem(i, j, item)
        self.table.selectRow(0)

    def _show_case(self) -> None:
        rows = self.table.selectionModel().selectedRows()
        if not rows or self.report is None:
            return
        case = self.report["cases"][rows[0].row()]
        lines = [f"{case['name']}  →  {case['verdict']}"]
        if case["params"]:
            lines.append("varied: " + ", ".join(f"{k} = {v}" for k, v in
                                                case["params"].items()))
        lines.append("")
        for ch in case["checks"]:
            mark = "✓" if ch["passed"] else "✕"
            lines.append(f"{mark} {ch['label']:28s} worst {_fmt_value(ch['worst'])} "
                         f"{ch['unit']} (seed {ch['worst_seed']})")
        ss = case["string_stability"]
        if ss is not None:
            mark = "✓" if ss["passed"] else "✕"
            se = ss.get("peak_gain_se")
            lines.append(f"{mark} string stability: peak |Γ| {ss['peak_gain']:.4f}"
                         + (f" ± {se:.4f}" if se else "")
                         + f" at {ss['peak_omega_rad_s']:.3f} rad/s "
                         f"(limit {ss['max_gain']:g})")
        for err in case["errors"]:
            lines.append(f"! error (seed {err['seed']}): {err['error']}")
        self.detail.setPlainText("\n".join(lines))
