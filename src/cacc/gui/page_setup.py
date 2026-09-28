"""Start page: environment check, new controller project, quick starts."""

from __future__ import annotations

import importlib
import platform
from pathlib import Path

from PySide6.QtCore import Signal
from PySide6.QtWidgets import QGridLayout, QHBoxLayout, QLineEdit, QMessageBox, QPushButton

import cacc
from cacc.gui.common import (FilePicker, Page, card, label, open_path, output_dir,
                             primary_button, resource)

CHECKS = (("numpy", "numerics"), ("scipy", "signal processing"),
          ("matplotlib", "charts"), ("yaml", "test plans"), ("PySide6", "this window"))


def environment_report() -> list[tuple[bool, str]]:
    """(ok, text) rows describing what this installation can do."""
    rows = [(True, f"cacc engine {cacc.__version__} · Python {platform.python_version()} "
                   f"· {platform.system()} {platform.release()}")]
    for mod, role in CHECKS:
        try:
            m = importlib.import_module(mod)
            rows.append((True, f"{mod} {getattr(m, '__version__', '')} — {role}"))
        except ImportError as exc:
            rows.append((False, f"{mod} missing — {role} ({exc})"))
    samples = resource("examples", "data", "synthetic_openacc_platoon.csv")
    rows.append((samples is not None, "sample drive log and test plans found" if samples
                 else "sample data not bundled (you can still use your own files)"))
    out = output_dir("any", "x").parent.parent  # Documents/CACC Studio
    try:
        out.mkdir(parents=True, exist_ok=True)
        rows.append((True, f"results are saved under {out}"))
    except OSError as exc:
        rows.append((False, f"cannot write results folder {out}: {exc}"))
    return rows


class SetupPage(Page):
    navigate = Signal(str)  # page key
    open_plan = Signal(str)  # plan path for the Evaluate page
    sample_audit = Signal()

    def __init__(self):
        super().__init__(
            "Welcome to CACC Studio",
            "Find out whether an adaptive-cruise controller amplifies traffic waves, "
            "from a real drive log or from the controller itself, and what V2V "
            "communication would change. Everything runs on this computer; no data "
            "leaves it.")

        grid = QGridLayout()
        grid.setSpacing(12)
        starts = (("Audit a drive log", "Upload 30–120 min of car-following data and get "
                   "a per-car verdict, a digital twin and a customer-ready report.",
                   "Try with the sample log", self.sample_audit.emit),
                  ("Evaluate a controller", "Run your controller through a test plan: "
                   "safety, comfort and string stability, with a PASS/FAIL per case.",
                   "Open the evaluator", lambda: self.navigate.emit("evaluate")),
                  ("Live platoon", "Drag sliders for time gap, latency and packet loss, "
                   "then watch the platoon react in real time.",
                   "Start the simulator", lambda: self.navigate.emit("simulate")))
        for col, (title, text, button, fn) in enumerate(starts):
            box, lay = card()
            lay.addWidget(label(title, "h2"))
            lay.addWidget(label(text, "subtitle"), 1)
            b = primary_button(button) if col == 0 else QPushButton(button)
            b.clicked.connect(fn)
            lay.addWidget(b)
            grid.addWidget(box, 0, col)
        self.body.addLayout(grid)

        box, lay = card()
        lay.addWidget(label("1 · Check this installation", "h2"))
        self.checks = label("", wrap=True)
        lay.addWidget(self.checks)
        recheck = QPushButton("Run check again")
        recheck.clicked.connect(self.refresh)
        row = QHBoxLayout()
        row.addWidget(recheck)
        row.addStretch(1)
        lay.addLayout(row)
        self.body.addWidget(box)

        box, lay = card()
        lay.addWidget(label("2 · Start a project for your own controller", "h2"))
        lay.addWidget(label(
            "Creates a controller template (Python class), a release-gate test plan and "
            "a GitHub Actions workflow. Replace the template's control law with yours and "
            "run the plan. It's the same as `cacc init` on the command line.", "subtitle"))
        row = QHBoxLayout()
        self.folder = FilePicker("Project folder", folder=True,
                                 placeholder="Empty folder for the new project…")
        self.name = QLineEdit()
        self.name.setPlaceholderText("project name (optional)")
        create = primary_button("Create project")
        create.clicked.connect(self._scaffold)
        row.addWidget(self.folder, 3)
        row.addWidget(self.name, 1)
        row.addWidget(create)
        lay.addLayout(row)
        self.created = label("", "subtitle")
        lay.addWidget(self.created)
        after = QHBoxLayout()
        self.open_ctrl = QPushButton("Edit controller file")
        self.run_gate = QPushButton("Run its release gate")
        self.open_ctrl.clicked.connect(lambda: open_path(self._root / "controllers" /
                                                         "my_controller.py"))
        self.run_gate.clicked.connect(lambda: self.open_plan.emit(
            str(self._root / "plans" / "release_gate.yaml")))
        for b in (self.open_ctrl, self.run_gate):
            b.setEnabled(False)
            after.addWidget(b)
        after.addStretch(1)
        lay.addLayout(after)
        self.body.addWidget(box)
        self.body.addStretch(1)
        self._root = Path()
        self.refresh()

    def refresh(self) -> None:
        self.checks.setText("<br>".join(
            f"<span style='color:{'#067a06' if ok else '#b02e2e'}'>"
            f"{'✓' if ok else '✕'}</span>&nbsp; {text}"
            for ok, text in environment_report()))

    def _scaffold(self) -> None:
        from cacc.scaffold import scaffold

        folder = self.folder.path()
        if not folder:
            QMessageBox.warning(self, "New project", "Choose a folder first.")
            return
        try:
            files = scaffold(folder, self.name.text().strip() or None)
        except FileExistsError as exc:
            if QMessageBox.question(self, "Overwrite?", f"{exc}\n\nOverwrite these files?"
                                    ) != QMessageBox.Yes:
                return
            files = scaffold(folder, self.name.text().strip() or None, force=True)
        except OSError as exc:
            QMessageBox.critical(self, "New project", str(exc))
            return
        self._root = Path(folder)
        self.created.setText("Created: " + ", ".join(
            str(Path(f).relative_to(self._root)) for f in files))
        for b in (self.open_ctrl, self.run_gate):
            b.setEnabled(True)
