"""CACC Studio main window: sidebar navigation over the four pages."""

from __future__ import annotations

from PySide6.QtWidgets import (QHBoxLayout, QListWidget, QListWidgetItem, QMainWindow,
                               QStackedWidget, QVBoxLayout, QWidget)

import cacc
from cacc.gui.common import APP_NAME, STYLESHEET, label, resource
from cacc.gui.page_audit import AuditPage
from cacc.gui.page_evaluate import EvaluatePage
from cacc.gui.page_setup import SetupPage
from cacc.gui.page_simulate import SimulatePage

NAV = (("setup", "  Start here"), ("audit", "  Audit a drive log"),
       ("evaluate", "  Evaluate a controller"), ("simulate", "  Live platoon"))


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(f"{APP_NAME} {cacc.__version__}")
        self.resize(1280, 820)
        self.setStyleSheet(STYLESHEET)
        self.pages = {"setup": SetupPage(), "audit": AuditPage(),
                      "evaluate": EvaluatePage(), "simulate": SimulatePage()}

        side = QWidget()
        side.setFixedWidth(230)
        side.setStyleSheet("background:#1f2328;")
        sl = QVBoxLayout(side)
        sl.setContentsMargins(0, 0, 0, 0)
        sl.setSpacing(0)
        sl.addWidget(label(APP_NAME, "brand", wrap=False))
        self.nav = QListWidget()
        self.nav.setObjectName("nav")
        for key, text in NAV:
            item = QListWidgetItem(text)
            item.setData(256, key)
            self.nav.addItem(item)
        sl.addWidget(self.nav, 1)
        foot = label(f"engine v{cacc.__version__}\nstring stability · V2X robustness",
                     wrap=True)
        foot.setStyleSheet("color:#8a8880; padding:14px; font-size:8pt;")
        sl.addWidget(foot)

        self.stack = QStackedWidget()
        for key, _ in NAV:
            self.stack.addWidget(self.pages[key])
        self.nav.currentRowChanged.connect(self.stack.setCurrentIndex)

        root = QWidget()
        lay = QHBoxLayout(root)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        lay.addWidget(side)
        lay.addWidget(self.stack, 1)
        self.setCentralWidget(root)

        setup: SetupPage = self.pages["setup"]
        setup.navigate.connect(self.show_page)
        setup.open_plan.connect(self._open_plan)
        setup.sample_audit.connect(self._sample_audit)
        self.nav.setCurrentRow(0)

    def show_page(self, key: str) -> None:
        self.nav.setCurrentRow([k for k, _ in NAV].index(key))

    def _open_plan(self, path: str) -> None:
        self.pages["evaluate"].open_plan(path)
        self.show_page("evaluate")

    def _sample_audit(self) -> None:
        p = resource("examples", "data", "synthetic_openacc_platoon.csv")
        if p is not None:
            self.pages["audit"].log.set_path(p)
        self.show_page("audit")
