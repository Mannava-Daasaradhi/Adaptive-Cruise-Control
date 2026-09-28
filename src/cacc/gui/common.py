"""Shared pieces of CACC Studio: theme, background jobs, plots, pickers.

Every heavy call (audit, test plan, sweep) runs through :func:`run_job` on
the Qt thread pool so the window never freezes; results come back on the
GUI thread through the job's ``done`` / ``failed`` signals.
"""

from __future__ import annotations

import sys
import traceback
from datetime import datetime
from pathlib import Path

from PySide6.QtCore import QObject, QRunnable, QStandardPaths, Qt, QThreadPool, QUrl, Signal
from PySide6.QtGui import QDesktopServices
from PySide6.QtWidgets import (QFileDialog, QFrame, QHBoxLayout, QLabel, QLineEdit,
                               QPushButton, QSizePolicy, QVBoxLayout, QWidget)

APP_NAME = "CACC Studio"

# house colours (cacc.plotstyle / the audit report)
BLUE, ORANGE, GREEN, RED, AQUA = "#2a78d6", "#e8722e", "#0f8a3c", "#e0403f", "#1baf7a"
INK, MUTED, LINE, PANEL, SURFACE = "#1a1a19", "#52514e", "#e4e3dc", "#f4f3f0", "#fcfcfb"
VERDICT_STYLE = {  # verdict -> (text colour, background, label)
    "PASS": ("#067a06", "#eaf6ea", "✓  PASS"),
    "FAIL": ("#b02e2e", "#fbeaea", "✕  FAIL"),
    "ERROR": ("#8a5d00", "#fdf3dc", "!  ERROR"),
    "string-stable": ("#067a06", "#eaf6ea", "✓  String-stable"),
    "string-unstable": ("#b02e2e", "#fbeaea", "✕  String-unstable"),
    "marginal": ("#8a5d00", "#fdf3dc", "!  Marginal"),
}

STYLESHEET = f"""
QMainWindow, QWidget#page {{ background: {SURFACE}; }}
QWidget {{ color: {INK}; font-size: 10pt; }}
QListWidget#nav {{ background: #1f2328; border: none; padding-top: 8px; }}
QListWidget#nav::item {{ color: #d6d4cc; padding: 12px 18px; border-left: 3px solid transparent; }}
QListWidget#nav::item:selected {{ background: #2c3239; color: white; border-left: 3px solid {BLUE}; }}
QLabel#brand {{ background: #1f2328; color: white; font-size: 14pt; font-weight: 600; padding: 18px; }}
QLabel#title {{ font-size: 17pt; font-weight: 600; }}
QLabel#subtitle {{ color: {MUTED}; }}
QLabel#h2 {{ font-size: 11.5pt; font-weight: 600; margin-top: 6px; }}
QFrame#card {{ background: {PANEL}; border: 1px solid {LINE}; border-radius: 8px; }}
QPushButton {{ background: white; border: 1px solid #c3c2b7; border-radius: 6px; padding: 6px 14px; }}
QPushButton:hover {{ border-color: {BLUE}; }}
QPushButton:disabled {{ color: #a9a79e; }}
QPushButton#primary {{ background: {BLUE}; color: white; border: none; font-weight: 600; padding: 8px 20px; }}
QPushButton#primary:hover {{ background: #1f63b5; }}
QPushButton#primary:disabled {{ background: #9dbde6; }}
QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {{ background: white; border: 1px solid #c3c2b7;
    border-radius: 5px; padding: 4px 6px; }}
QTableWidget {{ background: white; border: 1px solid {LINE}; gridline-color: {LINE}; }}
QHeaderView::section {{ background: {PANEL}; border: none; border-bottom: 1px solid {LINE};
    padding: 6px; font-weight: 600; }}
QPlainTextEdit {{ background: white; border: 1px solid {LINE}; font-family: Consolas, monospace; }}
QTabWidget::pane {{ border: 1px solid {LINE}; background: white; }}
QTabBar::tab {{ padding: 6px 14px; background: {PANEL}; color: {MUTED}; }}
QTabBar::tab:hover {{ color: {INK}; }}
QTabBar::tab:selected {{ background: white; color: {INK}; border-bottom: 2px solid {BLUE};
    font-weight: 600; }}
"""


def light_theme(app) -> None:
    """Fusion + an explicit light palette, so Windows dark mode cannot leave
    black tabs, scrollbars or selections inside the light pages."""
    from PySide6.QtGui import QColor, QPalette

    app.setStyle("Fusion")
    pal = QPalette()
    for role, colour in ((QPalette.Window, SURFACE), (QPalette.WindowText, INK),
                         (QPalette.Base, "#ffffff"), (QPalette.AlternateBase, PANEL),
                         (QPalette.Text, INK), (QPalette.Button, "#ffffff"),
                         (QPalette.ButtonText, INK), (QPalette.ToolTipBase, "#ffffff"),
                         (QPalette.ToolTipText, INK), (QPalette.Highlight, "#d6e6f8"),
                         (QPalette.HighlightedText, INK), (QPalette.PlaceholderText, "#8a8880"),
                         (QPalette.Link, BLUE)):
        pal.setColor(role, QColor(colour))
    pal.setColor(QPalette.Disabled, QPalette.ButtonText, QColor("#a9a79e"))
    pal.setColor(QPalette.Disabled, QPalette.Text, QColor("#a9a79e"))
    app.setPalette(pal)


# ----------------------------------------------------------------- paths
def resource_root() -> Path | None:
    """Folder holding ``scenarios/`` and ``examples/`` (repo or .exe bundle)."""
    if getattr(sys, "frozen", False):
        return Path(getattr(sys, "_MEIPASS", Path(sys.executable).parent))
    root = Path(__file__).resolve().parents[3]
    return root if (root / "scenarios").is_dir() else None


def resource(*parts: str) -> Path | None:
    root = resource_root()
    p = root.joinpath(*parts) if root else None
    return p if p is not None and p.exists() else None


def output_dir(kind: str, name: str) -> Path:
    """Fresh ``Documents/CACC Studio/<kind>/<name>_<stamp>`` folder path."""
    docs = QStandardPaths.writableLocation(QStandardPaths.DocumentsLocation) or str(Path.home())
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    return Path(docs) / APP_NAME / kind / f"{name}_{stamp}"


def open_path(path: str | Path) -> None:
    """Open a file or folder with the system's default application."""
    QDesktopServices.openUrl(QUrl.fromLocalFile(str(Path(path).resolve())))


# ------------------------------------------------------------ background jobs
class _Signals(QObject):
    done = Signal(object)
    failed = Signal(str)


class Job(QRunnable):
    def __init__(self, fn, *args, **kwargs):
        super().__init__()
        self.fn, self.args, self.kwargs = fn, args, kwargs
        self.signals = _Signals()

    def run(self) -> None:
        try:
            out = self.fn(*self.args, **self.kwargs)
        except Exception as exc:  # noqa: BLE001 — shown to the user, never swallowed
            msg = str(exc) if isinstance(exc, (ValueError, OSError)) else (
                f"{type(exc).__name__}: {exc}")
            self.signals.failed.emit(msg + "\n\n" + traceback.format_exc(limit=4))
        else:
            self.signals.done.emit(out)


_live_jobs: set[Job] = set()  # keep Python wrappers alive until they report


def run_job(fn, *args, on_done=None, on_error=None, **kwargs) -> Job:
    job = Job(fn, *args, **kwargs)
    job.setAutoDelete(False)
    _live_jobs.add(job)

    def _finish(handler, value):
        _live_jobs.discard(job)
        if handler:
            handler(value)

    job.signals.done.connect(lambda v: _finish(on_done, v))
    job.signals.failed.connect(lambda m: _finish(on_error, m))
    QThreadPool.globalInstance().start(job)
    return job


# ------------------------------------------------------------------ widgets
def label(text: str, name: str | None = None, wrap: bool = True) -> QLabel:
    lab = QLabel(text)
    if name:
        lab.setObjectName(name)
    lab.setWordWrap(wrap)
    lab.setTextInteractionFlags(Qt.TextSelectableByMouse)
    return lab


def card() -> tuple[QFrame, QVBoxLayout]:
    frame = QFrame()
    frame.setObjectName("card")
    lay = QVBoxLayout(frame)
    lay.setContentsMargins(14, 12, 14, 12)
    return frame, lay


def primary_button(text: str) -> QPushButton:
    b = QPushButton(text)
    b.setObjectName("primary")
    b.setCursor(Qt.PointingHandCursor)
    return b


class Page(QWidget):
    """A scrollable-free page with a title, subtitle and body layout."""

    def __init__(self, title: str, subtitle: str):
        super().__init__()
        self.setObjectName("page")
        self.body = QVBoxLayout(self)
        self.body.setContentsMargins(28, 22, 28, 22)
        self.body.setSpacing(10)
        self.body.addWidget(label(title, "title"))
        self.body.addWidget(label(subtitle, "subtitle"))


class VerdictBadge(QLabel):
    def __init__(self):
        super().__init__("")
        self.setAlignment(Qt.AlignCenter)
        self.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Fixed)
        self.hide()

    def set(self, verdict: str | None, extra: str = "") -> None:
        if verdict is None:
            self.hide()
            return
        fg, bg, text = VERDICT_STYLE.get(verdict, (INK, PANEL, verdict))
        self.setText(text + (f"   {extra}" if extra else ""))
        self.setStyleSheet(f"color:{fg}; background:{bg}; border:1px solid {fg};"
                           "border-radius:6px; padding:6px 14px; font-weight:600;"
                           "font-size:11pt;")
        self.show()


class FilePicker(QWidget):
    """Path field + Browse button; also accepts a file dropped onto it."""

    changed = Signal(str)

    def __init__(self, caption: str, filters: str = "All files (*)",
                 folder: bool = False, placeholder: str = ""):
        super().__init__()
        self.caption, self.filters, self.folder = caption, filters, folder
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        self.edit = QLineEdit()
        self.edit.setPlaceholderText(placeholder or ("Choose a folder…" if folder
                                                     else "Choose or drop a file…"))
        self.edit.textChanged.connect(self.changed)
        browse = QPushButton("Browse…")
        browse.clicked.connect(self._browse)
        lay.addWidget(self.edit, 1)
        lay.addWidget(browse)
        self.setAcceptDrops(True)

    def path(self) -> str:
        return self.edit.text().strip().strip('"')

    def set_path(self, p: str | Path) -> None:
        self.edit.setText(str(p))

    def _browse(self) -> None:
        start = self.path() or str(Path.home())
        if self.folder:
            p = QFileDialog.getExistingDirectory(self, self.caption, start)
        else:
            p, _ = QFileDialog.getOpenFileName(self, self.caption, start, self.filters)
        if p:
            self.set_path(p)

    def dragEnterEvent(self, e):  # noqa: N802 (Qt API)
        if e.mimeData().hasUrls():
            e.acceptProposedAction()

    def dropEvent(self, e):  # noqa: N802
        urls = e.mimeData().urls()
        if urls:
            self.set_path(urls[0].toLocalFile())


class PlotCanvas(QWidget):
    """A matplotlib figure embedded in Qt, styled like the reports."""

    def __init__(self, rows: int = 1, height: float = 3.0, sharex: bool = True):
        super().__init__()
        from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
        from matplotlib.figure import Figure

        self.fig = Figure(figsize=(6, height), facecolor="white", layout="constrained")
        self.canvas = FigureCanvasQTAgg(self.fig)
        self.rows, self.sharex = rows, sharex
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(self.canvas)
        self.axes = []
        self.clear()

    def clear(self, rows: int | None = None) -> list:
        self.fig.clear()
        self.rows = rows or self.rows
        axes = self.fig.subplots(self.rows, 1, sharex=self.sharex, squeeze=False)[:, 0]
        for ax in axes:
            for side in ("top", "right"):
                ax.spines[side].set_visible(False)
            for side in ("left", "bottom"):
                ax.spines[side].set_color("#c3c2b7")
            ax.tick_params(colors=MUTED, labelsize=8)
            ax.grid(True, color=LINE, linewidth=0.6)
            ax.set_axisbelow(True)
        self.axes = list(axes)
        return self.axes

    def draw(self) -> None:
        self.canvas.draw_idle()


def vehicle_colors(n: int) -> list:
    """Leader dark, followers along a blue→orange ramp (readable order)."""
    import matplotlib as mpl

    cmap = mpl.colormaps["plasma"]
    return [INK] + [cmap(0.1 + 0.75 * i / max(1, n - 1)) for i in range(n)]
