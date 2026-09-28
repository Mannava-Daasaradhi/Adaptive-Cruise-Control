"""CACC Studio — desktop front end for the cacc engine (PySide6).

    cacc gui            or    cacc-studio

Pages: start / environment check / new project (``cacc init``), drive-log
audit (``cacc audit``), test-plan evaluation (``cacc evaluate``), and a live
platoon simulator with the black-box string-stability sweep (``cacc run`` /
``cacc sweep``). Install with ``pip install -e ".[gui]"``.
"""

from __future__ import annotations

import sys


def main(argv: list[str] | None = None) -> int:
    import multiprocessing

    multiprocessing.freeze_support()  # evaluate -j N inside the packaged .exe
    try:
        from PySide6.QtWidgets import QApplication
    except ImportError:
        print("CACC Studio needs PySide6: pip install -e \".[gui]\"", file=sys.stderr)
        return 2
    import matplotlib

    matplotlib.use("QtAgg")
    from cacc.gui.app import MainWindow
    from cacc.gui.common import APP_NAME, light_theme

    app = QApplication.instance() or QApplication(sys.argv if argv is None else argv)
    app.setApplicationName(APP_NAME)
    light_theme(app)
    win = MainWindow()
    win.show()
    return app.exec()
