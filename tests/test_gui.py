"""CACC Studio (desktop GUI) smoke tests — headless; skipped without PySide6."""

import os
from pathlib import Path

import pytest

pytest.importorskip("PySide6")
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import matplotlib  # noqa: E402

matplotlib.use("QtAgg")

from PySide6.QtCore import QEventLoop, QTimer  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
LOG = ROOT / "examples" / "data" / "synthetic_openacc_platoon.csv"


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


@pytest.fixture
def window(app):
    from cacc.gui.app import MainWindow

    w = MainWindow()
    yield w
    w.close()


def test_window_builds_and_navigates(window):
    assert window.stack.count() == 4
    for key in ("audit", "evaluate", "simulate", "setup"):
        window.show_page(key)
        assert window.stack.currentWidget() is window.pages[key]


def test_environment_report_all_green(app):
    from cacc.gui.page_setup import environment_report

    rows = environment_report()
    assert rows and all(ok for ok, _ in rows), rows


def test_resources_found_in_repo():
    from cacc.gui.common import resource

    assert resource("examples", "data", "synthetic_openacc_platoon.csv") == LOG


def test_background_job_reports_on_gui_thread(app):
    from cacc.gui.common import run_job

    got, loop = {}, QEventLoop()
    run_job(lambda a, b: a + b, 2, 3, on_done=lambda v: (got.update(v=v), loop.quit()))
    run_job(lambda: 1 / 0, on_error=lambda m: (got.update(err=m), loop.quit()))
    QTimer.singleShot(10_000, loop.quit)
    while len(got) < 2:
        loop.exec()
    assert got["v"] == 5 and "ZeroDivisionError" in got["err"]


def test_live_sim_acc_amplifies_cacc_attenuates(window):
    from cacc.gui.page_simulate import LEADERS, build_config, simulate

    brake = next(iter(LEADERS.values()))
    _, acc = simulate(build_config("acc", 5, 20.0, 0.7, 0.1, 0.0), "acc", brake)
    _, cacc_ = simulate(build_config("cacc", 5, 20.0, 0.7, 0.1, 0.0), "cacc", brake)
    assert acc["l2_amplification_max"] > 1.0 > cacc_["l2_amplification_max"]
    page = window.pages["simulate"]
    page._sim_done(simulate(build_config("cacc", 5, 20.0, 0.7, 0.1, 0.0), "cacc", brake))
    assert page.road.res is not None and page.scrub.maximum() > 0
    assert page.metrics.item(0, 1).text()


def test_evaluate_inline_plan(window, tmp_path):
    from cacc.gui.page_evaluate import run_plan

    text = f"""
name: gui-smoke
base: {(ROOT / 'scenarios' / 'leader_brake.yaml').as_posix()}
under_test: {{controller: cacc}}
seeds: 1
criteria: ["min_gap >= 2.0"]
"""
    report, paths = run_plan(text, ROOT, None, 1, tmp_path / "out")
    assert report["verdict"] == "PASS" and paths["markdown"].is_file()
    page = window.pages["evaluate"]
    page._done((report, paths))
    assert page.table.rowCount() == 1 and page.table.item(0, 1).text() == "PASS"


def test_audit_sample_log(window, tmp_path):
    from cacc.gui.page_audit import run_audit

    res, paths = run_audit(str(LOG), 30.0, 0.5, 0.1, tmp_path / "audit")
    assert paths["report"].is_file()
    verdicts = {v.twin.follower: v.twin.verdict for v in res.vehicles}
    assert list(verdicts.values()).count("string-unstable") == 2
    page = window.pages["audit"]
    page._done((res, paths))
    assert page.table.rowCount() == 3
    assert "unstable" in page.table.item(0, 1).text().lower()


def test_scaffold_from_setup_page(window, tmp_path):
    page = window.pages["setup"]
    page.folder.set_path(tmp_path / "proj")
    page._scaffold()
    assert (tmp_path / "proj" / "plans" / "release_gate.yaml").is_file()
    assert page.run_gate.isEnabled()
