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
    assert window.stack.count() == 9
    for key in ("audit", "evaluate", "lab", "stress", "standards", "realcars", "hil",
                "simulate", "setup"):
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


def test_stress_page_grid_baseline_and_telemetry(window, tmp_path):
    from cacc.campaign import replay
    from cacc.gui.page_stress import run_stress

    ticks = []
    report, _ = run_stress("cacc", {}, ["hard-brake"], ["nominal", "spoof"], 1,
                           ["min_gap >= 1.0"], 1, tmp_path / "s",
                           lambda d, n: ticks.append((d, n)))
    assert ticks[-1] == (2, 2)
    page = window.pages["stress"]
    page._done((report, tmp_path / "s"))
    assert (page.grid.rowCount(), page.grid.columnCount()) == (1, 2)
    assert page.grid.item(0, 0).text().startswith("✓")
    assert page.grid.item(0, 1).text().startswith("✕")
    page._set_baseline()
    page._done((report, tmp_path / "s"))  # same grid again: no ▲/▼ marks
    assert "▲" not in page.grid.item(0, 1).text()
    cell = report["campaign"]["grid"][0][1]
    res = replay(report["cases"][1], "cacc", cell["worst_seed"])
    page.telemetry.show_run(res, "spoof")
    page.telemetry._pin()
    page.telemetry.show_run(res, "spoof again")
    assert page.telemetry.tiles["min_gap"].text().startswith("-")


def test_controller_lab_live_analysis_inside_view_and_handoff(window):
    from cacc.gui.page_lab import simulate

    lab = window.pages["lab"]
    lab.edits["e"].setText("0.2*s")  # improper: flagged, cannot be sent
    lab.refresh()
    assert lab.design is None and not lab.stress_btn.isEnabled()
    assert "improper" in lab.state["e"].toolTip()
    lab.edits["e"].setText("0.2/(h*s + 1)")
    lab.refresh()
    assert lab.report.string_stable and lab.report.internally_stable
    assert "peak |Γ|" in lab.physics.toPlainText()
    lab.show_inside(simulate(lab.design, "hard-brake", 0.2, 0.0, 0.05, 2))
    assert lab.inside_status.text().startswith("min gap")
    lab._send()
    stress = window.pages["stress"]
    assert window.stack.currentWidget() is stress
    assert stress.ctrl.currentData() == "lab" and stress._controller() == "cacc.tfctrl:TFController"
    assert stress.lab_design.options["blocks"]["e"] == "0.2/(h*s + 1)"


def test_stress_insight_tabs_3d_why_and_breaking_point(window):
    import numpy as np

    from cacc import campaign as cp
    from cacc.explain import explain

    page = window.pages["stress"]
    assert [page.tabs.tabText(i) for i in range(page.tabs.count())] == [
        "Traces", "3-D replay", "Why it failed", "Breaking point", "Energy && comfort",
        "Versions"]
    report = cp.run_campaign(cp.build_plan("cacc", ["hard-brake"], ["cut-in"], seeds=1))
    page._done((report, None))
    cell = report["campaign"]["grid"][0][0]
    res = cp.replay(report["cases"][0], "cacc", cell["worst_seed"])
    page._replayed(res, "cut-in", dict(cell, verdict="PASS"), 1)  # no explain job
    v = page.view3d
    assert v.view.crash and v.jump.isEnabled() and v.slider.maximum() == len(res.t) - 1
    for mode in ("Drone", "Chase", "Side"):
        v.cam.setCurrentText(mode)
        v.slider.setValue(1400)
        img = v.view.grab().toImage()
        assert img.width() > 0 and img.pixelColor(10, 10).isValid()
    v.view.set_frame(10**6)
    assert v.view.k == len(res.t) - 1
    page.why.show(explain(report, cell["name"], res=res, verify=False))
    assert "hit the car in front" in page.why.text.toPlainText()
    rows = [{"label": "V2V latency", "status": "holds", "fraction": 1.0,
             "value": 1.0, "unit": "s", "text": "holds"},
            {"label": "Road grip μ", "status": "breaks", "fraction": 0.4,
             "value": 0.6, "unit": "", "text": "breaks at 0.6"}]
    page.breaking.show_profile(sorted(rows, key=lambda r: r["fraction"]))
    assert page.breaking.status.text().startswith("Weakest: Road grip")
    page.breaking.show_map({"x": [0, 1], "y": [0, 1], "x_label": "x", "y_label": "y",
                            "margin": np.array([[0.5, -0.2], [0.1, -1.0]]),
                            "verdict": np.array([["PASS", "FAIL"], ["PASS", "FAIL"]]),
                            "pass_share": 0.5})
    assert "50%" in page.breaking.status.text()
    ctx = page._under_test()
    assert ctx[0] == page._controller() and ctx[3]


def test_lab_auto_tune_loads_the_result(window):
    from cacc import tfctrl as tc
    from cacc.autotune import TuneResult

    lab = window.pages["lab"]
    d = tc.Design.preset("lag-ff")
    tuned = tc.Design(dict(d.blocks, e="0.3"), "a", 0.8, 2.5, "lag-ff-tuned")
    from PySide6.QtWidgets import QMessageBox

    orig = QMessageBox.information
    QMessageBox.information = staticmethod(lambda *a, **k: None)
    try:
        lab._tuned(TuneResult(tuned, d, True, feasible=3))
    finally:
        QMessageBox.information = orig
    assert lab.edits["e"].text() == "0.3" and lab.h.value() == 0.8
    assert lab.tune_btn.isEnabled()


def test_aeb_energy_versions_and_standards(window, tmp_path):
    from cacc import campaign as cp
    from cacc import standards as st

    page = window.pages["stress"]
    page.versions.store_fn = lambda: tmp_path / "versions"
    page.preset.setCurrentIndex(2)  # crash-db
    assert "nhtsa-lvs" in page._checked(page.man_list)
    page.aeb.setChecked(True)
    report = cp.run_campaign(cp.build_plan("cacc", ["hard-brake"], ["cut-in"], seeds=1,
                                           aeb=page.aeb.isChecked()))
    page._done((report, None))
    assert "AEB on" in page.status.text() and page.grid.item(0, 0).text().startswith("✓")
    cell = report["campaign"]["grid"][0][0]
    res = cp.replay(report["cases"][0], "cacc", cell["worst_seed"])
    page._replayed(res, "cut-in + AEB", cell, 1)
    assert "Wh" in page.energy.summary.text()
    page.view3d.slider.setValue(1100)
    assert page.view3d.view.grab().toImage().width() > 0
    for name in ("v1", "v2"):
        page.versions.name.setText(name)
        page.versions.save()
    assert page.versions.table.columnCount() == 2
    assert "v1 → v2" in page.versions.diff.toPlainText()
    std = window.pages["standards"]
    rows = [dict(r, status=s_, evidence="x", value=1.0)
            for r, s_ in zip((dict(zip(("id", "standard", "requirement"), q[:3]),
                                   rule="min_gap > 0") for q in st.REQUIREMENTS[:3]),
                             ("PASS", "FAIL", "REVIEW"))]
    std.show_result({"rows": rows})
    assert std.table.rowCount() == 3 and std.table.item(1, 0).text() == "✕ FAIL"


def test_real_cars_and_hil_pages(window, tmp_path):
    from cacc.gui.page_hil import run_hil
    from cacc.gui.page_realcars import run_study
    from test_fieldstudy import microsim_csv

    d = tmp_path / "runs"
    d.mkdir()
    microsim_csv(d / "55_25_S_1.csv", 1.3)
    microsim_csv(d / "55_25_L_1.csv", 0.8)
    rc = window.pages["realcars"]
    rc.show_result(run_study(d, False, 1, tmp_path / "out", lambda *a: None))
    assert rc.table.rowCount() == 3 and "2 real runs" in rc.status.text()
    hp = window.pages["hil"]
    assert not hp.address.isEnabled()  # demo device: no address needed
    hp.show_result(run_hil("spawn", "", [2], "hard-brake", "nominal", 0.02, False, "cacc"))
    assert hp.status.text().startswith("every reply on time")
    assert "round trip" in hp.text.toPlainText()
