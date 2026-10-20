"""Energy & comfort score, version history and hardware in the loop."""

import numpy as np
import pytest

from cacc import campaign as cp
from cacc import energy
from cacc import history as hi
from cacc.adapters import hil
from cacc.adapters import hil_device as dev


# ----------------------------------------------------------------- energy
@pytest.fixture(scope="module")
def wave():
    import yaml

    base = yaml.safe_load(cp.BASE.read_text(encoding="utf-8"))
    tree = cp.compose(base, "stop-and-go", "nominal", "cacc")
    return cp.replay({"scenario": tree, "name": "x"}, "cacc", 1)


def test_energy_score_shapes_and_physics(wave):
    s = energy.score(wave)
    n = wave.vel.shape[1]
    assert all(np.shape(v) == (n,) for v in s.values())
    assert np.all(s["kwh_per_100km"] > 5) and np.all(s["range_km"] > 100)
    assert s["range_lost_pct"][0] == pytest.approx(0.0)
    assert np.all(s["regen_kwh"] >= 0) and np.all(s["friction_kwh"] >= -1e-12)
    from cacc.criteria import METRICS

    assert METRICS["range_loss_pct"].fn(wave) == pytest.approx(
        float(np.nanmax(s["range_lost_pct"][1:])))
    assert METRICS["rms_jerk"].fn(wave) > 0
    assert 0.0 <= METRICS["harsh_share"].fn(wave) <= 1.0


def test_energy_compare_table():
    rows = energy.compare(["cacc", "acc"], "hard-brake", "nominal")
    assert [r["kwh_per_100km"] for r in rows] == sorted(r["kwh_per_100km"] for r in rows)
    assert "kWh/100km" in energy.format_table(rows)


# ---------------------------------------------------------------- history
def _report(verdicts, margins):
    cells = [{"name": n, "verdict": v, "binding": "min_gap >= 1", "worst": 1.0,
              "worst_seed": 1, "margin": m, "metric": "min_gap"}
             for n, v, m in zip(("a | x", "b | x", "c | x"), verdicts, margins)]
    return {"campaign": {"grid": [cells], "aeb": False},
            "counts": {"PASS": verdicts.count("PASS"), "FAIL": verdicts.count("FAIL")},
            "under_test": {"controller": "cacc"}, "seeds": [1]}


def test_history_diff_flags_regressions(tmp_path):
    hi.save(_report(["PASS", "FAIL", "PASS"], [0.5, -0.2, 0.30]), "v1", tmp_path)
    hi.save(_report(["FAIL", "PASS", "PASS"], [-0.1, 0.1, 0.10]), "v2", tmp_path)
    old, new = hi.find(tmp_path, None, -2), hi.find(tmp_path, None, -1)
    rows = {r["cell"]: r["change"] for r in hi.diff(old, new)}
    assert rows == {"a | x": "regression", "b | x": "fixed", "c | x": "worse"}
    assert "1 regression" in hi.format_diff(old, new, hi.diff(old, new))
    assert "v2" in hi.format_table(tmp_path)
    with pytest.raises(ValueError, match="no version"):
        hi.find(tmp_path, "v9", -1)
    with pytest.raises(ValueError, match="not a stress"):
        hi.snapshot({"cases": []}, "x")


def test_history_cli(tmp_path, capsys):
    import json

    from cacc.cli import main

    for i, verdicts in enumerate((["PASS"] * 3, ["PASS", "FAIL", "PASS"])):
        p = tmp_path / f"r{i}.json"
        p.write_text(json.dumps(_report(verdicts, [0.5, 0.5 if i == 0 else -0.1, 0.5])),
                     encoding="utf-8")
        assert main(["history", "save", str(p), "--label", f"v{i}",
                     "--store", str(tmp_path / "h")]) == 0
    assert main(["history", "diff", "--store", str(tmp_path / "h")]) == 1
    assert "regression" in capsys.readouterr().out
    assert main(["history", "show", "--store", str(tmp_path / "h")]) == 0


# -------------------------------------------------------------------- HIL
def test_wire_formats_round_trip():
    m = {"t": 1.5, "e": -0.25, "e_dot": 0.1, "dv": -1.2, "u_ff": -2.0,
         "gap": 19.7, "v": 24.3, "a": -1.1}
    car, seq, back = dev.unpack_request(dev.pack_request(3, 77, m))
    assert (car, seq) == (3, 77) and all(back[k] == pytest.approx(m[k], abs=1e-5) for k in m)
    (ida, a), (idb, b) = dev.can_request(3, 77, m)
    seq, cm = dev.can_decode_request(a, b)
    assert (ida, idb, seq) == (0x203, 0x283, 77)
    assert cm["gap"] == pytest.approx(19.7) and cm["u_ff"] == pytest.approx(-2.0)
    assert dev.handle(dev.DeviceLaw(), b"junk") is None


def test_hil_udp_matches_software():
    r = hil.run("hard-brake", "nominal", spawn_device=True, cars=[2])
    assert r["timing"][0]["car"] == 2 and r["timing"][0]["misses"] == 0
    assert r["max_du"] < 1e-4 and r["min_gap_hw"] == pytest.approx(r["min_gap_sw"], abs=1e-3)
    assert "round trip" in hil.format_text(r)


def test_hil_timeout_holds_or_brakes():
    import socket

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))  # a device that never answers
    port = s.getsockname()[1]
    try:
        from cacc.controllers import ControllerParams

        ctrl = hil.HILController(ControllerParams(), address=f"127.0.0.1:{port}",
                                 timeout=0.01, on_timeout="brake")
        ctrl.bind_car(1)
        u = ctrl.output(None, 1.0, 0.0, 0.0, 0.0, gap=20.0, v=25.0, a=0.0, t=0.0)
        assert u == -3.0 and ctrl.misses == 1
    finally:
        s.close()
        hil.reset()


def test_device_cli_parses(monkeypatch):
    from cacc.cli import main

    seen = {}
    monkeypatch.setattr(dev, "serve_udp", lambda addr, law, **k: seen.update(a=addr, k=law.kind))
    assert main(["hil-device", "--udp", "0.0.0.0:6000", "--law", "acc"]) == 0
    assert seen == {"a": "0.0.0.0:6000", "k": "acc"}
