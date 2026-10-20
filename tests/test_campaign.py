"""Stress campaigns: cell composition, grid layout, verdicts, CLI."""

import math

import pytest
import yaml

from cacc import campaign as cp


@pytest.fixture(scope="module")
def base():
    return yaml.safe_load(cp.BASE.read_text(encoding="utf-8"))


def test_leader_stop_is_capped_at_road_grip(base):
    tree = cp.compose(base, "emergency-stop", "snow", "cacc")
    lim = 0.3 * 9.81
    assert tree["leader"]["decel"] == pytest.approx(-lim)
    # same speed drop as the dry stop: decel x duration is preserved
    assert tree["leader"]["decel"] * tree["leader"]["duration"] == pytest.approx(-8.0 * 3.1)
    assert tree["sim"]["t_final"] >= 10.0 + tree["leader"]["duration"] + 30.0
    dry = cp.compose(base, "emergency-stop", "nominal", "cacc")
    assert dry["leader"]["decel"] == -8.0


def test_attack_window_follows_the_maneuver_and_family_gains(base):
    tree = cp.compose(base, "hard-brake", "spoof-gated", "cthp")
    assert tree["attack"]["t0"] == 10.0 and tree["attack"]["t1"] == 25.0
    assert tree["trust"]["enabled"] is True
    assert tree["controller"]["kp"] == 0.009  # Ma-2025 gains for cthp
    assert "attack" not in base  # the base mapping is never mutated


def test_unknown_keys_fail_loudly(base):
    with pytest.raises(ValueError, match="maneuver"):
        cp.compose(base, "barrel-roll", "nominal", "cacc")
    with pytest.raises(ValueError, match="condition"):
        cp.compose(base, "cruise", "volcano", "cacc")


def test_acc_skips_radio_and_security_conditions():
    plan = cp.build_plan("acc", ["cruise"], ["nominal", "loss-30", "spoof", "wet"],
                         seeds=1)
    assert plan.campaign["conditions"] == ["nominal", "wet"]


def test_margin_sign_and_infinities():
    assert cp.margin({"worst": 3.0, "threshold": 2.0, "op": ">="}) == pytest.approx(0.5)
    assert cp.margin({"worst": 3.0, "threshold": 2.0, "op": "<="}) == pytest.approx(-0.5)
    assert cp.margin({"worst": math.inf, "threshold": 1.5, "op": ">="}) == math.inf
    assert cp.margin({"worst": None, "threshold": 1.0, "op": ">="}) == -math.inf


def test_campaign_finds_the_spoof_and_keeps_the_nominal_cell():
    plan = cp.build_plan("cacc", ["hard-brake"], ["nominal", "spoof"], seeds=1)
    rep = cp.run_campaign(plan)
    (row,) = rep["campaign"]["grid"]
    nominal, spoof = row
    assert nominal["verdict"] == "PASS" and nominal["margin"] > 0
    assert spoof["verdict"] == "FAIL" and spoof["metric"] == "min_gap"
    assert spoof["worst"] < 0  # the spoofed follower hits the car ahead
    res = cp.replay(rep["cases"][1], "cacc", spoof["worst_seed"])
    gaps = -(res.pos[:, 1:] - res.pos[:, :-1]) - res.config.vehicle.length
    assert gaps.min() == pytest.approx(spoof["worst"])
    assert "✕" in cp.format_grid(rep) and "✓" in cp.format_grid(rep)


def test_cli_stress_exit_code_and_reports(tmp_path, capsys):
    from cacc.cli import main

    code = main(["stress", "-c", "cacc", "-m", "cruise", "-k", "nominal", "wet",
                 "--seeds", "1", "-o", str(tmp_path)])
    assert code == 0
    assert (tmp_path / "report.json").is_file() and (tmp_path / "junit.xml").is_file()
    assert main(["stress", "--list"]) == 0
    assert "emergency-stop" in capsys.readouterr().out
