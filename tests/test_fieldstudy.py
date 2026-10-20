"""Field study of real ACC cars: microSIM-ACC reader, model-free numbers,
aggregation and the dataset download (with a fake server)."""

import io
import zipfile
from datetime import datetime, timedelta, timezone

import numpy as np
import pytest

from cacc import fieldstudy as fs
from cacc.fielddata import read_log


def microsim_csv(path, k_follow=1.2, n=200):
    """A braking run in the microSIM-ACC layout: the follower's braking is
    the leader's scaled by ``k_follow`` (and delayed)."""
    t = np.arange(n) * 0.2
    a_lead = np.where((t > 5) & (t < 10), -2.0, 0.0)
    v_lead = 24.0 + np.cumsum(a_lead) * 0.2
    a_f = np.roll(a_lead, 5) * k_follow
    v_f = 24.0 + np.cumsum(a_f) * 0.2
    gap = 30.0 + np.cumsum(v_lead - v_f) * 0.2
    t0 = datetime(2021, 3, 25, 8, 28, 5, 213399, tzinfo=timezone(timedelta(hours=-7)))
    rows = ["timestamps,speed,rpm,maf,timestamps.1,speed.1,rpm.1,maf.1,"
            "Inital Headway,spacing,Difference"]
    for k in range(n):
        ts = (t0 + timedelta(seconds=float(t[k]))).isoformat()
        ts = ts[:-6] + "887" + ts[-6:]  # nanosecond stamps like the real files
        rows.append(f"{ts},{v_lead[k] * 3.6:.0f},1,1,{ts},{v_f[k] * 3.6:.0f},1,1,"
                    f"1.2,{gap[k]:.3f},0")
    rows.append(",,,,,,,,,,")  # the real files end with blank rows
    path.write_text("\n".join(rows) + "\n", encoding="utf-8")
    return path


def test_reader_and_model_free_numbers(tmp_path):
    p = microsim_csv(tmp_path / "55_25_S_3.csv", k_follow=1.25)
    log = read_log(p)
    assert log.names == ["leader", "ACC gap setting S"]
    assert log.dt == pytest.approx(0.2, rel=1e-3) and log.meta["from_mph"] == "55"
    mf = fs.model_free(log, 1)
    assert mf["decel_amplification"] == pytest.approx(1.25, abs=0.12)
    assert mf["speed_undershoot"] > 0.5 and mf["min_gap"] < 30.0


def test_study_aggregates_by_setting(tmp_path):
    d = tmp_path / "runs"
    d.mkdir()
    microsim_csv(d / "55_25_L_1.csv", 0.8)
    microsim_csv(d / "55_25_S_1.csv", 1.3)
    microsim_csv(d / "55_25_S_2.csv", 1.2)
    rows, problems = fs.study(d, twin=False)
    agg = {a["group"]: a for a in fs.aggregate(rows)}
    assert agg["ACC gap setting S"]["amplified_share"] == 1.0
    assert agg["ACC gap setting L"]["amplified_share"] == 0.0
    assert agg["all runs"]["runs"] == 3 and not problems
    md = fs.write(rows, fs.aggregate(rows), problems, tmp_path / "out", "test")
    assert "braked harder than the car ahead" in md.read_text(encoding="utf-8")
    assert "3 real runs" in fs.headline(fs.aggregate(rows))


def test_fetch_keeps_runs_with_a_gap_column(tmp_path):
    good = microsim_csv(tmp_path / "x.csv").read_bytes()
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w") as z:
        z.writestr("set/55_0_M_1.csv", good)
        z.writestr("set/35_0_M_1.csv", b"timestamps,speed1,speed2\n1,2,3\n")
        z.writestr("set/readme.txt", b"hi")

    def opener(url, timeout):
        return io.BytesIO(buf.getvalue())

    out = fs.fetch_microsim(tmp_path / "data", opener=opener)
    assert [p.name for p in out.iterdir()] == ["55_0_M_1.csv"]


def test_cli_field_study(tmp_path, capsys):
    from cacc.cli import main

    d = tmp_path / "runs"
    d.mkdir()
    microsim_csv(d / "55_25_M_1.csv", 1.3)
    assert main(["field-study", str(d), "--no-twin", "-j", "1",
                 "-o", str(tmp_path / "out")]) == 0
    assert "1 real runs" in capsys.readouterr().out
    assert (tmp_path / "out" / "runs.csv").is_file()
    assert main(["field-study"]) == 2
