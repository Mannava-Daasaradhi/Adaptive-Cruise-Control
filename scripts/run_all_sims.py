#!/usr/bin/env python
"""Unified simulation runner -- sets up and runs every simulation in the repo.

Four backends live in this project and each used to be driven by hand:
the Python scientific core (``src/cacc`` + ``scripts/*_study.py``), the ROS 2
distributed stack in WSL (``ros2_ws``), the MATLAB/Simulink reproduction
(``matlab/run_all.m``) and the visual layer (video, filmstrip, dashboard,
deck).  This is the one entry point that checks the environment, runs them all
in dependency order, and prints a PASS/FAIL table.

    python scripts/run_all_sims.py --list        # the stage inventory
    python scripts/run_all_sims.py --doctor      # environment only
    python scripts/run_all_sims.py --quick       # reduced sizes
    python scripts/run_all_sims.py --group python matlab
    python scripts/run_all_sims.py --only base_paper certify
    python scripts/run_all_sims.py --live        # add the interactive 3-D views
    python scripts/run_all_sims.py               # everything headless

Each study keeps writing where it always did (``results/<study>/<stamp>/``);
this runner adds ``results/run_all/<stamp>/`` holding per-stage logs,
``summary.json`` and ``report.md``.
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import platform
import shutil
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable

ROOT = Path(__file__).resolve().parents[1]
WSL_DISTRO = os.environ.get("CACC_WSL_DISTRO", "Ubuntu")
GROUPS = ("python", "ros2", "matlab", "visual")


class Skip(Exception):
    """Raised by a stage builder when its prerequisite is absent."""


# --------------------------------------------------------------------------
# environment discovery
# --------------------------------------------------------------------------

def _imports(py: str, mod: str) -> bool:
    try:
        return subprocess.run([py, "-c", "import " + mod],
                              capture_output=True, timeout=120).returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


def _python_candidates() -> list[str]:
    cands = [sys.executable]
    home = Path.home()
    for base in (home / "miniconda3", home / "anaconda3", home / "mambaforge"):
        cands += sorted(str(p) for p in base.glob("envs/*/python.exe"))
        cands += sorted(str(p) for p in base.glob("envs/*/bin/python"))
        for p in (base / "python.exe", base / "bin" / "python"):
            if p.exists():
                cands.append(str(p))
    for name in ("python", "python3"):
        w = shutil.which(name)
        if w:
            cands.append(w)
    seen: set[str] = set()
    out: list[str] = []
    for c in cands:
        if c and c not in seen and Path(c).exists():
            seen.add(c)
            out.append(c)
    return out


def find_python(mod: str) -> str | None:
    """First interpreter on the machine that can ``import mod``."""
    for c in _python_candidates():
        if _imports(c, mod):
            return c
    return None


def find_matlab() -> str | None:
    w = shutil.which("matlab")
    if w:
        return w
    hits = sorted(glob.glob("C:/Program Files/MATLAB/R*/bin/matlab.exe"))
    return hits[-1] if hits else None


def wsl_ok() -> tuple[bool, str]:
    if not shutil.which("wsl.exe"):
        return False, "wsl.exe not on PATH"
    try:
        p = subprocess.run(["wsl.exe", "-d", WSL_DISTRO, "--", "bash", "-lc",
                            "ls /opt/ros"], capture_output=True, text=True,
                           timeout=180, errors="replace")
    except (OSError, subprocess.SubprocessError) as exc:
        return False, "wsl probe failed: " + str(exc)
    distros = (p.stdout or "").strip()
    if p.returncode != 0 or not distros:
        return False, "no ROS 2 under /opt/ros in " + WSL_DISTRO
    return True, WSL_DISTRO + ": ROS 2 " + distros.split()[0]


def to_wsl(p: Path) -> str:
    s = str(p).replace("\\", "/")
    if len(s) > 2 and s[1] == ":":
        return "/mnt/" + s[0].lower() + s[2:]
    return s


def wsl_argv(name: str, body: str) -> list[str]:
    """LF driver file in a space-free temp dir -- see docs/runbooks/RB-03."""
    d = Path(tempfile.gettempdir()) / "cacc_drv"
    d.mkdir(parents=True, exist_ok=True)
    f = d / (name + ".sh")
    header = "#!/usr/bin/env bash\nset -eo pipefail\n"      # never -u (ROS setup)
    header += 'REPO="' + to_wsl(ROOT) + '"\nexport REPO\n'
    f.write_bytes((header + body).replace("\r\n", "\n").encode())
    return ["wsl.exe", "-d", WSL_DISTRO, "--", "bash", to_wsl(f)]


def sh_run(script: str, args: str = "") -> str:
    """Driver body that CR-strips a repo shell script and runs it."""
    tag = Path(script).stem
    tmp = "/tmp/cacc_" + tag + ".sh"
    return ('tr -d "\\r" < "$REPO/scripts/' + script + '" > ' + tmp + "\n"
            + "bash " + tmp + " " + args + "\n")


# --------------------------------------------------------------------------
# context
# --------------------------------------------------------------------------

class Ctx:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.quick = args.quick
        self.py = args.python or find_python("cacc")
        self.ppt_py = find_python("pptx")
        self.matlab = find_matlab()
        self.wsl, self.wsl_note = wsl_ok()
        self.ffmpeg = shutil.which("ffmpeg")
        self.stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        self.outdir = ROOT / "results" / "run_all" / self.stamp
        self.logdir = self.outdir / "logs"

    def need_py(self) -> str:
        if not self.py:
            raise Skip("no interpreter with the 'cacc' package -- see RB-01 "
                       "(conda env cacc + pip install -e .)")
        return self.py

    def q(self, *flags: str) -> list[str]:
        return list(flags) if self.quick else []

    def newest(self, pattern: str, what: str) -> Path:
        # by mtime, NOT by name: ROS 2 CSVs are stamped by WSL (Etc/UTC)
        # while everything else stamps Windows local time -- see RB-03.
        hits = sorted(ROOT.glob(pattern), key=lambda p: p.stat().st_mtime)
        if not hits:
            raise Skip("no " + what + " yet (pattern " + pattern + ")")
        return hits[-1]

    def newest_stamp(self, study: str) -> str:
        d = ROOT / "results" / study
        stamps = sorted(p.name for p in d.iterdir() if p.is_dir()) if d.is_dir() else []
        return "results/" + study + "/" + stamps[-1] if stamps else ""


def art(study: str) -> Callable[[Ctx], str]:
    return lambda c: c.newest_stamp(study)


# --------------------------------------------------------------------------
# stages
# --------------------------------------------------------------------------

@dataclass
class Stage:
    name: str
    group: str
    desc: str
    argv: Callable[[Ctx], list[str]]
    kind: str = "python"                    # python | wsl | matlab
    cwd: Path = ROOT
    timeout: int = 2400
    fail_on: tuple[str, ...] = ()           # substrings meaning failure at rc 0
    want: tuple[str, ...] = ()              # substrings that MUST appear
    live: bool = False                      # interactive; needs --live
    artifact: Callable[[Ctx], str] = lambda c: ""


def _video(c: Ctx) -> list[str]:
    src = c.newest("results/ros2/*_cthp_n*.csv", "ROS 2 recorder CSV")
    return [c.need_py(), "scripts/render_platoon_video.py", str(src),
            "--speedup", "10"] + c.q("--frames", "6")


def _ros2_plot(c: Ctx) -> list[str]:
    src = c.newest("results/ros2/*_cthp_n*.csv", "ROS 2 recorder CSV")
    return [c.need_py(), "scripts/plot_ros2_run.py", str(src)]


def _matlab(c: Ctx) -> list[str]:
    if not c.matlab:
        raise Skip("MATLAB not found under C:/Program Files/MATLAB/R*/bin")
    return [c.matlab, "-sd", str(ROOT / "matlab"), "-batch", "run_all"]


def _ppt(c: Ctx) -> list[str]:
    if not c.ppt_py:
        raise Skip("python-pptx missing on every interpreter (--fix-env)")
    return [c.ppt_py, "scripts/build_ppt.py"]


def _wsl(name: str, script: str, args: str = "") -> Callable[[Ctx], list[str]]:
    def build(c: Ctx) -> list[str]:
        if not c.wsl:
            raise Skip(c.wsl_note)
        return wsl_argv(name, sh_run(script, args))
    return build


def _ros2_run(c: Ctx) -> list[str]:
    if not c.wsl:
        raise Skip(c.wsl_note)
    dur = "45" if c.quick else "120"
    return wsl_argv("run", sh_run("ros2_run_demo.sh",
                                  "6 " + dur + " cthp ma2025_case_a.yaml"))


def _demo_check(c: Ctx) -> list[str]:
    code = ("from pathlib import Path\n"
            "p = Path('demo/cacc_live_sim.html'); n = len(p.read_bytes())\n"
            "assert n > 50000, 'demo too small: %d B' % n\n"
            "print('DEMO_OK %s %.0f KiB' % (p, n / 1024))\n")
    return [c.need_py(), "-c", code]


def stages() -> list[Stage]:
    S = Stage
    P = "python"
    return [
        S("tests", P, "unit + property suite (src/cacc contracts)",
          lambda c: [c.need_py(), "-m", "pytest", "tests", "-q"],
          timeout=3000, fail_on=(" failed", " error"), want=("passed",)),

        S("base_paper", P, "Ma 2025 (T-ITS) reproduction, Figs. 4-15",
          lambda c: [c.need_py(), "scripts/reproduce_base_paper.py"] + c.q("--quick"),
          artifact=art("base_paper")),

        S("sweep_delay", P, "min string-stable headway vs V2V delay + capacity",
          lambda c: [c.need_py(), "scripts/sweep_delay.py"]
          + (["--steps", "8"] if c.quick else []),
          artifact=art("delay_sweep")),

        S("scenario_leader_brake", P, "ACC vs CACC, hard leader brake",
          lambda c: [c.need_py(), "scripts/run_scenario.py",
                     "scenarios/leader_brake.yaml"],
          artifact=art("leader_brake")),

        S("scenario_leader_sine", P, "ACC vs CACC, sinusoidal leader",
          lambda c: [c.need_py(), "scripts/run_scenario.py",
                     "scenarios/leader_sine.yaml"],
          artifact=art("leader_sine")),

        S("scenario_ma2025_sine", P, "the paper's own leader profile",
          lambda c: [c.need_py(), "scripts/run_scenario.py",
                     "scenarios/ma2025_sine.yaml"],
          artifact=art("ma2025_sine")),

        S("qos_adaptive", P, "D-016 QoS-aware CACC (rho estimate -> h adapt)",
          lambda c: [c.need_py(), "scripts/qos_adaptive_study.py"] + c.q("--quick"),
          artifact=art("qos_adaptive")),

        S("predictive_qos", P, "Flagship A (D-021) predictive channel map",
          lambda c: [c.need_py(), "scripts/predictive_qos_study.py"] + c.q("--quick"),
          artifact=art("predictive_qos")),

        S("spoof_defense", P, "Flagship B (D-022) trust gate vs spoofed V2V",
          lambda c: [c.need_py(), "scripts/spoof_defense_study.py"],
          artifact=art("spoof_defense")),

        S("stochastic_cert", P, "Flagship C (D-023) chance-constrained risk cert",
          lambda c: [c.need_py(), "scripts/stochastic_cert_study.py"],
          artifact=art("stochastic_cert")),

        S("certify", P, "D-018 certification harness (4 suites x seeds)",
          lambda c: [c.need_py(), "scripts/certify.py"]
          + (["--quick", "--seeds", "2"] if c.quick else []),
          # measured 44 min for 5 seeds x 4 suites (deep-zone-gains
          # dominates). 2 h is headroom, not a target.
          timeout=7200, fail_on=("certification FAILED",),
          want=("certification PASSED",), artifact=art("certification")),

        S("ros2_build", "ros2", "rsync repo -> ~/cacc_ws, colcon build on ext4",
          _wsl("build", "ros2_sync_build.sh"), kind="wsl",
          timeout=2400, want=("BUILD_OK",)),

        S("ros2_run", "ros2", "headless 12-process distributed platoon demo",
          _ros2_run, kind="wsl", timeout=1200, want=("RUN_OK",)),

        S("ros2_plot", "ros2", "cross-backend check of the recorder CSV",
          _ros2_plot, artifact=lambda c: "results/ros2/"),

        S("matlab_crosscheck", "matlab", "Simulink <-> Python agreement (ideal channel)",
          lambda c: [c.need_py(), "matlab/tests/crosscheck_python.py"],
          timeout=2400),

        S("matlab_run_all", "matlab", "D-024 Simulink reproduction, Figs. 4-15",
          _matlab, kind="matlab", cwd=ROOT / "matlab", timeout=3600,
          artifact=lambda c: "matlab/results/"),

        S("video", "visual", "MP4 highway animation from the ROS 2 run",
          _video, timeout=2400, artifact=lambda c: "results/videos/"),

        S("filmstrip", "visual", "4-frame filmstrip asset for the deck",
          lambda c: [c.need_py(), "scripts/make_sim_assets.py"],
          timeout=2400, artifact=lambda c: "results/frames/"),

        S("dashboard", "visual", "single-page HTML results dashboard",
          lambda c: [c.need_py(), "scripts/build_dashboard.py"],
          timeout=2400, artifact=art("dashboard")),

        S("browser_demo", "visual", "self-contained interactive browser sim",
          _demo_check, timeout=180, want=("DEMO_OK",),
          artifact=lambda c: "demo/cacc_live_sim.html"),

        S("ppt", "visual", "rebuild the 20-slide presentation deck",
          _ppt, timeout=900, artifact=lambda c: "report/"),

        S("rviz_view", "ros2", "live rviz2 platoon view (interactive)",
          _wsl("view", "ros2_view_demo.sh", "6 cthp ma2025_case_a.yaml"),
          kind="wsl", timeout=900, live=True),

        S("gazebo_view", "visual", "live Gazebo 3-D platoon view (interactive)",
          _wsl("gz", "ros2_gazebo_demo.sh", "6 cthp ma2025_case_a.yaml rviz"),
          kind="wsl", timeout=900, live=True),
    ]


# --------------------------------------------------------------------------
# execution
# --------------------------------------------------------------------------

@dataclass
class Result:
    stage: Stage
    status: str
    seconds: float = 0.0
    detail: str = ""
    log: str = ""


def child_env() -> dict[str, str]:
    env = dict(os.environ)
    env.setdefault("PYTHONIOENCODING", "utf-8")
    env.setdefault("MPLBACKEND", "Agg")          # headless figure rendering
    env["PYTHONUNBUFFERED"] = "1"
    return env


def tail_reason(text: str, n: int = 3) -> str:
    lines = [ln.strip() for ln in text.splitlines() if ln.strip()]
    return " | ".join(lines[-n:])[:240] if lines else "(no output)"


def execute(st: Stage, ctx: Ctx) -> Result:
    try:
        argv = st.argv(ctx)
    except Skip as exc:
        return Result(st, "SKIP", 0.0, str(exc))

    ctx.logdir.mkdir(parents=True, exist_ok=True)
    log = ctx.logdir / (st.name + ".log")
    t0 = time.time()
    print("  -> " + st.name + " ...", flush=True)
    try:
        p = subprocess.run(argv, cwd=str(st.cwd), env=child_env(),
                           capture_output=True, text=True, errors="replace",
                           timeout=st.timeout)
        out = (p.stdout or "") + (p.stderr or "")
        rc = p.returncode
    except subprocess.TimeoutExpired as exc:
        raw = exc.stdout or b""
        out = raw.decode("utf-8", "replace") if isinstance(raw, bytes) else str(raw)
        log.write_text(out + "\n\nTIMEOUT after %ds\n" % st.timeout, encoding="utf-8")
        return Result(st, "FAIL", time.time() - t0,
                      "timeout after %ds" % st.timeout, str(log.relative_to(ROOT)))
    except OSError as exc:
        return Result(st, "FAIL", time.time() - t0, "launch failed: " + str(exc))

    log.write_text("$ " + " ".join(argv) + "\n\n" + out, encoding="utf-8")
    rel = str(log.relative_to(ROOT))
    low = out.lower()

    if rc != 0:
        return Result(st, "FAIL", time.time() - t0,
                      "exit %d: %s" % (rc, tail_reason(out)), rel)
    for bad in st.fail_on:
        if bad.lower() in low:
            return Result(st, "FAIL", time.time() - t0,
                          "failure marker '" + bad.strip() + "' in output", rel)
    for good in st.want:
        if good.lower() not in low:
            return Result(st, "FAIL", time.time() - t0,
                          "missing expected marker '" + good + "'", rel)

    detail = st.artifact(ctx) or tail_reason(out, 1)
    return Result(st, "PASS", time.time() - t0, detail, rel)


# --------------------------------------------------------------------------
# reporting
# --------------------------------------------------------------------------

def doctor(ctx: Ctx) -> list[tuple[str, bool, str]]:
    rows = [
        ("host", True, platform.system() + " " + platform.release()),
        ("python (cacc)", bool(ctx.py), ctx.py or "NOT FOUND -- see RB-01"),
        ("python (pptx)", bool(ctx.ppt_py), ctx.ppt_py or "missing (--fix-env)"),
        ("WSL + ROS 2", ctx.wsl, ctx.wsl_note),
        ("MATLAB", bool(ctx.matlab), ctx.matlab or "not installed"),
        ("ffmpeg", bool(ctx.ffmpeg),
         ctx.ffmpeg or "missing -> video degrades to GIF"),
    ]
    if ctx.py:
        for mod in ("numpy", "scipy", "matplotlib", "yaml", "PIL", "pytest"):
            rows.append(("  " + mod, _imports(ctx.py, mod), ""))
    return rows


def fix_env(ctx: Ctx) -> None:
    if not ctx.py:
        print("cannot fix: no interpreter carries the cacc package")
        return
    for mod, pkg in (("pptx", "python-pptx"),):
        if not _imports(ctx.py, mod):
            print("installing " + pkg + " into " + ctx.py + " ...")
            subprocess.run([ctx.py, "-m", "pip", "install", "-q", pkg], check=False)
    if not _imports(ctx.py, "cacc"):
        subprocess.run([ctx.py, "-m", "pip", "install", "-q", "-e", str(ROOT)],
                       check=False)
    ctx.ppt_py = find_python("pptx")


def table(results: list[Result]) -> str:
    w = max([len(r.stage.name) for r in results] + [5])
    head = "stage".ljust(w) + "  group    status   time     detail"
    lines = [head, "-" * (len(head) + 40)]
    for r in results:
        lines.append("%s  %-7s  %-6s %7.1fs  %s"
                     % (r.stage.name.ljust(w), r.stage.group, r.status,
                        r.seconds, r.detail[:96]))
    return "\n".join(lines)


def write_report(ctx: Ctx, results: list[Result]) -> Path:
    ctx.outdir.mkdir(parents=True, exist_ok=True)
    payload = {
        "stamp": ctx.stamp,
        "quick": ctx.quick,
        "python": ctx.py,
        "matlab": ctx.matlab,
        "wsl": ctx.wsl_note,
        "stages": [{"name": r.stage.name, "group": r.stage.group,
                    "desc": r.stage.desc, "status": r.status,
                    "seconds": round(r.seconds, 2), "detail": r.detail,
                    "log": r.log} for r in results],
    }
    (ctx.outdir / "summary.json").write_text(json.dumps(payload, indent=2),
                                             encoding="utf-8")
    md = ["# Simulation run " + ctx.stamp, "",
          "mode: " + ("quick" if ctx.quick else "full")
          + "  |  python: `" + str(ctx.py) + "`", "",
          "| stage | group | status | time | detail |", "|---|---|---|---:|---|"]
    for r in results:
        md.append("| `%s` | %s | **%s** | %.1fs | %s |"
                  % (r.stage.name, r.stage.group, r.status, r.seconds, r.detail))
    rep = ctx.outdir / "report.md"
    rep.write_text("\n".join(md) + "\n", encoding="utf-8")
    return rep


# --------------------------------------------------------------------------

def main() -> int:
    all_stages = stages()
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--list", action="store_true", help="print the stage inventory")
    ap.add_argument("--doctor", action="store_true", help="environment report only")
    ap.add_argument("--fix-env", action="store_true",
                    help="pip-install whatever the doctor reports missing")
    ap.add_argument("--quick", action="store_true", help="reduced sizes everywhere")
    ap.add_argument("--group", nargs="+", choices=GROUPS, help="restrict to groups")
    ap.add_argument("--only", nargs="+", metavar="STAGE", help="run just these stages")
    ap.add_argument("--skip", nargs="+", metavar="STAGE", default=[])
    ap.add_argument("--live", action="store_true",
                    help="include the interactive rviz2 / Gazebo views")
    ap.add_argument("--stop-on-fail", action="store_true",
                    help="stop at the first failure (default: keep going)")
    ap.add_argument("--dry-run", action="store_true", help="print the commands only")
    ap.add_argument("--python", help="interpreter override for the cacc package")
    args = ap.parse_args()

    known = {s.name for s in all_stages}
    for bad in sorted((set(args.only or []) | set(args.skip)) - known):
        ap.error("unknown stage '" + bad + "'. Known: " + ", ".join(sorted(known)))

    if args.list:
        w = max(len(s.name) for s in all_stages)
        for s in all_stages:
            print(s.name.ljust(w) + "  %-7s %s%s"
                  % (s.group, s.desc, "  [--live]" if s.live else ""))
        return 0

    ctx = Ctx(args)
    if args.fix_env:
        fix_env(ctx)

    print("CACC unified simulation runner  |  " + ctx.stamp + "  |  "
          + ("quick" if ctx.quick else "full") + " mode\n")
    rows = doctor(ctx)
    w = max(len(k) for k, _, _ in rows)
    for k, ok, note in rows:
        print("  " + k.ljust(w) + "  " + ("ok  " if ok else "MISS") + "  " + note)
    print()
    if args.doctor:
        return 0

    sel = [s for s in all_stages
           if (not args.only or s.name in args.only)
           and (not args.group or s.group in args.group)
           and s.name not in args.skip
           and (args.live or not s.live or bool(args.only and s.name in args.only))]

    if args.dry_run:
        for s in sel:
            try:
                print("  " + s.name + ": " + " ".join(s.argv(ctx)))
            except Skip as exc:
                print("  " + s.name + ": SKIP -- " + str(exc))
        return 0

    # a sweep is long: keep the table readable when piped into tee/CI logs
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except (AttributeError, OSError):
        pass
    print("running %d stage(s)\n" % len(sel))
    results: list[Result] = []
    for s in sel:
        r = execute(s, ctx)
        results.append(r)
        print("     %-4s %6.1fs  %s" % (r.status, r.seconds, r.detail[:110]),
              flush=True)
        if r.status == "FAIL" and args.stop_on_fail:
            break

    print("\n" + table(results))
    rep = write_report(ctx, results)
    n_fail = sum(r.status == "FAIL" for r in results)
    n_skip = sum(r.status == "SKIP" for r in results)
    print("\n%d passed, %d failed, %d skipped   ->  %s"
          % (len(results) - n_fail - n_skip, n_fail, n_skip,
             rep.relative_to(ROOT)))
    return 1 if n_fail else 0


if __name__ == "__main__":
    raise SystemExit(main())
