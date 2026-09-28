"""Build CACC Studio as a Windows app a customer can run without Python.

    pip install -e ".[package]"
    python scripts/build_exe.py            # -> dist/CACC-Studio/CACC-Studio.exe
                                           #    + dist/CACC-Studio-<version>-win64.zip

One-folder build (not --onefile): it starts in a couple of seconds instead
of unpacking ~300 MB of numpy/scipy/Qt on every launch, and antivirus
scanners flag it far less. Ship the zip; the customer unzips it and
double-clicks CACC-Studio.exe. The sample scenarios, test plans and drive
log are bundled so every "Try with the sample…" button works offline.
Before zipping, the built .exe runs its own headless self-test (sample
audit + a test plan on 2 worker processes); a failure stops the build.
"""

from __future__ import annotations

import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
NAME = "CACC-Studio"
ENTRY = '''import multiprocessing
import sys


def self_test(report_path):
    """Exercise the bundled engine headless; write PASS/FAIL lines to a file."""
    import os
    import tempfile
    import traceback
    from pathlib import Path

    os.environ["QT_QPA_PLATFORM"] = "offscreen"
    lines = []
    try:
        from PySide6.QtWidgets import QApplication
        app = QApplication([])
        from cacc.gui.app import MainWindow
        from cacc.gui.common import resource
        from cacc.gui.page_audit import run_audit
        from cacc.gui.page_evaluate import run_plan
        from cacc.gui.page_setup import environment_report
        MainWindow()
        bad = [t for ok, t in environment_report() if not ok]
        lines.append(("FAIL " if bad else "PASS ") + "environment " + "; ".join(bad))
        tmp = Path(tempfile.mkdtemp())
        res, paths = run_audit(str(resource("examples", "data",
                                            "synthetic_openacc_platoon.csv")),
                               30.0, 0.5, 0.1, tmp / "audit")
        n = sum(v.twin.verdict == "string-unstable" for v in res.vehicles)
        lines.append(("PASS " if n == 2 and paths["report"].is_file() else "FAIL ")
                     + f"audit sample log: {n}/{len(res.vehicles)} unstable")
        plan = resource("examples", "plans", "smoke.yaml")
        rep, _ = run_plan(plan.read_text(encoding="utf-8"), plan.parent, plan, 2,
                          tmp / "eval")
        lines.append(("PASS " if rep["verdict"] == "PASS" else "FAIL ")
                     + f"smoke plan on 2 worker processes: {rep['verdict']}")
        app.quit()
    except Exception:
        lines.append("FAIL " + traceback.format_exc())
    Path(report_path).write_text("\\n".join(lines) + "\\n", encoding="utf-8")
    return 0 if all(s.startswith("PASS") for s in lines) else 1


if __name__ == "__main__":
    multiprocessing.freeze_support()  # parallel test plans (-j N) in the .exe
    if len(sys.argv) == 3 and sys.argv[1] == "--self-test":
        sys.exit(self_test(sys.argv[2]))
    from cacc.gui import main
    sys.exit(main())
'''


def main() -> int:
    try:
        import PyInstaller.__main__ as pyinstaller
    except ImportError:
        print('PyInstaller missing: pip install -e ".[package]"', file=sys.stderr)
        return 2
    sys.path.insert(0, str(ROOT / "src"))
    import cacc

    build = ROOT / "build" / "studio"
    build.mkdir(parents=True, exist_ok=True)
    entry = build / "cacc_studio.py"
    entry.write_text(ENTRY, encoding="utf-8")
    sep = ";" if sys.platform == "win32" else ":"
    data = [f"{ROOT / 'scenarios'}{sep}scenarios",
            f"{ROOT / 'examples'}{sep}examples"]
    args = [str(entry), "--name", NAME, "--noconfirm", "--clean", "--windowed",
            "--distpath", str(ROOT / "dist"), "--workpath", str(build / "work"),
            "--specpath", str(build), "--paths", str(ROOT / "src"),
            "--collect-submodules", "cacc", "--collect-submodules", "control",
            # loaded by name at savefig time (audit report charts are SVG)
            "--hidden-import", "matplotlib.backends.backend_svg",
            "--hidden-import", "matplotlib.backends.backend_agg",
            *[x for d in data for x in ("--add-data", d)],
            *[x for m in ("tkinter", "IPython", "jupyter", "notebook", "PyQt5",
                          "PyQt6", "pytest", "pygame") for x in ("--exclude-module", m)]]
    pyinstaller.run(args)

    app_dir = ROOT / "dist" / NAME
    exe = app_dir / (NAME + (".exe" if sys.platform == "win32" else ""))
    if not exe.is_file():
        print(f"build failed: {exe} not found", file=sys.stderr)
        return 1
    # the frozen app must run the real engine end to end before we ship it
    result = build / "self_test.txt"
    result.unlink(missing_ok=True)
    code = subprocess.run([str(exe), "--self-test", str(result)], timeout=900).returncode
    print("\nself-test of the built app:\n" + (result.read_text(encoding="utf-8")
                                                if result.is_file() else "(no report)"))
    if code != 0:
        print("self-test FAILED — not packaging", file=sys.stderr)
        return 1
    archive = shutil.make_archive(
        str(ROOT / "dist" / f"{NAME}-{cacc.__version__}-win64"), "zip",
        root_dir=app_dir.parent, base_dir=app_dir.name)
    size = sum(f.stat().st_size for f in app_dir.rglob("*") if f.is_file()) / 1e6
    print(f"\nbuilt {exe}  ({size:.0f} MB unpacked)\nship  {archive}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
