"""Test plans: evaluate a controller against a scenario matrix (D-025).

A *test plan* is the product-level input: which controller is under test,
which scenario it runs in, which conditions to sweep, how many seeds, and the
acceptance criteria. Evaluating it yields a verdict per case and overall,
plus machine-readable evidence (:mod:`cacc.reporting`). Schema::

    name: my-release-gate
    description: ...
    base: ../../scenarios/ma2025_sine.yaml   # or  scenario: {inline mapping}
    under_test:
      controller: cthp                  # built-in, 'pkg.mod:Name' or 'file.py:Name'
      options: {}                       # plugin constructor kwargs
    seeds: 5                            # N -> 1..N, or an explicit list
    overrides: {platoon.n_followers: 4} # dot paths into the scenario, all cases
    cases:                              # optional named variants (default: one)
      - name: hard-brake
        set: {leader: {profile: brake, decel: -6.0, t_start: 5, duration: 2}}
    matrix:                             # cartesian product, applied to each case
      network.delay: [0.0, 0.1, 0.2]
    criteria: ["min_gap >= 2.0", "min_ttc >= 1.5"]
    string_stability:                   # optional black-box frequency sweep
      max_gain: 1.0
      sweep: {n_followers: 3}           # cacc.stringstab.SweepConfig fields

Relative paths (``base``, a ``file.py`` plugin) resolve against the plan's
directory, so a plan and its controller can be moved together.
"""

from __future__ import annotations

import copy
import itertools
import logging
import time
import traceback
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, field, replace
from pathlib import Path

import yaml

from cacc.controllers import BUILTIN_KINDS, is_plugin_kind, make_controller
from cacc.criteria import METRICS, Criterion
from cacc.platoon import PlatoonSim
from cacc.scenario import SCENARIO_BLOCKS, scenario_from_dict
from cacc.stringstab import SweepConfig, measure_string_stability

log = logging.getLogger(__name__)

PLAN_KEYS = ("name", "description", "base", "scenario", "under_test", "seeds",
             "overrides", "cases", "matrix", "criteria", "string_stability")


class PlanError(ValueError):
    """The test plan is malformed (bad key, path, criterion, override...)."""


@dataclass
class Case:
    """One concrete scenario variant of a plan."""

    name: str
    raw: dict  # full scenario mapping after overrides / case set / matrix
    params: dict = field(default_factory=dict)  # what this case varied


@dataclass
class TestPlan:
    """A parsed, validated test plan."""

    __test__ = False  # not a pytest test class, despite the name

    name: str
    description: str
    controller: str
    options: dict
    seeds: list[int]
    cases: list[Case]
    criteria: list[Criterion]
    sweep: SweepConfig | None = None
    max_gain: float = 1.0
    source: Path | None = None


# ----------------------------------------------------------------- loading
def set_path(tree: dict, path: str, value) -> None:
    """Set ``tree['a']['b'] = value`` for ``path = 'a.b'``, creating levels.

    The first segment must be a scenario block, so typos fail loudly.
    """
    keys = path.split(".")
    if keys[0] not in SCENARIO_BLOCKS:
        raise PlanError(f"override {path!r}: {keys[0]!r} is not a scenario "
                        f"block ({', '.join(SCENARIO_BLOCKS)})")
    node = tree
    for k in keys[:-1]:
        nxt = node.get(k)
        if nxt is None:
            nxt = node[k] = {}
        elif not isinstance(nxt, dict):
            raise PlanError(f"override {path!r}: {k!r} is not a mapping")
        node = nxt
    node[keys[-1]] = copy.deepcopy(value)


def _resolve_controller(ref: str, root: Path) -> str:
    if not is_plugin_kind(ref):
        if ref.lower() not in BUILTIN_KINDS:
            raise PlanError(f"unknown controller {ref!r}: use one of "
                            f"{BUILTIN_KINDS} or a plugin 'module:Name'")
        return ref.lower()
    mod, _, attr = ref.rpartition(":")
    if mod.endswith(".py") and not Path(mod).is_absolute():
        mod = str((root / mod).resolve())
    return f"{mod}:{attr}"


def _short(path: str, all_paths: list[str]) -> str:
    """Last dot segment, unless it is ambiguous among the matrix axes."""
    last = path.rsplit(".", 1)[-1]
    clash = sum(p.rsplit(".", 1)[-1] == last for p in all_paths) > 1
    return path if clash else last


def load_plan(path: str | Path) -> TestPlan:
    """Parse and validate a test-plan YAML file."""
    path = Path(path)
    with open(path, encoding="utf-8") as f:
        raw = yaml.safe_load(f) or {}
    return plan_from_dict(raw, root=path.parent, source=path)


def plan_from_dict(raw: dict, root: Path = Path("."),
                   source: Path | None = None) -> TestPlan:
    """Build a :class:`TestPlan` from a parsed mapping (paths vs ``root``)."""
    unknown = sorted(set(raw) - set(PLAN_KEYS))
    if unknown:
        raise PlanError(f"unknown plan key(s) {unknown}; allowed: "
                        f"{', '.join(PLAN_KEYS)}")
    if ("base" in raw) == ("scenario" in raw):
        raise PlanError("give exactly one of 'base' (scenario file) or "
                        "'scenario' (inline mapping)")
    if "base" in raw:
        base_path = (root / raw["base"]).resolve()
        if not base_path.is_file():
            raise PlanError(f"base scenario not found: {base_path}")
        base = yaml.safe_load(base_path.read_text(encoding="utf-8"))
    else:
        base = copy.deepcopy(raw["scenario"])
    for k, v in (raw.get("overrides") or {}).items():
        set_path(base, k, v)

    sut = raw.get("under_test") or {}
    if "controller" not in sut:
        raise PlanError("under_test.controller is required")
    controller = _resolve_controller(str(sut["controller"]), root)
    options = dict(sut.get("options") or {})

    seeds = raw.get("seeds", 1)
    seeds = list(range(1, int(seeds) + 1)) if isinstance(seeds, int) else [
        int(s) for s in seeds]
    if not seeds:
        raise PlanError("need at least one seed")

    try:
        criteria = [Criterion.parse(c) for c in raw.get("criteria") or []]
    except (ValueError, KeyError, TypeError) as exc:
        raise PlanError(f"criteria: {exc}") from None

    sweep, max_gain = None, 1.0
    ss = raw.get("string_stability")
    if ss:
        ss = dict(ss) if isinstance(ss, dict) else {}
        max_gain = float(ss.pop("max_gain", 1.0))
        try:
            sweep = SweepConfig(**{k: tuple(v) if isinstance(v, list) else v
                                   for k, v in (ss.pop("sweep", None)
                                                or {}).items()})
        except TypeError as exc:
            raise PlanError(f"string_stability.sweep: {exc}") from None
        if ss:
            raise PlanError(f"unknown string_stability keys {sorted(ss)}")
    if not criteria and sweep is None:
        raise PlanError("the plan checks nothing: add criteria and/or "
                        "string_stability")

    variants = raw.get("cases") or [{"name": "base"}]
    matrix = raw.get("matrix") or {}
    axes = list(matrix)
    cases: list[Case] = []
    for var in variants:
        extra = set(var) - {"name", "set"}
        if extra or "name" not in var:
            raise PlanError(f"a case needs 'name' and optional 'set' "
                            f"(got keys {sorted(var)})")
        for combo in itertools.product(*(matrix[a] for a in axes)):
            tree = copy.deepcopy(base)
            for k, v in (var.get("set") or {}).items():
                set_path(tree, k, v)
            params = {}
            for axis, v in zip(axes, combo, strict=True):
                set_path(tree, axis, v)
                params[_short(axis, axes)] = v
            label = var["name"]
            if params:
                label += "[" + ",".join(f"{k}={v}" for k, v in params.items()) + "]"
            cases.append(Case(label, tree, params))
    if not cases:
        raise PlanError("the plan has no cases (an empty matrix axis?)")
    needs_pair = any(c.metric == "l2_amplification_max" for c in criteria)
    for case in cases:  # build once now so a bad case fails before any run
        try:
            sc = scenario_from_dict(case.raw, default_name=case.name)
        except (ValueError, TypeError, KeyError) as exc:
            raise PlanError(f"case {case.name!r}: {exc}") from None
        if needs_pair and sc.config.n_followers < 2:
            raise PlanError(f"case {case.name!r}: l2_amplification_max "
                            "compares followers and needs n_followers >= 2")
    try:  # import + contract-check the controller before any run
        make_controller(controller, sc.config.control, options)
    except (ImportError, OSError, ValueError, TypeError, AttributeError) as exc:
        raise PlanError(f"under_test: {exc}") from None
    return TestPlan(name=str(raw.get("name") or (source.stem if source else
                                                   "plan")),
                    description=str(raw.get("description") or "").strip(),
                    controller=controller, options=options, seeds=seeds,
                    cases=cases, criteria=criteria, sweep=sweep,
                    max_gain=max_gain, source=source)


# ----------------------------------------------------------------- running
def _run_seed(raw: dict, controller: str, options: dict, seed: int,
              metrics: tuple[str, ...]) -> dict:
    """Worker: one simulation -> the requested metric values."""
    t0 = time.perf_counter()
    try:
        sc = scenario_from_dict(raw)
        cfg = replace(sc.config, seed=seed)
        res = PlatoonSim(cfg, controller, sc.leader, options).run()
        vals = {m: METRICS[m].fn(res) for m in metrics}
        return {"values": vals, "elapsed_s": time.perf_counter() - t0}
    except Exception:  # noqa: BLE001 — report, don't crash the whole plan
        return {"error": traceback.format_exc(limit=6)}


def _run_sweep(raw: dict, controller: str, options: dict,
               sweep: SweepConfig) -> dict:
    """Worker: one black-box string-stability sweep."""
    t0 = time.perf_counter()
    try:
        cfg = scenario_from_dict(raw).config
        out = measure_string_stability(cfg, controller, sweep, options)
        return {"result": out.to_dict(), "elapsed_s": time.perf_counter() - t0}
    except Exception:  # noqa: BLE001
        return {"error": traceback.format_exc(limit=6)}


def evaluate(plan: TestPlan, jobs: int = 1) -> dict:
    """Run every (case, seed) and sweep of ``plan``; return the report dict.

    The report is plain JSON-able data; :mod:`cacc.reporting` renders it.
    """
    metrics = tuple(dict.fromkeys(c.metric for c in plan.criteria))
    tasks = []
    for ci, case in enumerate(plan.cases):
        if metrics:
            for seed in plan.seeds:
                tasks.append((("run", ci, seed), _run_seed,
                              (case.raw, plan.controller, plan.options, seed,
                               metrics)))
        if plan.sweep is not None:
            tasks.append((("sweep", ci, None), _run_sweep,
                          (case.raw, plan.controller, plan.options,
                           plan.sweep)))
    log.info("plan %r: %d case(s) x %d seed(s), %d job(s) on %d worker(s)",
             plan.name, len(plan.cases), len(plan.seeds), len(tasks), jobs)
    t0 = time.perf_counter()
    if jobs > 1 and len(tasks) > 1:
        with ProcessPoolExecutor(max_workers=jobs) as pool:
            futs = [(key, pool.submit(fn, *args)) for key, fn, args in tasks]
            outputs = {key: fut.result() for key, fut in futs}
    else:
        outputs = {key: fn(*args) for key, fn, args in tasks}

    cases = [_case_report(plan, ci, case, outputs)
             for ci, case in enumerate(plan.cases)]
    verdicts = {c["verdict"] for c in cases}
    verdict = ("ERROR" if "ERROR" in verdicts else
               "FAIL" if "FAIL" in verdicts else "PASS")
    return {
        "plan": plan.name,
        "description": plan.description,
        "source": str(plan.source) if plan.source else None,
        "under_test": {"controller": plan.controller, "options": plan.options},
        "seeds": plan.seeds,
        "criteria": [{"label": c.label, "metric": c.metric, "op": c.op,
                      "value": c.value, "unit": METRICS[c.metric].unit}
                     for c in plan.criteria],
        "string_stability": (None if plan.sweep is None else
                             {"max_gain": plan.max_gain,
                              "sweep": _sweep_dict(plan.sweep)}),
        "verdict": verdict,
        "counts": {v: sum(c["verdict"] == v for c in cases)
                   for v in ("PASS", "FAIL", "ERROR")},
        "cases": cases,
        "elapsed_s": round(time.perf_counter() - t0, 2),
    }


def _sweep_dict(sw: SweepConfig) -> dict:
    return {"base_period": sw.base_period, "harmonics": list(sw.harmonics),
            "amplitude": sw.amplitude, "n_followers": sw.n_followers,
            "first_hop": sw.first_hop}


def _case_report(plan: TestPlan, ci: int, case: Case, outputs: dict) -> dict:
    checks, errors = [], []
    runs = [outputs[("run", ci, s)] for s in plan.seeds] if plan.criteria else []
    for seed, out in zip(plan.seeds, runs, strict=True):
        if "error" in out:
            errors.append({"seed": seed, "error": out["error"]})
    ok_runs = [(s, o) for s, o in zip(plan.seeds, runs, strict=True)
               if "error" not in o]
    for crit in plan.criteria:
        vals = [o["values"][crit.metric] for _, o in ok_runs]
        chk = {"label": crit.label, "metric": crit.metric, "op": crit.op,
               "threshold": crit.value, "unit": METRICS[crit.metric].unit,
               "per_seed": {str(s): v for (s, _), v in zip(ok_runs, vals,
                                                           strict=True)}}
        if vals:
            worst, i = crit.worst(vals)
            chk.update(worst=worst, worst_seed=ok_runs[i][0],
                       passed=crit.passes(worst))
        else:
            chk.update(worst=None, worst_seed=None, passed=False)
        checks.append(chk)
    ss = None
    if plan.sweep is not None:
        out = outputs[("sweep", ci, None)]
        if "error" in out:
            errors.append({"seed": None, "error": out["error"]})
        else:
            r = out["result"]
            se = r["peak_gain_se"]
            ss = dict(r, max_gain=plan.max_gain,
                      passed=r["peak_gain"] <= plan.max_gain,
                      # verdict not resolved by the measurement noise
                      marginal=se is not None
                      and abs(r["peak_gain"] - plan.max_gain) <= 2 * se)
    passed = all(c["passed"] for c in checks) and (ss is None or ss["passed"])
    return {"name": case.name, "params": case.params,
            "verdict": "ERROR" if errors else ("PASS" if passed else "FAIL"),
            "checks": checks, "string_stability": ss, "errors": errors,
            "scenario": case.raw}
