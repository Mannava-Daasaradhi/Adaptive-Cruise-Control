"""Auto-tuner: search a Controller Lab design until the stress cells pass.

Say what you want, e.g. *"no collision, TTC >= 1.5 s, string-stable even at
150 ms latency, as short a headway as possible"*, and the tuner searches the
numbers in your blocks:

* **what is tuned** — every numeric constant in the four blocks other than
  the structural ``0``/``1`` (``0.2/(h*s + 1)`` tunes the 0.2, not the 1),
  plus the headway ``h``; each in a log box around its start value (a
  factor ``spread`` either side; ``h`` in ``h_range``);
* **stage 1, analytic** (milliseconds per candidate): a quasi-random
  (Sobol) sample is filtered by the physics the Lab already computes — all
  closed-loop poles in the left half-plane and peak |Γ| <= ``max_gain`` at
  every V2V delay in ``thetas`` — and ranked by headway, then damping;
* **stage 2, simulated**: the best few survivors run the target stress
  cells (default: hard brake, emergency stop on a wet road, hard brake with
  300 ms latency, a cut-in), judged by the criteria; the winner is the
  shortest-headway design that passes every cell, ties broken by margin.

The result keeps the block structure you wrote, so it stays readable: a
tuned design is the same transfer functions with new numbers.
"""

from __future__ import annotations

import ast
import math
import re
from dataclasses import dataclass, field

import numpy as np

from cacc import campaign as cp
from cacc import tfctrl as tc

DEFAULT_CELLS = (("hard-brake", "nominal"), ("emergency-stop", "wet"),
                 ("hard-brake", "latency-300ms"), ("cruise", "cut-in"))
_NUM = re.compile(r"(?<![\w.])(\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?(?![\w.])")


@dataclass
class Knob:
    """One tunable number: where it sits in the design and its range."""

    signal: str  # block name, or "h"
    index: int  # occurrence of a number in that block's text
    start: float
    lo: float
    hi: float


@dataclass
class TuneResult:
    design: tc.Design
    start: tc.Design
    passed: bool
    cells: list[dict] = field(default_factory=list)
    peak_gain: float = math.nan
    screened: int = 0
    feasible: int = 0
    simulated: int = 0
    log: list[str] = field(default_factory=list)

    def summary(self) -> str:
        head = (f"h {self.start.h:g} → {self.design.h:g} s · peak |Γ| "
                f"{self.peak_gain:.3f} · "
                + ("all target cells pass" if self.passed else
                   "no candidate passed every cell (best shown)"))
        cells = ", ".join(f"{c['cell']} {'✓' if c['verdict'] == 'PASS' else '✕'}"
                          for c in self.cells)
        return (f"{head}\n{cells}\n{self.screened} candidates screened, "
                f"{self.feasible} string-stable, {self.simulated} simulated")


def _numbers(text: str) -> list[tuple[int, int, float]]:
    """(start, end, value) of the plain numeric literals in a block text,
    skipping exponents (``s**2``)."""
    out = []
    for m in _NUM.finditer(text):
        before = text[:m.start()].rstrip()
        if before.endswith("**") or before.endswith("^"):
            continue
        out.append((m.start(), m.end(), float(m.group(0))))
    return out


def knobs(design: tc.Design, spread: float = 4.0,
          h_range: tuple[float, float] = (0.3, 2.0)) -> list[Knob]:
    """The tunable numbers of ``design`` (see module doc)."""
    out = [Knob("h", 0, design.h, *h_range)]
    for sig in tc.SIGNALS:
        for k, (_a, _b, v) in enumerate(_numbers(design.blocks.get(sig, "0"))):
            if v in (0.0, 1.0):
                continue
            out.append(Knob(sig, k, v, v / spread, v * spread))
    return out


def apply(design: tc.Design, ks: list[Knob], values) -> tc.Design:
    """A copy of ``design`` with the knob numbers replaced."""
    blocks = dict(design.blocks)
    h = design.h
    by_sig: dict[str, dict[int, float]] = {}
    for k, v in zip(ks, values, strict=True):
        if k.signal == "h":
            h = float(v)
        else:
            by_sig.setdefault(k.signal, {})[k.index] = float(v)
    for sig, repl in by_sig.items():
        text = blocks[sig]
        for idx, (a, b, _v) in reversed(list(enumerate(_numbers(text)))):
            if idx in repl:
                text = text[:a] + f"{repl[idx]:.4g}" + text[b:]
        ast.parse(text, mode="eval")  # still an expression
        blocks[sig] = text
    return tc.Design(blocks, design.ff_signal, round(h, 3), design.r,
                     design.name if design.name.endswith("-tuned")
                     else design.name + "-tuned")


def screen(design: tc.Design, tau: float, thetas, act_delay: float,
           max_gain: float) -> tuple[bool, float, float]:
    """(feasible, worst peak |Γ| over thetas, slowest pole real part)."""
    try:
        poles = tc.closed_loop_poles(design, tau)
    except (ValueError, np.linalg.LinAlgError):
        return False, math.inf, 0.0
    if not poles.size or np.any(poles.real >= -1e-6):
        return False, math.inf, 0.0
    w = np.logspace(-2.5, 1.5, 600)
    peak = max(float(np.abs(tc.gamma(w, design, tau, th, act_delay)).max())
               for th in thetas)
    return peak <= max_gain, peak, float(-poles.real.max())


def _cells_plan(design, cells, criteria, seeds, dt):
    import copy

    import yaml

    base = yaml.safe_load(cp.BASE.read_text(encoding="utf-8"))
    base.setdefault("controller", {}).update(design.controller_block)
    trees = []
    for man, cond in cells:
        t = cp.compose(copy.deepcopy(base), man, cond, tc.REF)
        t["sim"]["dt"] = dt
        trees.append((f"{man} | {cond}", t))
    return trees


def simulate_cells(design: tc.Design, cells=DEFAULT_CELLS,
                   criteria=cp.DEFAULT_CRITERIA, seeds: int = 1, dt: float = 0.02,
                   jobs: int = 1) -> list[dict]:
    """Run the target cells for one design -> [{cell, verdict, margin}]."""
    from cacc.envelope import _margin, _run

    trees = _cells_plan(design, cells, criteria, seeds, dt)
    cases = _run(trees, tc.REF, design.options, criteria, seeds, jobs)
    return [{"cell": n, "verdict": c["verdict"], "margin": _margin(c)}
            for (n, _t), c in zip(trees, cases, strict=True)]


def tune(design: tc.Design, cells=DEFAULT_CELLS, criteria=cp.DEFAULT_CRITERIA,
         tau: float = 0.2, act_delay: float = 0.0, thetas=(0.05, 0.15),
         max_gain: float = 1.0, samples: int = 256, finalists: int = 6,
         seeds: int = 1, spread: float = 4.0,
         h_range: tuple[float, float] = (0.3, 2.0), jobs: int = 1,
         seed: int = 0, polish: int = 2, progress=None) -> TuneResult:
    """Search ``design``'s numbers (see module doc)."""
    from scipy.stats import qmc

    design.tfs()  # the start must parse
    ks = knobs(design, spread, h_range)
    lo = np.log([k.lo for k in ks])
    hi = np.log([k.hi for k in ks])
    m = max(4, int(round(math.log2(max(samples, 16)))))
    pts = qmc.Sobol(len(ks), scramble=True, seed=seed).random_base2(m)
    cand = [np.array([k.start for k in ks])] + [np.exp(lo + p * (hi - lo))
                                                 for p in pts]
    tick = progress or (lambda done, total, msg="": None)
    ranked = []
    for n, vals in enumerate(cand):
        try:
            d = apply(design, ks, vals)
            ok, peak, decay = screen(d, tau, thetas, act_delay, max_gain)
        except (ValueError, SyntaxError, ZeroDivisionError):
            continue
        if ok:
            # shortest headway first, then robustness (peak) and settling
            ranked.append((d.h + 0.2 * peak - 0.05 * min(decay, 2.0), d, peak))
        if n % 32 == 0:
            tick(n, len(cand) + finalists, "screening")
    res = TuneResult(design, design, False, screened=len(cand), feasible=len(ranked))
    if not ranked:
        res.log.append("no string-stable candidate: widen spread or h_range, "
                       "or change the block structure")
        _, res.peak_gain, _ = screen(design, tau, thetas, act_delay, max_gain)
        return res
    # finalists spread over the feasible headways (best robustness in each
    # band), simulated shortest first: the first that passes every cell wins
    ranked.sort(key=lambda r: r[1].h)
    bands = np.array_split(np.arange(len(ranked)), min(finalists, len(ranked)))
    pick = [min((ranked[j] for j in band), key=lambda r: r[0] - r[1].h)
            for band in bands if band.size]
    best = None
    for k, (_s, d, peak) in enumerate(pick):
        cells_out = simulate_cells(d, cells, criteria, seeds, jobs=jobs)
        res.simulated += 1
        passed = all(c["verdict"] == "PASS" for c in cells_out)
        worst = min(c["margin"] for c in cells_out)
        res.log.append(f"h={d.h:g} peak={peak:.3f} "
                       + ("PASS" if passed else "FAIL") + f" worst margin {worst:+.2f}")
        tick(len(cand) + k + 1, len(cand) + len(pick), "simulating")
        if best is None or worst > best[0]:
            best = (worst, d, peak, cells_out, passed)
        if passed:
            best = (worst, d, peak, cells_out, passed)
            break
    if best[4]:  # polish: a couple of shorter-headway feasible designs
        tried = {id(r[1]) for r in pick}
        lo_h = max((r[1].h for r in pick if r[1].h < best[1].h), default=0.0)
        shorter = sorted((r for r in ranked if lo_h < r[1].h < best[1].h
                          and id(r[1]) not in tried), key=lambda r: r[0] - r[1].h)
        for _s, d, peak in sorted(shorter[:polish], key=lambda r: r[1].h):
            cells_out = simulate_cells(d, cells, criteria, seeds, jobs=jobs)
            res.simulated += 1
            ok = all(c["verdict"] == "PASS" for c in cells_out)
            worst = min(c["margin"] for c in cells_out)
            res.log.append(f"polish h={d.h:g} " + ("PASS" if ok else "FAIL"))
            if ok:
                best = (worst, d, peak, cells_out, True)
                break
    _, d, peak, cells_out, passed = best
    res.design, res.peak_gain, res.cells, res.passed = d, peak, cells_out, passed
    return res
