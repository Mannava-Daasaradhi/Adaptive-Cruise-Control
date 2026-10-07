"""HTML rendering of the String-Stability Audit report (D-027).

Split out of :mod:`cacc.audit` (which computes every number and writes the
files) so that each module stays small: this one turns an
:class:`cacc.audit.AuditResult` into one self-contained, printable,
theme-aware HTML page with inline SVG charts. Nothing here computes a
verdict; every number comes from :mod:`cacc.audit` / :mod:`cacc.twin`.
"""

from __future__ import annotations

import html
import io
import math

import numpy as np

from cacc.analysis import gamma_magnitude
from cacc.audit import (BUDGET_MAX, KA_GRID, LATENCY_GRID, AuditResult,
                        VehicleAudit)
from cacc.audit_style import CSS

# reference categorical slots 1-3 (validated all-pairs, light surface) and
# the recessive ink used for reference lines and axes
_BLUE, _ORANGE, _AQUA = "#2a78d6", "#eb6834", "#1baf7a"
_INK, _MUTED, _GRID = "#0b0b0b", "#52514e", "#e4e3df"
#: chart literals rewritten to CSS tokens so the charts follow the theme
_THEME_TOKENS = {_INK: "var(--ink)", _MUTED: "var(--muted)",
                 _GRID: "var(--line)", _BLUE: "var(--s1)",
                 _ORANGE: "var(--s2)", _AQUA: "var(--s3)",
                 "#ffffff": "var(--surface)", "#000000": "var(--ink)"}



# ------------------------------------------------------------------ charts
def _figure(w: float, h: float, rows: int = 1):
    from matplotlib.figure import Figure  # no pyplot: no global GUI state

    fig = Figure(figsize=(w, h))
    axes = fig.subplots(rows, 1, sharex=True, squeeze=False)[:, 0]
    for ax in axes:
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        for side in ("left", "bottom"):
            ax.spines[side].set_color(_MUTED)
        ax.tick_params(which="both", colors=_MUTED, labelsize=8)
        ax.grid(True, color=_GRID, linewidth=0.6)
        ax.set_axisbelow(True)
    return fig, axes


def _svg(fig) -> str:
    import matplotlib

    buf = io.StringIO()
    with matplotlib.rc_context({"svg.fonttype": "none",
                                "font.family": "sans-serif",
                                "svg.hashsalt": "cacc-audit"}):
        fig.savefig(buf, format="svg", bbox_inches="tight", transparent=True,
                    metadata={"Date": None, "Creator": None, "Format": None,
                              "Type": None})
    s = buf.getvalue()
    s = s[s.index("<svg"):]  # drop the XML prolog; inline in HTML
    for literal, token in _THEME_TOKENS.items():
        s = s.replace(literal, token)
    return s


def _end_labels(ax, x_end: float, items: list[tuple[float, str, str | None]],
                min_sep: float = 0.08) -> None:
    """Direct labels at the right end of lines, nudged apart vertically.

    ``items`` are (y at the line end, text, marker color or None); the axis
    limits must already be final. ``min_sep`` is a fraction of the y-span.
    """
    lo, hi = ax.get_ylim()
    sep = min_sep * (hi - lo)
    order = sorted(items, key=lambda it: it[0])
    ys = [it[0] for it in order]
    for k in range(1, len(ys)):
        ys[k] = max(ys[k], ys[k - 1] + sep)
    over = ys[-1] - (hi - 0.5 * sep) if ys else 0.0
    if over > 0:
        ys = [y - over for y in ys]
    for (y, text, color), y_txt in zip(order, ys, strict=True):
        if color is not None:
            ax.plot([x_end], [y], "o", ms=3.5, color=color, clip_on=False)
        ax.annotate(text, (x_end, y_txt), xytext=(6, 0),
                    textcoords="offset points", va="center", fontsize=8,
                    color=_INK, annotation_clip=False)


#: points drawn per time-series line (keeps the report small; a line is
#: visually exact well below the log's sample count)
_MAX_POINTS = 800


def fit_chart(v: VehicleAudit) -> str:
    tr = v.trace
    k = max(1, tr["t"].size // _MAX_POINTS)
    tr = {key: val[::k] for key, val in tr.items()}
    t = tr["t"] - tr["t"][0]
    fig, (a1, a2) = _figure(6.6, 3.6, rows=2)
    for ax, meas, twin, unit in ((a1, tr["gap"], tr["gap_twin"], "gap [m]"),
                                 (a2, tr["speed"], tr["speed_twin"],
                                  "speed [m/s]")):
        ax.plot(t, meas, color=_BLUE, lw=1.4, label="measured")
        ax.plot(t, twin, color=_ORANGE, lw=1.4, label="twin")
        ax.set_ylabel(unit, fontsize=8, color=_MUTED)
    a1.legend(frameon=False, fontsize=8, loc="lower right", ncol=2,
              bbox_to_anchor=(1.0, 1.0), borderaxespad=0.2, labelcolor=_INK)
    a2.set_xlabel("time in segment [s]", fontsize=8, color=_MUTED)
    return _svg(fig)


def gain_chart(v: VehicleAudit) -> str:
    r, m = v.twin, v.twin.model
    w = np.logspace(-2, np.log10(3.0), 400)
    fig, (ax,) = _figure(6.6, 2.8)
    ax.axhline(1.0, color=_MUTED, lw=1.0, ls=(0, (4, 3)))
    g0 = m.gain(w)
    ax.semilogx(w, g0, color=_BLUE, lw=1.6)
    ka, th = r.what_if["ka"], r.what_if["delay_s"]
    g1 = gamma_magnitude(w, "cthp", m.T, kp=m.k_s, kv=m.k_v, ka_eff=ka,
                         tau=m.tau, theta=th)
    ax.semilogx(w, g1, color=_ORANGE, lw=1.6)
    emp = r.empirical
    if emp.get("peak_gain") is not None:
        ax.plot([emp["peak_omega_rad_s"]], [emp["peak_gain"]], "D", ms=6,
                color=_AQUA, markeredgecolor="white", markeredgewidth=1.0)
        ax.annotate("model-free peak (data)",
                    (emp["peak_omega_rad_s"], emp["peak_gain"]),
                    xytext=(6, 6), textcoords="offset points", fontsize=8,
                    color=_INK)
    ax.set_xlabel("frequency ω [rad/s]", fontsize=8, color=_MUTED)
    ax.set_ylabel("speed gain |Γ(jω)|", fontsize=8, color=_MUTED)
    top = max(1.1, float(np.nanmax(g0)), float(np.nanmax(g1)),
              emp.get("peak_gain") or 0.0)
    ax.set_ylim(0.0, top * 1.12)
    ax.set_xlim(w[0], w[-1])
    _end_labels(ax, w[-1], [(float(g0[-1]), "twin (radar only)", _BLUE),
                            (float(g1[-1]),
                             f"with V2V ka={ka:g}, {th * 1000:.0f} ms", _ORANGE),
                            (1.0, "limit |Γ| = 1", None)])
    return _svg(fig)


def latency_chart(v: VehicleAudit) -> str:
    T = v.twin.model.T
    fig, (ax,) = _figure(6.6, 2.8)
    colors = dict(zip(KA_GRID, (_BLUE, _ORANGE, _AQUA), strict=True))
    lat = np.array(LATENCY_GRID) * 1000
    ymax, ends = T, [(T, f"in use {T:.2f} s", None)]
    for ka in KA_GRID:
        y = np.array([row["min_time_gap_s"] if row["min_time_gap_s"] is not None
                      else np.nan for row in v.grid if row["ka"] == ka], float)
        if np.isfinite(y).any():
            ymax = max(ymax, float(np.nanmax(y)))
        ax.plot(lat, y, "-o", color=colors[ka], lw=1.6, ms=4.5,
                markeredgecolor="white", markeredgewidth=0.8)
        if np.isfinite(y[-1]):
            ends.append((float(y[-1]), f"ka = {ka:g}", colors[ka]))
    ax.axhline(T, color=_INK, lw=1.0, ls=(0, (4, 3)))
    ax.set_xlabel("V2V latency [ms]", fontsize=8, color=_MUTED)
    ax.set_ylabel("smallest stable time gap [s]", fontsize=8, color=_MUTED)
    top = min(ymax, 4 * T) * 1.15
    ax.set_ylim(0.0, top)
    ax.set_xlim(lat[0] - 10, lat[-1] + 10)
    _end_labels(ax, lat[-1] + 10, [e for e in ends if e[0] <= top])
    return _svg(fig)


# ------------------------------------------------------------------ report
_VERDICT = {"string-unstable": ("critical", "✕", "String-unstable"),
            "string-stable": ("good", "✓", "String-stable"),
            "marginal": ("warning", "!", "Marginal")}


def _e(x) -> str:
    return html.escape(str(x))


def _badge(verdict: str) -> str:
    cls, icon, text = _VERDICT[verdict]
    return f'<span class="badge {cls}"><b>{icon}</b> {text}</span>'


def _s(x, fmt="{:.2f}", unit=" s", none="—") -> str:
    return none if x is None or (isinstance(x, float) and not math.isfinite(x)) \
        else fmt.format(x) + unit


def _recommendation(v: VehicleAudit) -> str:
    r, T = v.twin, v.twin.model.T
    best = v.best_v2v
    v2v = ""
    if best is not None:
        ka, b = best
        lim = ("any latency up to 1 s" if b >= BUDGET_MAX
               else f"V2V latency up to {b * 1000:.0f} ms")
        v2v = (f"V2V feedforward (ka = {ka:g}) keeps the current {T:.2f} s "
               f"gap string-stable for {lim}.")
    if r.verdict == "string-unstable":
        if v.recommended_T is None:
            return ("No time gap up to 10 s is string-stable with these gains: "
                    "retune them. " + v2v).strip()
        return (f"Raise the time gap to ≥ {v.recommended_T:.1f} s, or add "
                "V2V. " + (v2v or "None of the tested feedforward gains "
                           f"suffices at the current {T:.2f} s gap."))
    if r.verdict == "marginal":
        return ("Too close to the boundary to call: log more varied driving "
                f"or keep ≥ {_s(v.recommended_T, '{:.1f}')}. " + v2v)
    return f"Keeps a {r.margin:.2f} s margin. " + v2v


def _grid_table(v: VehicleAudit) -> str:
    T = v.twin.model.T
    head = "".join(f"<th>{th * 1000:.0f} ms</th>" for th in LATENCY_GRID)
    rows = []
    for ka in KA_GRID:
        cells = []
        for row in (x for x in v.grid if x["ka"] == ka):
            t_min = row["min_time_gap_s"]
            if t_min is None:
                cells.append('<td class="na">none ≤ 10 s</td>')
            elif t_min <= T:
                cells.append(f'<td class="ok"><b>{t_min:.2f} s ✓</b></td>')
            else:
                cells.append(f"<td>{t_min:.2f} s</td>")
        b = v.budgets[ka]
        budget = ("—" if b is None else "≥ 1000 ms" if b >= BUDGET_MAX
                  else f"{b * 1000:.0f} ms")
        rows.append(f"<tr><th>ka = {ka:g}</th>{''.join(cells)}"
                    f"<td class='num'>{budget}</td></tr>")
    return ('<div class="scroll"><table class="grid"><thead><tr>'
            '<th>feedforward</th>' + head
            + "<th>latency budget</th></tr></thead><tbody>" + "".join(rows)
            + "</tbody></table></div>")


def _road_sentence(rt: dict, which: str) -> str:
    if rt["collision"]:
        what = (f"<b>car {rt['min_gap_car']} collides</b> (smallest gap "
                f"{rt['min_gap_m']:.1f} m)")
    else:
        what = (f"the smallest gap is {rt['min_gap_m']:.1f} m "
                f"(car {rt['min_gap_car']})")
    return (f"At the {which} {rt['time_gap_s']:.2f} s time gap, {what}; "
            f"hardest braking {rt['peak_decel_mps2']:.1f} m/s².")


def _road_html(v: VehicleAudit) -> str:
    if not v.road:
        return ""
    rt = v.road["as_calibrated"]
    parts = [_road_sentence(rt, "current")]
    if "recommended" in v.road:
        parts.append(_road_sentence(v.road["recommended"], "recommended"))
    return (f'<p class="note"><b>On the road.</b> {rt["cars"]} cars tuned like '
            f"this one; the lead car brakes at {rt['brake_mps2']:g} m/s² for "
            f"{rt['brake_s']:g} s from {rt['v0_mps']:.1f} m/s. "
            + " ".join(parts) + "</p>")


_PARAM_DOC = (("k_s", "1/s²", "gap-error gain"), ("k_v", "1/s",
              "speed-difference gain"), ("T", "s", "time gap in use"),
              ("s0", "m", "standstill-gap intercept"),
              ("tau", "s", "actuator / powertrain lag"),
              ("delay", "s", "actuation dead time"))


def _vehicle_section(v: VehicleAudit) -> str:
    r = v.twin
    m = r.model
    params = "".join(
        f"<tr><td>{p}</td><td class='num'>{getattr(m, p):.4g}</td>"
        f"<td class='num'>± {r.se[p]:.2g}</td><td>{u}</td><td>{d}</td></tr>"
        for p, u, d in _PARAM_DOC if p in r.se)
    law = ("a′ = (u(t − delay) − a) / tau" if r.structure == "lag+delay"
           else "a′ = (u − a) / tau")
    alt = r.alternative or {}
    alt_txt = (f" The {alt['structure']} structure fits "
               f"{'about as well' if alt['comparable'] else 'worse'} "
               f"(ΔBIC {alt['delta_bic']:+.1f}) and says {alt['verdict']}."
               if alt else "")
    warn = "".join(f"<li>{_e(w)}</li>" for w in r.warnings)
    warn_html = f"<ul class='warn'>{warn}</ul>" if warn else ""
    emp = r.empirical
    emp_txt = (f"{emp['peak_gain']:.3f} at {emp['peak_omega_rad_s']:.3f} rad/s "
               f"(coherence {emp['coherence']:.2f})"
               if emp.get("peak_gain") is not None
               else f"unavailable ({_e(emp.get('note', ''))})")
    need = ("no time gap up to 10 s" if r.min_time_gap is None
            else f"{r.min_time_gap:.2f} s")
    se = ("SE < 0.01 s" if r.margin_se < 0.005 else
          f"SE {r.margin_se:.2f} s") if math.isfinite(r.margin_se) else "SE n/a"
    margin = "—" if r.min_time_gap is None else f"{r.margin:+.2f} s"
    return f"""
<section class="vehicle">
  <h2>{_e(r.follower)} <span class="sub">behind {_e(r.leader)}</span>
    {_badge(r.verdict)}</h2>
  <div class="kpis">
    <div><span>time gap in use</span><b>{m.T:.2f} s</b></div>
    <div><span>needs (radar only)</span><b>{need}</b></div>
    <div><span>margin ({se})</span><b>{margin}</b></div>
    <div><span>‖Γ‖∞ of the twin</span><b>{r.hinf:.3f}</b></div>
    <div><span>confidence</span><b>{_e(r.confidence)}</b></div>
  </div>
  {warn_html}
  <p class="rec"><b>Recommendation.</b> {_e(_recommendation(v))}</p>
  {_road_html(v)}
  <h3>Calibrated twin</h3>
  <p class="note">{r.duration_s:.0f} s of ACC-engaged driving in
    {r.segments} segment(s), mean speed {r.v_mean:.1f} m/s. Model
    ({_e(r.structure)}): u = k_s (s − s0 − T v) + k_v Δv, {law}.{_e(alt_txt)}
    Gap / speed variation explained: R² {r.r2_gap:.3f} / {r.r2_speed:.3f}.</p>
  <table><thead><tr><th>parameter</th><th>value</th><th>std. error</th>
    <th>unit</th><th>meaning</th></tr></thead><tbody>{params}</tbody></table>
  <h3>Fit to the log</h3>
  <p class="note">The twin is driven by the measured lead-vehicle speed only;
    residuals: gap RMSE {r.gap_rmse:.3g} m, speed RMSE {r.speed_rmse:.3g} m/s
    (longest segment shown).</p>
  <figure>{fit_chart(v)}</figure>
  <h3>Frequency response</h3>
  <p class="note">Speed gain from the lead vehicle to this car. Above 1 at any
    frequency, a disturbance grows down a line of such cars. Model-free
    cross-check from the raw speeds: {emp_txt}.</p>
  <figure>{gain_chart(v)}</figure>
  <h3>What V2V would change</h3>
  <p class="note">Smallest string-stable time gap if the car received its
    predecessor's acceleration with feedforward gain ka at the given latency.
    <b>✓</b> = the current {m.T:.2f} s gap would be string-stable. Latency
    budget = the largest latency that still allows the current gap.</p>
  {_grid_table(v)}
  <figure>{latency_chart(v)}</figure>
</section>"""


_CSS = CSS



def _road_cell(v: VehicleAudit) -> str:
    rt = v.road.get("as_calibrated")
    if rt is None:
        return "—"
    return ("<b>collision</b>" if rt["collision"]
            else f"{rt['min_gap_m']:.1f} m")


def render_html(res: AuditResult) -> str:
    """The self-contained report (inline CSS and SVG, no network needed)."""
    log, meta = res.log, res.meta
    rows = []
    for v in res.vehicles:
        r = v.twin
        need = ("> 10 s" if r.min_time_gap is None
                else f"{r.min_time_gap:.2f} s")
        emp = r.empirical.get("peak_gain")
        rows.append(
            f"<tr><td><b>{_e(r.follower)}</b></td><td>{_badge(r.verdict)}</td>"
            f"<td class='num'>{r.model.T:.2f} s</td><td class='num'>{need}</td>"
            f"<td class='num'>{_s(r.margin, '{:+.2f}')}</td>"
            f"<td class='num'>{r.hinf:.3f}</td>"
            f"<td class='num'>{'—' if emp is None else f'{emp:.3f}'}</td>"
            f"<td class='num'>{_road_cell(v)}</td>"
            f"<td>{_e(_recommendation(v))}</td></tr>")
    n_bad = sum(v.twin.verdict == "string-unstable" for v in res.vehicles)
    n_crash = sum(v.road.get("as_calibrated", {}).get("collision", False)
                  for v in res.vehicles)
    headline = (f"{n_bad} of {len(res.vehicles)} calibrated vehicle(s) "
                "amplify speed disturbances (string-unstable)." if n_bad else
                f"None of the {len(res.vehicles)} calibrated vehicle(s) is "
                "string-unstable.")
    if n_crash:
        headline += (f" In a five-car line braking at 3 m/s², {n_crash} of "
                     "them end in a collision at the current tuning.")
    skipped = "".join(f"<li>{_e(s)}</li>" for s in res.skipped)
    skipped_html = (f"<h3>Not calibrated</h3><ul class='note'>{skipped}</ul>"
                    if skipped else "")
    sections = "".join(_vehicle_section(v) for v in res.vehicles)
    return f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>String-Stability Audit — {_e(meta['log_file'])}</title>
<style>{_CSS}</style></head>
<body><main>
<h1>String-Stability Audit</h1>
<p class="meta">{_e(meta['log_file'])} · {log.n_vehicles} vehicles ·
  {log.dt:.3g} s sampling · {log.t[-1] - log.t[0]:.0f} s ·
  generated {_e(meta['created_utc'])} by cacc {_e(meta['version'])}</p>

<h2>Summary</h2>
<p><b>{_e(headline)}</b> A car is string-stable when a speed disturbance from
the car ahead never grows as it passes through it (|Γ(jω)| ≤ 1 at every
frequency). Each verdict is taken on the time-gap margin — the time gap the car
uses minus the smallest one at which its measured control behaviour is
string-stable — at two standard errors.</p>
<div class="scroll"><table>
<thead><tr><th>vehicle</th><th>verdict</th><th>time gap in use</th>
<th>needs (radar only)</th><th>margin</th><th>‖Γ‖∞ twin</th>
<th>model-free peak</th><th>5-car brake: min gap</th>
<th>recommendation</th></tr></thead>
<tbody>{''.join(rows)}</tbody></table></div>
{skipped_html}

<h3>Method</h3>
<p class="note">For every follower with ACC engaged for at least
{meta['min_duration_s']:g} s, a linear lag ACC model (the digital twin) is
fitted by simulating the car against the measured speed of the car ahead and
matching the measured gap and speed. Parameter uncertainty comes from the fit
Jacobian, inflated for autocorrelated residuals. The string-stability boundary
and the V2V what-ifs are exact for the twin; a Welch estimate from the raw
speeds is shown as a model-free cross-check.</p>

<h3>Limits</h3>
<ul class="note">
<li>The twin is a linear small-signal model around the log's mean speed;
  stop-and-go segments should be excluded from margin claims.</li>
<li><code>s0</code> is the equilibrium intercept, not necessarily the physical
  standstill distance.</li>
<li>The model-free estimate needs enough excitation by the lead vehicle; it is
  reported only at frequencies with coherence ≥ 0.6.</li>
<li>V2V what-ifs assume the predecessor's acceleration is received with the
  stated latency and no loss; they do not model packet loss or noise.</li>
</ul>
{sections}

<section class="vehicle">
<h2>Reproduce</h2>
<p class="note">Every number in this report is produced by one command from the
log file identified below. Files next to this report: <code>calibration.json</code>
(all numbers), <code>twin_*.yaml</code> (twin scenarios) and
<code>gate_*.yaml</code> (release-gate test plans:
<code>cacc evaluate gate_….yaml</code>).</p>
<table><tbody>
<tr><td>command</td><td><code>cacc audit {_e(res.source)} --min-duration
  {meta['min_duration_s']:g} --v2v-ka {meta['what_if']['ka']:g} --v2v-delay
  {meta['what_if']['delay_s']:g}</code></td></tr>
<tr><td>log SHA-256</td><td><code>{_e(meta['log_sha256'])}</code></td></tr>
<tr><td>tool</td><td>cacc {_e(meta['version'])}</td></tr>
<tr><td>generated (UTC)</td><td>{_e(meta['created_utc'])}</td></tr>
</tbody></table>
</section>
</main></body></html>
"""
