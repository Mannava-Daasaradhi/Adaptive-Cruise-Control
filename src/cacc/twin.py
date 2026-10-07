"""Digital twin of a production ACC from a drive log (D-026).

Model (the standard linear car-following ACC abstraction used in the
field-data string-stability literature): the follower commands

    u = k_s (s - s0 - T v) + k_v (v_lead - v)

and realizes it through a first-order lag ``a' = (u - a) / tau``. This is
exactly this repository's CTHP law with ``kp = k_s``, ``kv = k_v``,
``h = T``, ``r = s0`` and **no V2V feedforward** (``ka = 0``), so a fitted
twin inherits everything the platform already has: the exact analytic
string-stability transfer (:func:`cacc.analysis.gamma`), the simulator, test
plans, and a *what-if* answer to "would V2V feedforward fix this car?".

Calibration simulates the follower against the *measured* leader speed and
fits the gap and speed trajectories (not accelerations differentiated from
noisy speeds) by bounded nonlinear least squares, over every ACC-engaged
segment of the log. Parameter standard errors come from the Jacobian,
inflated for residual autocorrelation (AR(1) effective sample size), and are
propagated to ``||Gamma||_inf``. A model-free Welch estimate of the speed
transfer is reported next to the twin's as a cross-check.

**Structure and confidence (fix of 2026-10-08).** Production ACCs also have
a pure actuation dead time, the term published field studies fit (Gunter et
al., IEEE T-ITS). Each hop is therefore fitted twice, with the lag model and
with lag + dead time (``delay``; simulated with a second-order Pade
approximation, analysed with the exact ``e^{-delay s}``), and the structures
are compared by an autocorrelation-aware BIC (:mod:`cacc.twin_checks`). The
report carries the chosen structure, the alternative's verdict, and a
``confidence`` level with warnings: tiny error bars on a twin that misfits a
real car are the failure this guards against.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field

import numpy as np
from scipy import signal

from cacc.fielddata import PlatoonLog
from cacc.twin_checks import BIC_COMPARABLE, assess, bic, r_squared
from cacc.twin_fit import (_covariance, _fit, _initial_guesses, _segment_data,
                           _stability, _verdict)
from cacc.twin_model import (_DELAY_MIN, _LO, PARAMS, PARAMS_DELAY, STRUCTURES,
                             UNITS, LinearACC, simulate_follower)

__all__ = ["LinearACC", "PARAMS", "PARAMS_DELAY", "STRUCTURES", "TwinReport",
           "UNITS", "calibrate_hop", "calibrate_log", "empirical_gain",
           "simulate_follower", "synthesize_log"]

_log = logging.getLogger(__name__)


@dataclass
class TwinReport:
    """Calibration result for one follower of a log."""

    follower: str
    leader: str
    hop: int
    source: str
    duration_s: float
    segments: int
    model: LinearACC
    se: dict  # standard error per parameter
    gap_rmse: float  # [m]
    speed_rmse: float  # [m/s]
    v_mean: float  # mean follower speed [m/s]
    hinf: float
    hinf_se: float
    peak_omega: float
    empirical: dict = field(default_factory=dict)  # Welch cross-check
    min_time_gap: float | None = None  # smallest string-stable T at these gains
    margin: float = float("nan")  # T - min_time_gap [s]
    margin_se: float = float("nan")
    what_if: dict = field(default_factory=dict)  # V2V feedforward what-if
    structure: str = "lag"  # model structure chosen by the BIC
    alternative: dict = field(default_factory=dict)  # the other structure
    confidence: str = "high"  # high / medium / low (cacc.twin_checks)
    warnings: list = field(default_factory=list)
    r2_gap: float = float("nan")  # share of gap variation explained
    r2_speed: float = float("nan")
    resid_rho1: float = 0.0  # lag-1 autocorrelation of the residuals

    @property
    def verdict(self) -> str:
        """Decided on the time-gap margin at 2 standard errors.

        ``||Gamma||_inf`` of a stable lag ACC sits at exactly 1 (at zero
        frequency) whatever its gains, so its standard error cannot express
        how close the design is to the boundary; the margin ``T - T_min``
        can. ``string-unstable`` / ``string-stable`` when resolved,
        ``marginal`` otherwise.
        """
        if self.min_time_gap is None:  # unstable at every T <= 10 s
            return "string-unstable"
        if self.margin + 2 * self.margin_se < 0.0:
            return "string-unstable"
        if self.margin - 2 * self.margin_se > 0.0:
            return "string-stable"
        return "marginal"

    def to_dict(self) -> dict:
        m = self.model
        names = PARAMS_DELAY if self.structure == "lag+delay" else PARAMS
        return {
            "follower": self.follower, "leader": self.leader, "hop": self.hop,
            "source": self.source, "duration_s": round(self.duration_s, 1),
            "segments": self.segments,
            "structure": self.structure,
            "confidence": self.confidence, "warnings": list(self.warnings),
            "params": {p: round(getattr(m, p), 5) for p in names},
            "param_se": {p: round(v, 5) for p, v in self.se.items()},
            "units": UNITS,
            "fit": {"gap_rmse_m": round(self.gap_rmse, 4),
                    "speed_rmse_mps": round(self.speed_rmse, 4),
                    "r2_gap": _round(self.r2_gap, 4),
                    "r2_speed": _round(self.r2_speed, 4),
                    "residual_lag1_autocorr": round(self.resid_rho1, 3)},
            "v_mean_mps": round(self.v_mean, 3),
            "hinf": round(self.hinf, 5), "hinf_se": round(self.hinf_se, 5),
            "peak_omega_rad_s": round(self.peak_omega, 4),
            "verdict": self.verdict,
            "min_string_stable_time_gap_s": _round(self.min_time_gap),
            "time_gap_margin_s": _round(self.margin),
            "time_gap_margin_se_s": _round(self.margin_se, 4),
            "empirical": self.empirical, "what_if_v2v": self.what_if,
            "alternative_structure": self.alternative,
        }


def calibrate_hop(log: PlatoonLog, i: int, min_duration: float = 30.0,
                  v2v_ka: float = 0.5, v2v_delay: float = 0.1,
                  structures: tuple[str, ...] = STRUCTURES) -> TwinReport:
    """Fit a :class:`LinearACC` twin to follower ``i`` (1-based) of ``log``.

    ``structures`` = ``("lag",)`` fits only the classic lag model;
    the default also fits lag + dead time and keeps the better one by BIC.
    """
    if not structures or "lag" not in structures or not set(structures) \
            <= set(STRUCTURES):
        raise ValueError(f"structures must include 'lag' and be a subset of "
                         f"{STRUCTURES}")
    segs = log.hop_segments(i, min_duration)
    if not segs:
        raise ValueError(f"follower {i} ({log.names[i]}): no ACC-engaged "
                         f"segment of >= {min_duration:g} s with valid data")
    data = [_segment_data(log, i, s) for s in segs]
    v_bar = float(np.mean(np.concatenate([d[2] for d in data])))
    sg = max(float(np.std(np.concatenate([d[3] for d in data]))), 0.5)
    sv = max(float(np.std(np.concatenate([d[2] for d in data]))), 0.1)

    fits = {"lag": _fit(data, v_bar, sg, sv, False,
                        _initial_guesses(data, v_bar))}
    if "lag+delay" in structures:  # nested: start from the lag solution
        q = fits["lag"]["sol"].x
        fits["lag+delay"] = _fit(data, v_bar, sg, sv, True,
                                 [np.append(q, 0.05), np.append(q, 0.3)])
    # one effective sample size for every structure (that of the simplest),
    # so the BIC values are on the same scale and comparable
    m_obs, rho_ref = fits["lag"]["sol"].fun.size, fits["lag"]["rho1"]
    for f in fits.values():
        f["bic"] = bic(f["sol"].cost, m_obs, f["sol"].x.size, rho_ref)
    # Occam: the dead-time structure becomes the primary twin only on very
    # strong evidence (its two extra Pade states can also soak up a noisy
    # initial transient). Closer than that, both stay plausible: the lag
    # twin is primary and the audit plans for the worse of the two.
    name = ("lag+delay" if "lag+delay" in fits
            and fits["lag+delay"]["bic"] < fits["lag"]["bic"] - BIC_COMPARABLE
            else "lag")
    fit = fits[name]
    q, model = fit["sol"].x, fit["model_of"](fit["sol"].x)
    n_par = q.size
    cov_q = _covariance(fit)
    # map q-covariance to the reported parameters (s0 = s_bar - T v_bar)
    M = np.eye(n_par)
    M[3, 2], M[3, 3] = -v_bar, 1.0
    cov_p = M @ cov_q @ M.T
    names = PARAMS_DELAY if name == "lag+delay" else PARAMS
    se = {p: float(np.sqrt(max(cov_p[k, k], 0.0))) for k, p in enumerate(names)}

    hinf, w_pk = model.hinf()
    grad = np.zeros(n_par)
    for k in range(n_par):  # d hinf / d q by central differences
        dq = np.zeros(n_par)
        dq[k] = 1e-4 * max(abs(q[k]), 1e-3)
        grad[k] = (fit["model_of"](np.clip(q + dq, fit["lo"], fit["hi"])).hinf()[0]
                   - fit["model_of"](np.clip(q - dq, fit["lo"], fit["hi"])).hinf()[0]
                   ) / (2 * dq[k])
    hinf_se = float(np.sqrt(max(grad @ cov_q @ grad, 0.0)))
    t_min, margin, margin_se = _stability(fit, cov_q)

    gap_err = np.concatenate([simulate_follower(model, t, v_l, x0)[0] - gap
                              for t, v_l, v_f, gap, x0 in data])
    spd_err = np.concatenate([simulate_follower(model, t, v_l, x0)[1] - v_f
                              for t, v_l, v_f, gap, x0 in data])
    r2_gap = r_squared(gap_err, np.concatenate([d[3] for d in data]))
    r2_speed = r_squared(spd_err, np.concatenate([d[2] for d in data]))

    alt = {}
    others = [k for k in fits if k != name]
    if others:
        other = fits[others[0]]
        a_model = other["model_of"](other["sol"].x)
        a_tmin, a_margin, a_se = _stability(other, _covariance(other))
        a_names = PARAMS_DELAY if others[0] == "lag+delay" else PARAMS
        alt = {"structure": others[0],
               "params": {p: round(getattr(a_model, p), 5) for p in a_names},
               "delta_bic": round(other["bic"] - fit["bic"], 2),
               "comparable": bool(abs(other["bic"] - fit["bic"])
                                  < BIC_COMPARABLE),
               "verdict": _verdict(a_tmin, a_margin, a_se),
               "min_string_stable_time_gap_s": _round(a_tmin),
               "hinf": round(a_model.hinf()[0], 5)}
    confidence, warns = assess(r2_gap, r2_speed, fit["rho1"],
                               _verdict(t_min, margin, margin_se), t_min,
                               alt or None)
    if name == "lag+delay" and (model.tau <= 1.01 * _LO[4]
                                or model.delay < _DELAY_MIN):
        warns.append("lag and dead time are not separately identified from "
                     "this data (one sits on its bound); their sum "
                     f"({model.tau + model.delay:.2f} s) is, and the verdict "
                     "does not depend on the split")

    h_v2v, w_v2v = model.hinf(v2v_ka, v2v_delay)
    rep = TwinReport(
        follower=log.names[i], leader=log.names[i - 1], hop=i,
        source=log.source,
        duration_s=float(sum(d[0][-1] - d[0][0] for d in data)),
        segments=len(segs), model=model, se=se,
        gap_rmse=float(np.sqrt(np.mean(gap_err**2))),
        speed_rmse=float(np.sqrt(np.mean(spd_err**2))),
        v_mean=v_bar, hinf=hinf, hinf_se=hinf_se, peak_omega=w_pk,
        empirical=empirical_gain(log, i, segs),
        min_time_gap=t_min, margin=margin, margin_se=margin_se,
        what_if={"ka": v2v_ka, "delay_s": v2v_delay, "hinf": round(h_v2v, 5),
                 "peak_omega_rad_s": round(w_v2v, 4),
                 "min_string_stable_time_gap_s": _round(
                     model.min_stable_time_gap(v2v_ka, v2v_delay))},
        structure=name, alternative=alt, confidence=confidence,
        warnings=warns, r2_gap=r2_gap, r2_speed=r2_speed,
        resid_rho1=fit["rho1"])
    _log.info("twin %s (%s, confidence %s): k_s=%.4f k_v=%.3f T=%.2f "
              "s0=%.1f tau=%.2f delay=%.2f |G|=%.4f", rep.follower, name,
              confidence, model.k_s, model.k_v, model.T, model.s0, model.tau,
              model.delay, hinf)
    return rep


def _round(x, n=3):
    return None if x is None else round(float(x), n)


def empirical_gain(log: PlatoonLog, i: int, segs: list[slice],
                   max_omega: float = 1.5, min_coherence: float = 0.6) -> dict:
    """Model-free Welch estimate of |V_i / V_{i-1}| over the segments.

    Reports the peak gain over frequencies up to ``max_omega`` whose
    coherence is at least ``min_coherence`` (where the leader actually
    excited the follower); ``None`` when no frequency qualifies.
    """
    fs = 1.0 / log.dt
    lengths = [s.stop - s.start for s in segs]
    # one Welch segment length for all data segments -> one frequency grid
    nper = int(min(1024, max(64, max(lengths) // 4)))
    usable = [s for s, n in zip(segs, lengths, strict=True) if n >= nper]
    if not usable:
        return {"peak_gain": None, "note": "segments too short for Welch"}
    Pxx = Pyy = Pxy = 0.0
    for s in usable:  # length-weighted average of the spectra
        x = signal.detrend(log.speed[s, i - 1])
        y = signal.detrend(log.speed[s, i])
        freqs, pxx = signal.welch(x, fs, nperseg=nper)
        _, pyy = signal.welch(y, fs, nperseg=nper)
        _, pxy = signal.csd(x, y, fs, nperseg=nper)
        Pxx, Pyy, Pxy = Pxx + pxx * x.size, Pyy + pyy * x.size, Pxy + pxy * x.size
    omega = 2 * np.pi * freqs
    coh = np.abs(Pxy) ** 2 / np.maximum(Pxx * Pyy, 1e-30)
    gain = np.abs(Pxy) / np.maximum(Pxx, 1e-30)
    ok = (omega > 0) & (omega <= max_omega) & (coh >= min_coherence)
    if not ok.any():
        return {"peak_gain": None,
                "note": f"no frequency with coherence >= {min_coherence}"}
    k = np.flatnonzero(ok)[np.argmax(gain[ok])]
    return {"peak_gain": round(float(gain[k]), 4),
            "peak_omega_rad_s": round(float(omega[k]), 4),
            "coherence": round(float(coh[k]), 3),
            "band_rad_s": [round(float(omega[ok].min()), 4),
                           round(float(omega[ok].max()), 4)]}


def calibrate_log(log: PlatoonLog, min_duration: float = 30.0,
                  **kw) -> tuple[list[TwinReport], list[str]]:
    """Calibrate every follower with usable data; returns (reports, skipped)."""
    reports, skipped = [], []
    for i in range(1, log.n_vehicles):
        try:
            reports.append(calibrate_hop(log, i, min_duration, **kw))
        except ValueError as exc:
            skipped.append(str(exc))
    return reports, skipped


def synthesize_log(models: list[LinearACC], t: np.ndarray,
                   v_lead: np.ndarray, names: list[str] | None = None,
                   speed_noise: float = 0.03, gap_noise: float = 0.05,
                   seed: int = 0, engaged: np.ndarray | None = None
                   ) -> PlatoonLog:
    """A heterogeneous platoon log: follower ``i`` runs ``models[i-1]``
    behind the (measured) speed of vehicle ``i-1``, starting in equilibrium;
    GNSS-like white noise is added to the recorded speeds and gaps.

    Used for demos and ground-truth tests of :func:`calibrate_hop`.
    """
    rng = np.random.default_rng(seed)
    speeds, gaps = [np.asarray(v_lead, float)], []
    for m in models:
        v0 = float(speeds[-1][0])
        g, v = simulate_follower(m, t, speeds[-1], (m.s0 + m.T * v0, v0, 0.0))
        speeds.append(v)
        gaps.append(g)
    speed = np.column_stack(speeds)
    gap = np.column_stack(gaps)
    speed = speed + rng.normal(0.0, speed_noise, speed.shape)
    gap = gap + rng.normal(0.0, gap_noise, gap.shape)
    n = len(models) + 1
    names = names or ["leader"] + [f"follower_{i}" for i in range(1, n)]
    if engaged is None:
        engaged = np.ones((t.size, n), bool)
        engaged[:, 0] = False
    return PlatoonLog(np.asarray(t, float), speed, gap, list(names), engaged,
                      "synthetic")
