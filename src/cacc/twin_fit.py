"""Least-squares fitting machinery of the drive-log twin (D-026).

Segment preparation, start values, the bounded fit of one model structure,
the AR(1)-inflated covariance and the delta-method stability margin. Used
by :func:`cacc.twin.calibrate_hop`.
"""

from __future__ import annotations

import numpy as np
from scipy import optimize

from cacc.fielddata import PlatoonLog
from cacc.twin_model import (_BURN_IN_S, _DELAY_BOUNDS, _DELAY_MIN, _HI, _LO,
                             LinearACC, simulate_follower)

def _segment_data(log: PlatoonLog, i: int, seg: slice):
    t = log.t[seg]
    v_l, v_f, gap = log.speed[seg, i - 1], log.speed[seg, i], log.gap[seg, i - 1]
    # initial acceleration from a short least-squares slope (robust to noise)
    k = min(10, t.size)
    a0 = float(np.polyfit(t[:k] - t[0], v_f[:k], 1)[0])
    return t, v_l, v_f, gap, (float(gap[0]), float(v_f[0]), a0)


def _initial_guesses(data, v_bar: float) -> list[np.ndarray]:
    """A regression-based start plus two generic ones."""
    starts = [np.array([0.1, 0.4, 1.5, 5.0 + 1.5 * v_bar, 0.5]),
              np.array([0.03, 0.2, 2.0, 5.0 + 2.0 * v_bar, 1.0])]
    X, y = [], []
    for t, v_l, v_f, gap, _ in data:
        dt = float(np.median(np.diff(t)))
        w = max(3, int(round(1.0 / dt)) | 1)  # ~1 s smoothing window, odd
        kern = np.ones(w) / w
        vs = np.convolve(v_f, kern, mode="same")
        a = np.gradient(vs, dt)
        sl = slice(w, -w or None)
        X.append(np.column_stack([np.ones_like(gap), gap, v_f, v_l - v_f])[sl])
        y.append(a[sl])
    try:
        c, *_ = np.linalg.lstsq(np.vstack(X), np.concatenate(y), rcond=None)
        if c[1] > 1e-3 and -c[2] / c[1] > 0.2:
            T = -c[2] / c[1]
            s0 = -c[0] / c[1]
            starts.insert(0, np.clip([c[1], max(c[3], 0.0), T, s0 + T * v_bar,
                                      0.5], _LO, _HI))
    except np.linalg.LinAlgError:
        pass
    return starts


def _fit(data, v_bar: float, sg: float, sv: float, with_delay: bool,
         starts: list[np.ndarray]) -> dict:
    """Bounded least squares of one structure from several starts."""
    lo = np.append(_LO, _DELAY_BOUNDS[0]) if with_delay else _LO
    hi = np.append(_HI, _DELAY_BOUNDS[1]) if with_delay else _HI

    def model_of(q) -> LinearACC:
        k_s, k_v, T, s_bar, tau = (float(x) for x in q[:5])
        delay = float(q[5]) if with_delay else 0.0
        return LinearACC(k_s, k_v, T, s_bar - T * v_bar, tau,
                         delay if delay >= _DELAY_MIN else 0.0)

    def residuals(q):
        m = model_of(q)
        out = []
        for t, v_l, v_f, gap, x0 in data:
            g_sim, v_sim = simulate_follower(m, t, v_l, x0)
            b = min(int(round(_BURN_IN_S / float(np.median(np.diff(t))))),
                    t.size // 4)
            out += [(g_sim[b:] - gap[b:]) / sg, (v_sim[b:] - v_f[b:]) / sv]
        r = np.concatenate(out)
        return np.where(np.isfinite(r), r, 1e3)

    best = None
    for q0 in starts:
        sol = optimize.least_squares(residuals, np.clip(q0, lo, hi),
                                     bounds=(lo, hi), x_scale="jac",
                                     max_nfev=400)
        if best is None or sol.cost < best.cost:
            best = sol
    r = best.fun
    rho = float(np.clip(np.corrcoef(r[:-1], r[1:])[0, 1], 0.0, 0.99))
    return {"sol": best, "model_of": model_of, "lo": lo, "hi": hi,
            "rho1": rho, "structure": "lag+delay" if with_delay else "lag"}


def _stability(fit: dict, cov_q: np.ndarray) -> tuple:
    """(T_min, margin, margin_se) of a fit, margin SE by the delta method."""
    q, model_of = fit["sol"].x, fit["model_of"]
    lo, hi = fit["lo"], fit["hi"]

    def margin_of(qq) -> float:
        mm = model_of(qq)
        t_min = mm.min_stable_time_gap()
        return np.nan if t_min is None else mm.T - t_min

    t_min = model_of(q).min_stable_time_gap()
    margin, margin_se = margin_of(q), np.nan
    if t_min is not None:
        gm = np.zeros(q.size)
        for k in range(q.size):
            dq = np.zeros(q.size)
            dq[k] = 1e-3 * max(abs(q[k]), 1e-2)
            gm[k] = (margin_of(np.clip(q + dq, lo, hi))
                     - margin_of(np.clip(q - dq, lo, hi))) / (2 * dq[k])
        margin_se = float(np.sqrt(max(gm @ cov_q @ gm, 0.0))) \
            if np.all(np.isfinite(gm)) else np.nan
    return t_min, float(margin), float(margin_se)


def _covariance(fit: dict) -> np.ndarray:
    """Parameter covariance, inflated for AR(1) residuals.

    A parameter pinned at a bound (e.g. a dead time of 0) is an active
    constraint, not an estimate: it is held fixed (zero variance) instead of
    letting its near-zero Jacobian column blow up the inverse.
    """
    sol = fit["sol"]
    m_obs, n_par = sol.fun.size, sol.x.size
    rho = fit["rho1"]
    s2 = 2 * sol.cost / max(m_obs - n_par, 1) * (1 + rho) / (1 - rho)
    span = fit["hi"] - fit["lo"]
    free = ((sol.x - fit["lo"]) > 1e-6 * span) & ((fit["hi"] - sol.x) > 1e-6 * span)
    if n_par == 6 and sol.x[5] < _DELAY_MIN:  # dead time pinned at zero
        free[5] = False
    cov = np.zeros((n_par, n_par))
    J = sol.jac[:, free]
    try:
        cov[np.ix_(free, free)] = s2 * np.linalg.inv(J.T @ J)
    except np.linalg.LinAlgError:
        cov[:] = np.nan
    return cov


def _verdict(t_min, margin, margin_se) -> str:
    """Same rule as :attr:`TwinReport.verdict`."""
    if t_min is None or margin + 2 * margin_se < 0.0:
        return "string-unstable"
    return "string-stable" if margin - 2 * margin_se > 0.0 else "marginal"
