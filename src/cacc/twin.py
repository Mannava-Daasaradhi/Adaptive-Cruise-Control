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
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field

import numpy as np
from scipy import optimize, signal

from cacc.analysis import OMEGA_DEFAULT, gamma_magnitude, min_stable_headway
from cacc.fielddata import PlatoonLog

_log = logging.getLogger(__name__)

PARAMS = ("k_s", "k_v", "T", "s0", "tau")
UNITS = {"k_s": "1/s^2", "k_v": "1/s", "T": "s", "s0": "m", "tau": "s"}
# fit vector q = (k_s, k_v, T, s_bar, tau), s_bar = s0 + T * v_bar: the
# equilibrium gap at the mean speed is far better identified than s0 itself
_LO = np.array([1e-3, 0.0, 0.2, 0.5, 0.05])
_HI = np.array([3.0, 3.0, 5.0, 300.0, 3.0])


@dataclass(frozen=True)
class LinearACC:
    """Linear lag ACC: u = k_s (s - s0 - T v) + k_v dv, a' = (u - a)/tau."""

    k_s: float
    k_v: float
    T: float
    s0: float
    tau: float

    def gain(self, omega: np.ndarray, ka: float = 0.0,
             theta: float = 0.0) -> np.ndarray:
        """|Gamma(j omega)|; ``ka``/``theta`` add V2V feedforward (what-if)."""
        return gamma_magnitude(omega, "cthp", self.T, kp=self.k_s, kv=self.k_v,
                               ka_eff=ka, tau=self.tau, theta=theta)

    def hinf(self, ka: float = 0.0, theta: float = 0.0) -> tuple[float, float]:
        """(||Gamma||_inf, frequency of the peak [rad/s])."""
        g = self.gain(OMEGA_DEFAULT, ka, theta)
        k = int(np.argmax(g))
        return float(g[k]), float(OMEGA_DEFAULT[k])

    def min_stable_time_gap(self, ka: float = 0.0,
                            theta: float = 0.0) -> float | None:
        """Smallest time gap T at which these gains are string-stable."""
        try:
            return min_stable_headway("cthp", theta=theta, kp=self.k_s,
                                      kv=self.k_v, ka_eff=ka, tau=self.tau,
                                      lo=0.05, hi=10.0)
        except ValueError:
            return None

    def scenario(self, v0: float, name: str = "twin",
                 description: str = "") -> dict:
        """A scenario mapping that runs this twin with the built-in CTHP law
        (``cacc sweep twin.yaml -c cthp``, or ``base:`` of a test plan)."""
        return {
            "name": name,
            "description": description,
            "platoon": {"n_followers": 5, "v0": round(float(v0), 3)},
            "vehicle": {"tau": round(self.tau, 4), "length": 4.0,
                        "u_min": -8.0, "u_max": 3.0},
            "controller": {"kp": round(self.k_s, 5), "kv": round(self.k_v, 5),
                           "ka": 0.0, "h": round(self.T, 4),
                           "r": round(self.s0, 3)},
            "network": {"delay": 0.0},
            "leader": {"profile": "brake", "t_start": 10.0, "duration": 3.0,
                       "decel": -3.0},
            "sim": {"dt": 0.01, "t_final": 120.0, "seed": 1},
        }


def simulate_follower(model: LinearACC, t: np.ndarray, v_lead: np.ndarray,
                      x0: tuple[float, float, float]) -> tuple[np.ndarray,
                                                                np.ndarray]:
    """Gap and speed of ``model`` following the measured ``v_lead``.

    Exact LTI solution (first-order-hold input) from the initial state
    ``x0 = (gap, speed, acceleration)``.
    """
    k_s, k_v, T, s0, tau = (model.k_s, model.k_v, model.T, model.s0, model.tau)
    A = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, 1.0],
                  [k_s / tau, -(k_s * T + k_v) / tau, -1.0 / tau]])
    B = np.array([[1.0, 0.0], [0.0, 0.0], [k_v / tau, -k_s * s0 / tau]])
    sys = signal.StateSpace(A, B, np.eye(3), np.zeros((3, 2)))
    u = np.column_stack([v_lead, np.ones_like(v_lead)])
    _, y, _ = signal.lsim(sys, u, t - t[0], X0=np.asarray(x0, float))
    return y[:, 0], y[:, 1]


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
        return {
            "follower": self.follower, "leader": self.leader, "hop": self.hop,
            "source": self.source, "duration_s": round(self.duration_s, 1),
            "segments": self.segments,
            "params": {p: round(getattr(m, p), 5) for p in PARAMS},
            "param_se": {p: round(v, 5) for p, v in self.se.items()},
            "units": UNITS,
            "fit": {"gap_rmse_m": round(self.gap_rmse, 4),
                    "speed_rmse_mps": round(self.speed_rmse, 4)},
            "v_mean_mps": round(self.v_mean, 3),
            "hinf": round(self.hinf, 5), "hinf_se": round(self.hinf_se, 5),
            "peak_omega_rad_s": round(self.peak_omega, 4),
            "verdict": self.verdict,
            "min_string_stable_time_gap_s": _round(self.min_time_gap),
            "time_gap_margin_s": _round(self.margin),
            "time_gap_margin_se_s": _round(self.margin_se, 4),
            "empirical": self.empirical, "what_if_v2v": self.what_if,
        }


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


def calibrate_hop(log: PlatoonLog, i: int, min_duration: float = 30.0,
                  v2v_ka: float = 0.5, v2v_delay: float = 0.1) -> TwinReport:
    """Fit a :class:`LinearACC` twin to follower ``i`` (1-based) of ``log``."""
    segs = log.hop_segments(i, min_duration)
    if not segs:
        raise ValueError(f"follower {i} ({log.names[i]}): no ACC-engaged "
                         f"segment of >= {min_duration:g} s with valid data")
    data = [_segment_data(log, i, s) for s in segs]
    v_bar = float(np.mean(np.concatenate([d[2] for d in data])))
    sg = max(float(np.std(np.concatenate([d[3] for d in data]))), 0.5)
    sv = max(float(np.std(np.concatenate([d[2] for d in data]))), 0.1)

    def model_of(q) -> LinearACC:
        k_s, k_v, T, s_bar, tau = (float(x) for x in q)
        return LinearACC(k_s, k_v, T, s_bar - T * v_bar, tau)

    def residuals(q):
        m = model_of(q)
        out = []
        for t, v_l, v_f, gap, x0 in data:
            g_sim, v_sim = simulate_follower(m, t, v_l, x0)
            out += [(g_sim - gap) / sg, (v_sim - v_f) / sv]
        r = np.concatenate(out)
        return np.where(np.isfinite(r), r, 1e3)

    best = None
    for q0 in _initial_guesses(data, v_bar):
        sol = optimize.least_squares(residuals, q0, bounds=(_LO, _HI),
                                     x_scale="jac", max_nfev=400)
        if best is None or sol.cost < best.cost:
            best = sol
    q, model = best.x, model_of(best.x)
    r = best.fun
    m_obs, n_par = r.size, q.size
    # AR(1) inflation: time-series residuals are strongly autocorrelated
    rho = float(np.clip(np.corrcoef(r[:-1], r[1:])[0, 1], 0.0, 0.99))
    infl = (1 + rho) / (1 - rho)
    s2 = 2 * best.cost / max(m_obs - n_par, 1) * infl
    J = best.jac
    try:
        cov_q = s2 * np.linalg.inv(J.T @ J)
    except np.linalg.LinAlgError:
        cov_q = np.full((n_par, n_par), np.nan)
    # map q-covariance to the reported parameters (s0 = s_bar - T v_bar)
    M = np.eye(5)
    M[3, 2], M[3, 3] = -v_bar, 1.0
    cov_p = M @ cov_q @ M.T
    se = {p: float(np.sqrt(max(cov_p[k, k], 0.0))) for k, p in enumerate(PARAMS)}

    hinf, w_pk = model.hinf()
    grad = np.zeros(n_par)
    for k in range(n_par):  # d hinf / d q by central differences
        dq = np.zeros(n_par)
        dq[k] = 1e-4 * max(abs(q[k]), 1e-3)
        grad[k] = (model_of(np.clip(q + dq, _LO, _HI)).hinf()[0]
                   - model_of(np.clip(q - dq, _LO, _HI)).hinf()[0]) / (2 * dq[k])
    hinf_se = float(np.sqrt(max(grad @ cov_q @ grad, 0.0)))

    def margin_of(qq) -> float:
        mm = model_of(qq)
        t_min = mm.min_stable_time_gap()
        return np.nan if t_min is None else mm.T - t_min

    t_min = model.min_stable_time_gap()
    margin, margin_se = margin_of(q), np.nan
    if t_min is not None:
        gm = np.zeros(n_par)
        for k in range(n_par):
            dq = np.zeros(n_par)
            dq[k] = 1e-3 * max(abs(q[k]), 1e-2)
            gm[k] = (margin_of(np.clip(q + dq, _LO, _HI))
                     - margin_of(np.clip(q - dq, _LO, _HI))) / (2 * dq[k])
        margin_se = float(np.sqrt(max(gm @ cov_q @ gm, 0.0))) \
            if np.all(np.isfinite(gm)) else np.nan

    gap_err = np.concatenate([simulate_follower(model, t, v_l, x0)[0] - gap
                              for t, v_l, v_f, gap, x0 in data])
    spd_err = np.concatenate([simulate_follower(model, t, v_l, x0)[1] - v_f
                              for t, v_l, v_f, gap, x0 in data])
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
        min_time_gap=t_min, margin=float(margin), margin_se=float(margin_se),
        what_if={"ka": v2v_ka, "delay_s": v2v_delay, "hinf": round(h_v2v, 5),
                 "peak_omega_rad_s": round(w_v2v, 4),
                 "min_string_stable_time_gap_s": _round(
                     model.min_stable_time_gap(v2v_ka, v2v_delay))})
    _log.info("twin %s: k_s=%.4f k_v=%.3f T=%.2f s0=%.1f tau=%.2f |G|=%.4f",
              rep.follower, *(getattr(model, p) for p in PARAMS), hinf)
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
