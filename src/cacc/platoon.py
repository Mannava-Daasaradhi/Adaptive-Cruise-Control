"""N-vehicle platoon simulator (fixed-step RK4) and scenario loading.

Topology: vehicle 0 is the **leader** with a prescribed acceleration profile
``a0(t)``; followers ``1..n`` are identical vehicles in one-vehicle look-ahead.
Under CACC each follower receives its predecessor's commanded acceleration
through a :class:`cacc.network.V2VLink` (delay / sampling / packet loss);
under ACC the feedforward is absent.

The V2V delay is handled with a transmitted-history delay line (interpolated
lookup at ``t - delay``), which is the standard numerical treatment of the
delay term in this DDE and keeps the integrator an explicit fixed-step RK4.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

import numpy as np
import yaml

from cacc.controllers import ControllerParams, make_controller
from cacc.estimation import AdaptConfig, ChannelEstimator, HeadwayAdapter
from cacc.network import V2VLink
from cacc.vehicle import N_STATES, VehicleParams

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class PlatoonConfig:
    """Full configuration of one platoon simulation."""

    n_followers: int = 5
    v0: float = 20.0  # initial platoon cruise speed [m/s]
    vehicle: VehicleParams = VehicleParams()
    control: ControllerParams = ControllerParams()
    delay: float = 0.1  # V2V latency theta [s]
    loss_prob: float = 0.0
    msg_rate: float | None = None  # None = continuous link
    noise_rho: float | None = None  # multiplicative channel noise (Ma 2025)
    noise_gammas: tuple[float, ...] | None = None  # None = paper's 16-bit channel
    noise_rate: float = 100.0  # noise-factor hold rate [Hz]
    rho_schedule: tuple | None = None  # [(t, rho), ...] time-varying channel
    adapt: AdaptConfig = AdaptConfig()  # QoS-adaptive outer loop (D-016)
    dt: float = 0.01
    t_final: float = 40.0
    seed: int = 1


@dataclass
class SimResult:
    """Time series produced by :meth:`PlatoonSim.run` (S samples).

    Column 0 of ``pos/vel/acc`` is the leader; columns ``1..n`` the followers.
    ``err`` and ``u`` have one column per follower.
    """

    t: np.ndarray  # (S,)
    pos: np.ndarray  # (S, n+1)
    vel: np.ndarray  # (S, n+1)
    acc: np.ndarray  # (S, n+1) leader column = commanded a0(t)
    err: np.ndarray  # (S, n) spacing errors
    u: np.ndarray  # (S, n) commanded accelerations (saturated)
    controller: str
    config: PlatoonConfig
    h: np.ndarray | None = None  # (S, n) per-follower headway (adaptive runs)
    rho_hat: np.ndarray | None = None  # (S, n) channel estimates (NaN = none)


@dataclass
class Scenario:
    """A named simulation case: configuration + leader maneuver."""

    name: str
    config: PlatoonConfig
    leader: Callable[[float], float]
    description: str = ""


class PlatoonSim:
    """Simulate a homogeneous platoon under ACC or CACC.

    Parameters
    ----------
    config:
        Platoon, vehicle, controller, network and integration settings.
    controller:
        ``'acc'`` or ``'cacc'``.
    leader_accel:
        Leader acceleration profile ``a0(t)`` [m/s^2]; default: constant cruise.
    """

    def __init__(
        self,
        config: PlatoonConfig,
        controller: str = "cacc",
        leader_accel: Callable[[float], float] | None = None,
    ):
        self.cfg = config
        self.kind = controller.lower()
        self.leader_accel = leader_accel or (lambda t: 0.0)
        n = config.n_followers
        if n < 1:
            raise ValueError("need at least one follower")
        self.ctrls = [make_controller(self.kind, config.control) for _ in range(n)]
        self.nc = self.ctrls[0].n_states
        # link i (0-based) carries u_{i-1} -> follower i; leader feeds link 0
        self.links = [
            V2VLink(config.delay, config.loss_prob, config.msg_rate,
                    seed=config.seed + i, noise_rho=config.noise_rho,
                    noise_gammas=config.noise_gammas, noise_rate=config.noise_rate,
                    rho_schedule=config.rho_schedule)
            for i in range(n)
        ]
        # --- QoS-adaptive outer loop (D-016): per-follower h, estimators ---
        c = config.control
        self.h_i = np.full(n, c.h)
        ad = config.adapt
        # theta_hat: timestamp-based delay estimation is exact up to clock
        # sync (measured directly in the ROS backend); the offline receiver
        # uses the true link delay
        self._theta_hat = config.delay
        self._use_pred = ad.predictor and config.delay > 0.0
        self.estimators: list[ChannelEstimator] | None = None
        self.adapters: list[HeadwayAdapter] | None = None
        if ad.enabled:
            if self.kind != "cthp":
                raise ValueError("headway adaptation is defined for 'cthp' only")
            if config.msg_rate is not None:
                raise ValueError("adaptation requires the continuous link mode")
            self.estimators = [ChannelEstimator(ad) for _ in range(n)]
            first = HeadwayAdapter(c.ka, c.kp, c.kv, config.vehicle.tau, ad,
                                   h0=c.h, theta=config.delay)
            self.adapters = [first] + [
                HeadwayAdapter(c.ka, c.kp, c.kv, config.vehicle.tau, ad,
                               h0=c.h, theta=config.delay, table=first._table)
                for _ in range(n - 1)
            ]
        if self.ctrls[0].uses_v2v and not self.links[0].passthrough:
            if config.delay > 0 and config.delay < config.dt:
                raise ValueError("delay must be 0 or >= dt for the delay-line lookup")
        self.block = N_STATES + self.nc  # states per follower
        self.n_states = 2 + n * self.block  # leader (p0, v0) + followers
        # scratch buffers filled by _deriv for recording (values at stage k1)
        self._io_e = np.zeros(n)
        self._io_edot = np.zeros(n)
        self._io_u = np.zeros(n)

    # ---------------------------------------------------------------- state
    def initial_state(self) -> np.ndarray:
        """Steady cruise at v0 with exact desired spacing (all errors zero)."""
        cfg = self.cfg
        c, veh = cfg.control, cfg.vehicle
        x = np.zeros(self.n_states)
        x[0] = 0.0  # leader position
        x[1] = cfg.v0
        gap = c.r + c.h * cfg.v0 + veh.length
        for i in range(cfg.n_followers):
            b = 2 + i * self.block
            x[b + 0] = -(i + 1) * gap  # p_i
            x[b + 1] = cfg.v0
            x[b + 2] = 0.0  # a_i
            # controller states (CACC xi) start at 0 = steady-state input
        return x

    # ------------------------------------------------------------- dynamics
    def _deriv(self, t: float, x: np.ndarray) -> np.ndarray:
        """Full state derivative; also fills the _io_* scratch buffers."""
        cfg = self.cfg
        c, veh = cfg.control, cfg.vehicle
        dx = np.zeros_like(x)
        a0 = self.leader_accel(t)
        dx[0] = x[1]
        dx[1] = a0
        p_prev, v_prev = x[0], x[1]
        a_prev = a0  # the leader realizes its commanded profile exactly
        u_live_prev = a0  # what the predecessor is commanding right now
        for i, ctrl in enumerate(self.ctrls):
            b = 2 + i * self.block
            p, v, a = x[b], x[b + 1], x[b + 2]
            xc = x[b + 3 : b + 3 + self.nc]
            h_i = self.h_i[i]  # per-follower headway (adaptive runs, D-016)
            e = (p_prev - p - veh.length) - (c.r + h_i * v)
            e_dot = v_prev - v - h_i * a
            dv = v_prev - v
            if ctrl.uses_v2v:
                link = self.links[i]
                if link.passthrough:
                    u_ff = a_prev if ctrl.ff_signal == "a" else u_live_prev
                elif self._use_pred:
                    u_ff = link.receive_predicted(t, self._theta_hat,
                                                  base=self.cfg.adapt.pred_base)
                else:
                    u_ff = link.receive(t)
            else:
                u_ff = 0.0
            u = veh.clamp(ctrl.output(xc, e, e_dot, u_ff, dv))
            dx[b] = v
            dx[b + 1] = a
            dx[b + 2] = (u - a) / veh.tau
            if self.nc:
                dx[b + 3 : b + 3 + self.nc] = ctrl.deriv(xc, e, e_dot, u_ff, dv)
            self._io_e[i] = e
            self._io_edot[i] = e_dot
            self._io_u[i] = u
            p_prev, v_prev, a_prev, u_live_prev = p, v, a, u
        return dx

    # ------------------------------------------------------------------ run
    def run(self) -> SimResult:
        """Integrate with fixed-step RK4 and return the recorded time series."""
        cfg = self.cfg
        n, dt = cfg.n_followers, cfg.dt
        steps = int(round(cfg.t_final / dt))
        S = steps + 1
        t_arr = np.arange(S) * dt
        pos = np.zeros((S, n + 1))
        vel = np.zeros((S, n + 1))
        acc = np.zeros((S, n + 1))
        err = np.zeros((S, n))
        u_arr = np.zeros((S, n))
        adapting = self.adapters is not None
        h_arr = np.tile(self.h_i, (S, 1)) if adapting else None
        rho_arr = np.full((S, n), np.nan) if adapting else None
        est_every = max(1, round(1.0 / (cfg.adapt.est_rate * dt))) if adapting else 0
        lag = round(self._theta_hat / dt)

        x = self.initial_state()
        log.info(
            "sim start: controller=%s n=%d h=%.2fs delay=%.3fs loss=%.2f mode=%s "
            "dt=%g t_final=%g",
            self.kind, n, cfg.control.h, cfg.delay, cfg.loss_prob,
            "continuous" if cfg.msg_rate is None else f"{cfg.msg_rate}Hz sampled",
            dt, cfg.t_final,
        )
        for k in range(S):
            t = k * dt
            k1 = self._deriv(t, x)  # also fills _io_* at the accepted state
            # record
            pos[k, 0], vel[k, 0] = x[0], x[1]
            acc[k, 0] = self.leader_accel(t)
            for i in range(n):
                b = 2 + i * self.block
                pos[k, i + 1], vel[k, i + 1], acc[k, i + 1] = x[b], x[b + 1], x[b + 2]
            err[k] = self._io_e
            u_arr[k] = self._io_u
            # broadcast over the links (leader feeds link 0); CACC shares the
            # commanded acceleration u, CTHP the realized acceleration a
            if self.ctrls[0].uses_v2v:
                self.links[0].send(t, acc[k, 0])
                use_a = self.ctrls[0].ff_signal == "a"
                for i in range(1, n):
                    self.links[i].send(t, acc[k, i] if use_a else u_arr[k, i - 1])
            # QoS-adaptive outer loop (D-016): estimate the channel from
            # receiver-side pairs (noisy delayed feedforward vs radar accel
            # timestamp-aligned theta_hat in the past) and slew the headway
            if adapting and k % est_every == 0 and k >= lag:
                est_dt = est_every * dt
                for i in range(n):
                    est, adp = self.estimators[i], self.adapters[i]
                    est.add_sample(t, self.links[i].receive(t), acc[k - lag, i])
                    self.h_i[i] = adp.update(est_dt, est.rho_hat(t))
            if adapting:
                h_arr[k] = self.h_i
                rho_arr[k] = [
                    (e.rho_hat() or np.nan) for e in self.estimators
                ]
            if k == steps:
                break
            # classic RK4 step
            k2 = self._deriv(t + dt / 2, x + dt / 2 * k1)
            k3 = self._deriv(t + dt / 2, x + dt / 2 * k2)
            k4 = self._deriv(t + dt, x + dt * k3)
            x = x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

        peak = np.max(np.abs(err), axis=0)
        log.info("sim done: peak |e_i| per follower = %s m", np.array2string(peak, precision=3))
        return SimResult(t_arr, pos, vel, acc, err, u_arr, self.kind, cfg,
                         h=h_arr, rho_hat=rho_arr)


# ------------------------------------------------------------ scenario I/O
def make_leader_profile(spec: dict) -> Callable[[float], float]:
    """Build a leader acceleration profile ``a0(t)`` from a scenario dict.

    Supported profiles:
      * ``brake``: constant ``decel`` for ``duration`` seconds from ``t_start``
      * ``sine``: ``amplitude * sin(2*pi*freq_hz*(t - t_start))`` from
        ``t_start``, optionally only for ``duration`` seconds
      * ``constant``: fixed ``value``
    """
    kind = spec["profile"].lower()
    if kind == "brake":
        t0 = float(spec.get("t_start", 5.0))
        dur = float(spec.get("duration", 2.0))
        dec = float(spec.get("decel", -4.0))
        base = lambda t: dec if t0 <= t < t0 + dur else 0.0  # noqa: E731
    elif kind == "sine":
        t0 = float(spec.get("t_start", 5.0))
        amp = float(spec.get("amplitude", 1.0))
        f = float(spec.get("freq_hz", 0.1))
        dur = spec.get("duration")  # seconds of sine; None = forever
        t1 = np.inf if dur is None else t0 + float(dur)
        base = (lambda t: amp * np.sin(2 * np.pi * f * (t - t0))  # noqa: E731
                if t0 <= t < t1 else 0.0)
    elif kind == "constant":
        val = float(spec.get("value", 0.0))
        base = lambda t: val  # noqa: E731
    elif kind == "bursts":
        # several sine bursts: [[t_start, duration, amplitude, freq_hz], ...]
        bursts = [tuple(float(v) for v in b) for b in spec["bursts"]]

        def base(t: float, _b=bursts) -> float:
            for t0, dur, amp, f in _b:
                if t0 <= t < t0 + dur:
                    return amp * np.sin(2 * np.pi * f * (t - t0))
            return 0.0
    else:
        raise ValueError(f"unknown leader profile: {kind!r}")
    return _with_probe(base, spec)


def _with_probe(base: Callable[[float], float], spec: dict) -> Callable[[float], float]:
    """Superpose a small multi-sine probing dither on a leader profile.

    Channel-noise estimation needs persistent excitation (w-samples exist
    only while the communicated acceleration is nonzero, D-016); a
    ``probe_amplitude`` of a few cm/s^2 keeps the estimators fed without
    disturbing the platoon meaningfully.
    """
    amp = float(spec.get("probe_amplitude", 0.0))
    if amp <= 0.0:
        return base
    w1, w2 = 2 * np.pi * 0.31, 2 * np.pi * 0.73  # incommensurate tones
    return lambda t: base(t) + 0.5 * amp * (np.sin(w1 * t) + np.sin(w2 * t))


def load_scenario(path: str | Path) -> Scenario:
    """Load a YAML scenario file into a :class:`Scenario`."""
    path = Path(path)
    with open(path, encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    net = dict(raw.get("network", {}))
    if net.get("rho_schedule") is not None:
        net["rho_schedule"] = tuple(tuple(x) for x in net["rho_schedule"])
    cfg = PlatoonConfig(
        vehicle=VehicleParams(**raw.get("vehicle", {})),
        control=ControllerParams(**raw.get("controller", {})),
        adapt=AdaptConfig(**raw.get("adaptation", {})),
        **raw.get("platoon", {}),
        **net,
        **raw.get("sim", {}),
    )
    leader = make_leader_profile(raw["leader"])
    name = raw.get("name", path.stem)
    log.info("loaded scenario %r from %s", name, path)
    return Scenario(name=name, config=cfg, leader=leader,
                    description=raw.get("description", ""))
