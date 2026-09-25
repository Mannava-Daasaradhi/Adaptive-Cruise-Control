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
from dataclasses import dataclass, field, replace
from typing import Callable

import numpy as np

from cacc.controllers import ControllerParams, is_plugin_kind, make_controller
from cacc.estimation import (AdaptConfig, ChannelEstimator, GainScheduler,
                             HeadwayAdapter)
from cacc.network import LinkAttack, V2VLink
from cacc.qos_map import QoSMap
from cacc.trust import TrustConfig, TrustGate
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
    qos_map: tuple | None = None  # [(x0, x1, rho), ...] spatial channel map (D-021)
    adapt: AdaptConfig = AdaptConfig()  # QoS-adaptive outer loop (D-016)
    attack: LinkAttack | None = None  # malicious V2V injection (D-022)
    trust: TrustConfig = TrustConfig()  # physics-consistency gate (D-022)
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
    gains: np.ndarray | None = None  # (S, n, 2) live (kp, kv) when re-tuning
    trust: np.ndarray | None = None  # (S, n) trust weight g (gated runs, D-022)
    ff_inj: np.ndarray | None = None  # (S, n) injected feedforward u_ff_eff - a_radar


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
        ``'acc'``, ``'cacc'``, ``'cthp'`` or a plugin reference
        ``'module:Name'`` / ``'file.py:Name'`` (D-025).
    leader_accel:
        Leader acceleration profile ``a0(t)`` [m/s^2]; default: constant cruise.
    controller_options:
        Keyword arguments for a plugin controller's constructor.
    """

    def __init__(
        self,
        config: PlatoonConfig,
        controller: str = "cacc",
        leader_accel: Callable[[float], float] | None = None,
        controller_options: dict | None = None,
    ):
        self.cfg = config
        self.kind = controller if is_plugin_kind(controller) else controller.lower()
        self.leader_accel = leader_accel or (lambda t: 0.0)
        n = config.n_followers
        if n < 1:
            raise ValueError("need at least one follower")
        self.ctrls = [make_controller(self.kind, config.control, controller_options)
                      for _ in range(n)]
        self.nc = self.ctrls[0].n_states
        # plugins may ask for the raw measurements (gap, v, a, t) — D-025
        self._raw = bool(getattr(self.ctrls[0], "raw_inputs", False))
        # spatial channel map (D-021): drive each link's rho from where that
        # link physically is (follower i trails the leader), so a patch is
        # entered later down the string; also exposes a position preview
        self.qos_map = None
        link_scheds = [config.rho_schedule] * n
        if config.qos_map is not None:
            base = config.noise_rho if config.noise_rho is not None else 50.0
            self.qos_map = QoSMap(config.qos_map, rho_base=base)
            link_scheds = self._link_rho_schedules(config, self.qos_map)
        # link i (0-based) carries u_{i-1} -> follower i; leader feeds link 0.
        # A malicious node (D-022) tampers only with its target link.
        atk = config.attack
        self.links = [
            V2VLink(config.delay, config.loss_prob, config.msg_rate,
                    seed=config.seed + i,
                    noise_rho=(None if config.qos_map is not None
                               else config.noise_rho),
                    noise_gammas=config.noise_gammas, noise_rate=config.noise_rate,
                    rho_schedule=link_scheds[i],
                    attack=(atk if atk is not None and atk.target_link == i
                            else None))
            for i in range(n)
        ]
        # physics-consistency trust gates (D-022): one per follower, guarding
        # its incoming feedforward against V2V spoofing
        self.gates = None
        if config.trust.enabled:
            self.gates = [TrustGate(config.trust, seed_offset=i)
                          for i in range(n)]
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
            # front-first staggered recovery (D-019): follower i may start
            # shrinking h only i * stagger_s after its recovery demand
            for i, adp in enumerate(self.adapters):
                adp.stagger_delay = i * ad.stagger_s
        self.schedulers: list[GainScheduler] | None = None
        if ad.enabled and ad.adapt_gains:
            sch0 = GainScheduler(c.ka, c.kp, c.kv, config.vehicle.tau, ad,
                                 theta=config.delay)
            self.schedulers = [sch0] + [
                GainScheduler(c.ka, c.kp, c.kv, config.vehicle.tau, ad,
                              theta=config.delay, table=sch0._table)
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
        self._io_g = np.ones(n)  # trust weight g (D-022)
        self._io_ffinj = np.zeros(n)  # injected feedforward u_ff_eff - a_radar

    # ---------------------------------------------------------------- state
    def initial_state(self) -> np.ndarray:
        """Steady cruise at v0 with exact desired spacing (all errors zero)."""
        cfg = self.cfg
        x = np.zeros(self.n_states)
        x[0] = 0.0  # leader position
        x[1] = cfg.v0
        gap = self._pitch(cfg.v0)
        for i in range(cfg.n_followers):
            b = 2 + i * self.block
            x[b + 0] = -(i + 1) * gap  # p_i
            x[b + 1] = cfg.v0
            x[b + 2] = 0.0  # a_i
            # controller states (CACC xi) start at 0 = steady-state input
        return x

    def _pitch(self, v: float) -> float:
        """Front-to-front equilibrium spacing at speed ``v`` [m].

        A plugin's own ``equilibrium_gap(v)`` wins over the scenario policy
        ``r + h*v`` so the platoon starts in the law's true equilibrium.
        """
        eq = getattr(self.ctrls[0], "equilibrium_gap", None)
        c = self.cfg.control
        gap = float(eq(v)) if eq is not None else c.r + c.h * v
        return gap + self.cfg.vehicle.length

    def _link_rho_schedules(self, config: PlatoonConfig, qmap: QoSMap):
        """Per-link rho step schedules from the spatial map (D-021).

        Each link's channel is driven by the position of its follower along a
        nominal (maneuver-integrated) trajectory: follower ``i`` trails the
        leader by ``(i+1)`` standoff gaps, so it enters a patch that much
        later. Small closed-loop spacing ripples don't change which (large)
        patch a vehicle is in, so the nominal trajectory is exact enough.
        """
        n, dt = config.n_followers, config.dt
        gap0 = self._pitch(config.v0)
        tg = np.arange(0.0, config.t_final + dt, dt)
        a0 = np.array([self.leader_accel(float(t)) for t in tg])
        v = config.v0 + np.concatenate(
            [[0.0], np.cumsum(0.5 * (a0[1:] + a0[:-1]) * dt)])
        x_lead = np.concatenate([[0.0], np.cumsum(0.5 * (v[1:] + v[:-1]) * dt)])
        return [qmap.time_schedule(tg, x_lead - (i + 1) * gap0)
                for i in range(n)]

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
                # physics-consistency gate (D-022): fuse the (possibly spoofed)
                # V2V feedforward toward the independent radar estimate of the
                # predecessor's acceleration; caps the injection at r0/2
                if self.gates is not None:
                    u_ff, g_i, _r_i, a_radar = self.gates[i].apply(t, u_ff, a_prev)
                    self._io_g[i] = g_i
                    self._io_ffinj[i] = u_ff - a_radar
            else:
                u_ff = 0.0
            if self._raw:
                raw = dict(gap=p_prev - p - veh.length, v=v, a=a, t=t)
                u = veh.clamp(ctrl.output(xc, e, e_dot, u_ff, dv, **raw))
                if self.nc:
                    dx[b + 3 : b + 3 + self.nc] = ctrl.deriv(
                        xc, e, e_dot, u_ff, dv, **raw)
            else:
                u = veh.clamp(ctrl.output(xc, e, e_dot, u_ff, dv))
                if self.nc:
                    dx[b + 3 : b + 3 + self.nc] = ctrl.deriv(xc, e, e_dot, u_ff, dv)
            dx[b] = v
            dx[b + 1] = a
            dx[b + 2] = (u - a) / veh.tau
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
        retuning = self.schedulers is not None
        gains_arr = None
        if retuning:
            gains_arr = np.empty((S, n, 2))
            gains_arr[:] = (cfg.control.kp, cfg.control.kv)
        gating = self.gates is not None
        trust_arr = np.ones((S, n)) if gating else None
        ffinj_arr = np.zeros((S, n)) if gating else None
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
            if gating:
                trust_arr[k] = self._io_g
                ffinj_arr[k] = self._io_ffinj
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
                preview_on = self.qos_map is not None and cfg.adapt.preview_s > 0
                for i in range(n):
                    est, adp = self.estimators[i], self.adapters[i]
                    est.add_sample(t, self.links[i].receive(t), acc[k - lag, i])
                    rho = est.rho_hat(t)
                    # predictive QoS-map lookahead (D-021): worst channel this
                    # follower meets within preview_s s at its current speed
                    rho_prev = None
                    if preview_on:
                        b = 2 + i * self.block
                        rho_prev = self.qos_map.min_rho_ahead(
                            x[b], x[b + 1], cfg.adapt.preview_s)
                    h_req = None
                    if retuning and rho is not None:
                        # joint gain + headway adaptation (D-017): slew
                        # (kp, kv) toward the re-tuned feasible design and
                        # target the headway required at those gains
                        kp_i, kv_i, h_req = self.schedulers[i].update(
                            est_dt, adp.rho_safe_of(rho))
                        self.ctrls[i] = make_controller(
                            "cthp", replace(cfg.control, kp=kp_i, kv=kv_i))
                    self.h_i[i] = adp.update(est_dt, rho, h_required=h_req,
                                             rho_preview=rho_prev)
            if adapting:
                h_arr[k] = self.h_i
                rho_arr[k] = [
                    (e.rho_hat() or np.nan) for e in self.estimators
                ]
                if retuning:
                    gains_arr[k] = [(s.kp, s.kv) for s in self.schedulers]
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
                         h=h_arr, rho_hat=rho_arr, gains=gains_arr,
                         trust=trust_arr, ff_inj=ffinj_arr)


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


def __getattr__(name: str):
    """Backward compatibility: scenario I/O moved to :mod:`cacc.scenario`."""
    if name in ("load_scenario", "scenario_from_dict", "SCENARIO_BLOCKS"):
        from cacc import scenario

        return getattr(scenario, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
