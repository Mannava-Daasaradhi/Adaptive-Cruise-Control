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
from dataclasses import replace
from typing import Callable

import numpy as np

from cacc.aeb import AEB
from cacc.controllers import is_plugin_kind, make_controller
from cacc.estimation import ChannelEstimator, GainScheduler, HeadwayAdapter
from cacc.faults import Faults
from cacc.network import V2VLink
from cacc.qos_map import QoSMap, link_schedules
from cacc.radar import DelayLine, RadarSensor
from cacc.simtypes import PlatoonConfig, Scenario, SimResult  # noqa: F401 (re-export)
from cacc.traffic import Traffic, snapshot
from cacc.trust import TrustGate
from cacc.vehicle import N_STATES

log = logging.getLogger(__name__)


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
        for i, c in enumerate(self.ctrls):  # e.g. a hardware-in-the-loop car
            if hasattr(c, "bind_car"):
                c.bind_car(i + 1)
        # plugins may ask for the raw measurements (gap, v, a, t) — D-025
        self._raw = bool(getattr(self.ctrls[0], "raw_inputs", False))
        # the simulator reads these once, from the first follower's law
        def shape(c):
            return (c.n_states, bool(getattr(c, "raw_inputs", False)),
                    c.uses_v2v, c.ff_signal)
        if any(shape(c) != shape(self.ctrls[0]) for c in self.ctrls[1:]):
            raise ValueError(
                f"controller {controller!r} built followers that disagree on "
                "n_states / raw_inputs / uses_v2v / ff_signal; every follower "
                "must run the same kind of law")
        # spatial channel map (D-021): drive each link's rho from where that
        # link physically is (follower i trails the leader), so a patch is
        # entered later down the string; also exposes a position preview
        self.qos_map = None
        link_scheds = [config.rho_schedule] * n
        if config.qos_map is not None:
            base = config.noise_rho if config.noise_rho is not None else 50.0
            self.qos_map = QoSMap(config.qos_map, rho_base=base)
            link_scheds = link_schedules(self.qos_map, config, self.leader_accel,
                                         self._pitch(config.v0))
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
                            else None),
                    delay_jitter=config.delay_jitter,
                    loss_burst=config.loss_burst)
            for i in range(n)
        ]
        # physics-consistency trust gates (D-022): one per follower, guarding
        # its incoming feedforward against V2V spoofing
        self.gates = None
        if config.trust.enabled:
            self.gates = [TrustGate(config.trust, seed_offset=i)
                          for i in range(n)]
        # modelled radar (fix of 2026-10-08): one sensor per follower, read by
        # the trust gate and by the channel estimator; None = ideal radar
        rad = config.radar
        if rad.lag > 0.0 and rad.lag < config.dt:
            raise ValueError("radar lag must be 0 or >= dt")
        if not rad.ideal and config.trust.radar_noise > 0.0:
            raise ValueError("set radar noise in the radar block only "
                             "(trust.radar_noise is the legacy radar)")
        self.radars = (None if rad.ideal else
                       [RadarSensor(rad, seed_offset=i) for i in range(n)])
        # lag-matched comparison: the V2V value received radar.lag s ago
        self.echoes = None
        if config.trust.enabled and config.trust.match_lag:
            if rad.lag <= 0.0:
                raise ValueError("trust.match_lag needs a lagged radar "
                                 "(radar.lag > 0)")
            self.echoes = [DelayLine(rad.lag) for _ in range(n)]
        # actuation dead time: each follower realizes u(t - act_delay)
        self.act_lines = None
        if config.vehicle.act_delay > 0.0:
            if config.vehicle.act_delay < config.dt:
                raise ValueError("vehicle act_delay must be 0 or >= dt")
            self.act_lines = [DelayLine(config.vehicle.act_delay)
                              for _ in range(n)]
        self.traffic = Traffic.build(config)  # mixed traffic, None = pure platoon
        self.faults = Faults.build(config)  # sensor / powertrain faults
        self.aeb = AEB(config.aeb, n) if config.aeb.enabled else None
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
        self._io_v2v = np.zeros(n)  # pre-gate V2V value (lag-matched gate)
        self._rho_used = np.full(n, np.nan)  # sender-reported rho in use
        self._io_radar = np.zeros((n, 2))  # true (gap, closing) to what is ahead

    # ---------------------------------------------------------------- state
    def initial_state(self) -> np.ndarray:
        """Steady cruise at v0 with exact desired spacing (all errors zero)."""
        cfg = self.cfg
        x = np.zeros(self.n_states)
        x[0] = 0.0  # leader position
        x[1] = cfg.v0 if cfg.lead_v0 is None else cfg.lead_v0
        gap = self._pitch(cfg.v0)
        L, p = cfg.vehicle.length, 0.0
        for i in range(cfg.n_followers):
            b = 2 + i * self.block
            if i == 0 and cfg.gap0 is not None:  # a scene set by the scenario
                p -= cfg.gap0 + L
            else:
                p -= gap if self.traffic is None else self.traffic.gap0(i, cfg.v0, gap - L) + L
            x[b + 0] = p  # p_i
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

    # ------------------------------------------------------------- dynamics
    def _deriv(self, t: float, x: np.ndarray) -> np.ndarray:
        """Full state derivative; also fills the _io_* scratch buffers."""
        cfg = self.cfg
        c, veh = cfg.control, cfg.vehicle
        dx = np.zeros_like(x)
        a0 = self.leader_accel(t)
        dx[0] = max(x[1], 0.0)  # brakes hold a stopped car: no reversing
        dx[1] = a0 if x[1] > 0.0 or a0 > 0.0 else 0.0
        p_prev, v_prev = x[0], x[1]
        a_prev = a0  # the leader realizes its commanded profile exactly
        u_live_prev = a0  # what the predecessor is commanding right now
        for i, ctrl in enumerate(self.ctrls):
            b = 2 + i * self.block
            p, v, a = x[b], x[b + 1], x[b + 2]
            xc = x[b + 3 : b + 3 + self.nc]
            h_i = self.h_i[i]  # per-follower headway (adaptive runs, D-016)
            linked, tf = True, self.traffic
            if tf is not None:  # a cut-in car ahead, or a human (no V2V)
                p_prev, v_prev, a_prev, linked = tf.front(
                    i, t, p_prev, v_prev, a_prev, i > 0 and tf.human[i - 1])
            gap, closing = p_prev - p - veh.length, v - v_prev
            self._io_radar[i] = gap, closing
            fl = self.faults
            if fl is not None:  # what the radar reports, and whether V2V arrives
                gap, closing = fl.sense(i, t, gap, closing)
                linked = linked and fl.v2v_ok(i, t)
            e = gap - (c.r + h_i * v)
            dv = -closing
            e_dot = dv - h_i * a
            if ctrl.uses_v2v and linked:
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
                    self._io_v2v[i] = u_ff
                    a_rad = (None if self.radars is None
                             else self.radars[i].estimate(t, a_prev))
                    ref = (None if self.echoes is None
                           else self.echoes[i].read(t))
                    u_ff, g_i, _r_i, a_radar = self.gates[i].apply(
                        t, u_ff, a_prev, a_radar=a_rad, a_v2v_ref=ref)
                    self._io_g[i] = g_i
                    self._io_ffinj[i] = u_ff - a_radar
            else:
                u_ff = 0.0
            lo, hi = veh.bounds(v)
            if fl is not None:
                lo, hi = fl.bounds(i, t, lo, hi)
            human = tf is not None and tf.human[i]
            if human:
                u = tf.human_u(i, t, gap, v, v_prev, a_prev)
            elif self._raw:
                raw = dict(gap=gap, v=v, a=a, t=t)
                u = ctrl.output(xc, e, e_dot, u_ff, dv, **raw)
                if self.nc:
                    dx[b + 3 : b + 3 + self.nc] = ctrl.deriv(
                        xc, e, e_dot, u_ff, dv, **raw)
            else:
                u = ctrl.output(xc, e, e_dot, u_ff, dv)
                if self.nc:
                    dx[b + 3 : b + 3 + self.nc] = ctrl.deriv(xc, e, e_dot, u_ff, dv)
            if self.aeb is not None and not human:  # radar-only safety layer
                u = self.aeb.command(i, u, gap, closing, v, v_prev, a_prev, lo)
            u = float(min(max(u, lo), hi))
            u_act = u if self.act_lines is None else self.act_lines[i].read(t)
            u_eff = u_act - (0.0 if fl is None else fl.drag(i, t, v))
            # standstill: the brakes hold the car, they never drive it backwards
            dx[b] = max(v, 0.0)
            dx[b + 1] = 0.0 if v <= 0.0 and a < 0.0 else a
            dx[b + 2] = ((0.0 if v <= 0.0 and u_eff <= 0.0 else u_eff) - a) / veh.tau
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
        aeb_arr = np.zeros((S, n), dtype=np.int8) if self.aeb is not None else None
        ffinj_arr = np.zeros((S, n)) if gating else None
        est_every = max(1, round(1.0 / (cfg.adapt.est_rate * dt))) if adapting else 0
        lag = round(self._theta_hat / dt)

        x = self.initial_state()
        tf = self.traffic
        cut_rec = ([np.full((S, 3), np.nan) for _ in tf.cars] if tf is not None
                   else [])
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
            if tf is not None:
                tf.step(t, pos[k], vel[k], acc[k])
                for rec, row in zip(cut_rec, snapshot(tf, t), strict=True):
                    rec[k] = row
            if gating:
                trust_arr[k] = self._io_g
                ffinj_arr[k] = self._io_ffinj
            for i in range(n if self.aeb is not None or self.faults is not None else 0):
                g_true, c_true = self._io_radar[i]
                if self.faults is not None:
                    self.faults.record(i, t, g_true, c_true)
                if self.aeb is not None:
                    aeb_arr[k, i] = self.aeb.level[i]
                    self.aeb.observe(i, c_true, vel[k, i + 1])
            # delay-line histories (accepted states only): radar sees the
            # predecessor's realized acceleration, the echo keeps the pre-gate
            # V2V value, the actuator keeps the saturated command
            for i in range(n):
                if self.radars is not None:
                    self.radars[i].record(t, acc[k, i])
                if self.echoes is not None:
                    self.echoes[i].record(t, self._io_v2v[i])
                if self.act_lines is not None:
                    self.act_lines[i].record(t, u_arr[k, i])
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
                    if cfg.adapt.rho_source == "reported":
                        # the sender's own noise level, one message late
                        rho = self.links[i].rho_at(t - cfg.delay)
                        self._rho_used[i] = np.nan if rho is None else rho
                    else:
                        # radar reference at the timestamp-aligned instant:
                        # the receiver knows its own radar's filter delay, so
                        # the lag is aligned away; the radar's noise is not
                        a_ref = acc[k - lag, i]
                        if self.radars is not None:
                            a_ref += self.radars[i].noise_at(t - self._theta_hat)
                        est.add_sample(t, self.links[i].receive(t), a_ref)
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
                rho_arr[k] = (self._rho_used if cfg.adapt.rho_source == "reported"
                              else [(e.rho_hat() or np.nan)
                                    for e in self.estimators])
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
        stats = ([lk.stats() for lk in self.links] if cfg.msg_rate is not None
                 else None)
        return SimResult(t_arr, pos, vel, acc, err, u_arr, self.kind, cfg,
                         h=h_arr, rho_hat=rho_arr, gains=gains_arr,
                         trust=trust_arr, ff_inj=ffinj_arr, link_stats=stats,
                         aeb=aeb_arr,
                         traffic=None if tf is None else {
                             "humans": list(tf.cfg.humans),
                             "cut_ins": tf.tracks(t_arr, cut_rec)})


# ------------------------------------------------------------ scenario I/O
from cacc.leader import make_leader_profile  # noqa: E402,F401  (re-export)


def __getattr__(name: str):
    """Backward compatibility: scenario I/O moved to :mod:`cacc.scenario`."""
    if name in ("load_scenario", "scenario_from_dict", "SCENARIO_BLOCKS"):
        from cacc import scenario

        return getattr(scenario, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
