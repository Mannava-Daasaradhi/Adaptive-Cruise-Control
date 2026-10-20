"""Drive a platoon in CARLA with any cacc controller.

    cacc carla scenarios/ev_highway.yaml -c cacc --town Town04 -o carla_run.csv
    cacc audit carla_run.csv          # the same audit as for a real drive log

The leader follows the scenario's maneuver; every follower runs the chosen
controller (built-in, plugin, Controller Lab design or FMU) on the gap,
speeds and accelerations measured in CARLA, and its acceleration command is
turned into throttle and brake by :func:`pedals`. V2V is ideal apart from
the scenario's delay. The run is logged in the generic drive-log format, so
CARLA runs go through ``cacc audit`` / ``cacc calibrate`` like field data.

The controller side (:class:`PlatoonDriver`, :func:`pedals`) is plain Python
and tested without CARLA; only :func:`run_carla` imports the ``carla``
client (``pip install carla`` matching the server version).
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np

from cacc.controllers import make_controller


@dataclass(frozen=True)
class PedalMap:
    """Inverse longitudinal model: acceleration command -> pedals.

    ``a_throttle`` / ``a_brake`` are the accelerations at full throttle and
    full brake; ``k_a`` closes a proportional loop on the measured
    acceleration so the car tracks the command despite the map error.
    """

    a_throttle: float = 3.5
    a_brake: float = 9.0
    k_a: float = 0.3
    coast: float = -0.3  # deceleration with both pedals released


def pedals(u: float, a_meas: float, m: PedalMap = PedalMap()) -> tuple[float, float]:
    """(throttle, brake) in [0, 1] for commanded acceleration ``u``."""
    cmd = u + m.k_a * (u - a_meas)
    if cmd >= m.coast:
        return float(np.clip((cmd - m.coast) / (m.a_throttle - m.coast), 0, 1)), 0.0
    return 0.0, float(np.clip((m.coast - cmd) / (m.a_brake + m.coast), 0, 1))


class PlatoonDriver:
    """Controller side of the co-simulation: measurements in, commands out."""

    def __init__(self, scenario, controller: str = "cacc",
                 options: dict | None = None, dt: float = 0.05):
        cfg = scenario.config
        self.cfg, self.dt, self.leader = cfg, float(dt), scenario.leader
        self.ctrls = [make_controller(controller, cfg.control, options or {})
                      for _ in range(cfg.n_followers)]
        self.xc = [np.zeros(c.n_states) for c in self.ctrls]
        self.raw = [bool(getattr(c, "raw_inputs", False)) for c in self.ctrls]
        lag = max(1, int(round(cfg.delay / self.dt)) + 1) if cfg.delay > 0 else 1
        self.v2v = [deque([0.0] * lag, maxlen=lag) for _ in self.ctrls]
        self.v_ref = cfg.v0
        self.u_prev = np.zeros(cfg.n_followers + 1)

    def spacing(self, i: int) -> float:
        """Initial bumper gap ahead of follower ``i`` (equilibrium)."""
        c, v0 = self.ctrls[i - 1], self.cfg.v0
        gap = getattr(c, "equilibrium_gap", None)
        return float(gap(v0)) if gap else self.cfg.control.r + self.cfg.control.h * v0

    def step(self, t: float, gaps, speeds, accels) -> np.ndarray:
        """Commanded accelerations [leader, followers...] at time ``t``.

        ``gaps[i-1]`` is the bumper gap ahead of follower ``i``; ``speeds``
        and ``accels`` include the leader in column 0.
        """
        cfg, c = self.cfg, self.cfg.control
        a0 = float(self.leader(t))
        self.v_ref = max(0.0, self.v_ref + a0 * self.dt)
        u = np.empty(cfg.n_followers + 1)
        u[0] = a0 + 0.5 * (self.v_ref - speeds[0])  # leader tracks its profile
        for i, ctrl in enumerate(self.ctrls, start=1):
            v_p, v, a = speeds[i - 1], speeds[i], accels[i]
            e = gaps[i - 1] - (c.r + c.h * v)
            e_dot, dv = v_p - v - c.h * a, v_p - v
            sent = accels[i - 1] if ctrl.ff_signal == "a" else self.u_prev[i - 1]
            self.v2v[i - 1].append(float(sent))
            u_ff = self.v2v[i - 1][0] if ctrl.uses_v2v else 0.0
            kw = dict(gap=gaps[i - 1], v=v, a=a, t=t) if self.raw[i - 1] else {}
            cmd = ctrl.output(self.xc[i - 1], e, e_dot, u_ff, dv, **kw)
            if ctrl.n_states:  # explicit Euler with 10 substeps of the frame
                h = self.dt / 10
                for _ in range(10):
                    self.xc[i - 1] = self.xc[i - 1] + h * np.asarray(
                        ctrl.deriv(self.xc[i - 1], e, e_dot, u_ff, dv, **kw))
            u[i] = cfg.vehicle.clamp(cmd, v)
        self.u_prev = u
        return u


def _straight_spawn(world, gaps, length: float, spawn_index: int = 0):
    """Transforms for the platoon on one lane: leader first, then each
    follower ``gap + length`` further back along the lane."""
    m = world.get_map()
    wp = m.get_waypoint(m.get_spawn_points()[spawn_index].location)
    out = [wp.transform]
    for g in gaps:
        prev = wp.previous(g + length)
        if not prev:
            raise RuntimeError("lane too short for the platoon; pick another town")
        wp = prev[0]
        out.append(wp.transform)
    for tr in out:
        tr.location.z += 0.5
    return out


def run_carla(scenario, controller: str = "cacc", options: dict | None = None,
              host: str = "localhost", port: int = 2000, town: str | None = "Town04",
              dt: float = 0.05, out: str | None = None,
              blueprint: str = "vehicle.tesla.model3", spawn_index: int = 0):
    """Run the scenario in a CARLA server; return the :class:`PlatoonLog`."""
    try:
        import carla
    except ImportError:
        raise ImportError("CARLA co-simulation needs the CARLA Python client "
                          "(pip install carla, same version as the server)") from None
    from cacc.fielddata import PlatoonLog, write_log

    client = carla.Client(host, port)
    client.set_timeout(30.0)
    world = client.load_world(town) if town else client.get_world()
    settings = world.get_settings()
    original = carla.WorldSettings(**{k: getattr(settings, k) for k in (
        "synchronous_mode", "fixed_delta_seconds", "no_rendering_mode")})
    settings.synchronous_mode, settings.fixed_delta_seconds = True, dt
    world.apply_settings(settings)
    drv = PlatoonDriver(scenario, controller, options, dt)
    cfg = scenario.config
    n = cfg.n_followers
    bp = world.get_blueprint_library().find(blueprint)
    cars = []
    try:
        gaps0 = [drv.spacing(i) for i in range(1, n + 1)]
        for tr in _straight_spawn(world, gaps0, cfg.vehicle.length, spawn_index):
            cars.append(world.spawn_actor(bp, tr))
        world.tick()
        for car in cars:
            fwd = car.get_transform().get_forward_vector()
            car.set_target_velocity(carla.Vector3D(fwd.x * cfg.v0, fwd.y * cfg.v0, 0))
        steps = int(round(cfg.t_final / dt))
        T, V, GAP = np.zeros(steps), np.zeros((steps, n + 1)), np.zeros((steps, n))
        prev_v = np.full(n + 1, cfg.v0)
        for k in range(steps):
            world.tick()
            t = k * dt
            loc = [c.get_location() for c in cars]
            fwd = [c.get_transform().get_forward_vector() for c in cars]
            v = np.array([c.get_velocity().x * f.x + c.get_velocity().y * f.y
                          for c, f in zip(cars, fwd, strict=True)])
            acc = (v - prev_v) / dt
            prev_v = v
            ext = [2 * c.bounding_box.extent.x for c in cars]
            gaps = np.array([loc[i - 1].distance(loc[i]) - 0.5 * (ext[i - 1] + ext[i])
                             for i in range(1, n + 1)])
            u = drv.step(t, gaps, v, acc)
            for car, ui, ai in zip(cars, u, acc, strict=True):
                th, br = pedals(ui, ai)
                car.apply_control(carla.VehicleControl(throttle=th, brake=br,
                                                       steer=_lane_keep(world, car)))
            T[k], V[k], GAP[k] = t, v, gaps
        log = PlatoonLog(T, V, GAP, [f"car{i}" for i in range(n + 1)],
                         source=f"CARLA {town or ''} · {controller}")
        if out:
            write_log(log, out)
        return log
    finally:
        for car in cars:
            car.destroy()
        world.apply_settings(original)


def _lane_keep(world, car, look: float = 8.0, gain: float = 0.8) -> float:
    """Pure-pursuit steering toward the lane centre ``look`` metres ahead."""
    tr = car.get_transform()
    wp = world.get_map().get_waypoint(tr.location).next(look)
    if not wp:
        return 0.0
    tgt = wp[0].transform.location
    f, d = tr.get_forward_vector(), tgt - tr.location
    cross = f.x * d.y - f.y * d.x
    return float(np.clip(gain * cross / look, -1.0, 1.0))
