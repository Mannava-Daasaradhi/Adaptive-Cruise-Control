"""Mixed traffic: people driving in the line, and cars cutting in.

A platoon on a real road shares it with human drivers. Scenario block::

    traffic:
      humans: [3]               # followers driven by a person, not the controller
      idm: {T: 1.4, s0: 2.0, a_max: 1.5, b: 2.0, reaction: 0.8}
      cut_ins:
        - {t: 10.0, ahead_of: 3, gap_frac: 0.5, dv: -3.0,
           decel: -3.0, duration: 2.0, leave: null}

Physics:

* **Human drivers** use the Intelligent Driver Model (Treiber, Hennecke &
  Helbing 2000) and act ``reaction`` seconds late (a dead time on the
  command); the car itself is the same (powertrain lag, grip and power
  limits). A human starts at its own IDM equilibrium gap, not the
  controller's.
* **Nobody broadcasts a human's or a stranger's acceleration**: the car
  behind a human, or behind a car that cut in, receives no V2V, so its
  feedforward is 0. A CACC then runs as an ACC on the short CACC headway,
  the realistic failure this block is there to expose.
* **A cut-in** car changes into the lane ahead of follower ``ahead_of`` at
  time ``t``, ``gap_frac`` of the way into the free space, at follower
  speed ``+ dv`` (negative = slower, the dangerous case). It brakes at
  ``decel`` for ``duration`` s, otherwise it follows its own leader by IDM
  (so it never drives through it); it is limited by road grip and stops at
  standstill. It leaves the lane at ``leave`` (``null`` = stays). The radar
  of ``ahead_of`` switches to it the moment it starts the lane change
  (``LANE_CHANGE`` s, drawn by the 3-D view). ``gap`` places it a fixed
  bumper gap ahead instead (``"eq"`` = the controller's own desired gap,
  so the follower starts in equilibrium behind it); ``blind`` cars ignore
  traffic and just hold speed (plus ``decel``), the car that swerves out of
  the lane and uncovers a stopped one.

:func:`front_tracks` gives every follower's gap and closing speed to
whatever is really in front of it; the safety metrics use it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from cacc.ev import SURFACES, G
from cacc.radar import DelayLine

LANE_CHANGE = 2.0  # [s] lateral move of a cut-in car (drawing only)
LANE_WIDTH = 3.5  # [m]


@dataclass(frozen=True)
class IDMParams:
    T: float = 1.4  # desired time gap [s]
    s0: float = 2.0  # standstill gap [m]
    a_max: float = 1.5  # comfortable acceleration [m/s^2]
    b: float = 2.0  # comfortable deceleration [m/s^2]
    delta: float = 4.0
    v_des: float | None = None  # desired speed [m/s]; None = 1.3 x cruise
    reaction: float = 0.8  # perception-reaction dead time [s]

    def __post_init__(self):
        if min(self.T, self.a_max, self.b) <= 0 or self.s0 < 0 or self.reaction < 0:
            raise ValueError("idm: T, a_max, b must be > 0; s0, reaction >= 0")

    def accel(self, gap: float, v: float, v_front: float, v_des: float,
              a_front: float = 0.0) -> float:
        """IDM + constant-acceleration heuristic (Treiber & Kesting, *Traffic
        Flow Dynamics* 2013, sec. 11.3.7): plain IDM brakes at full grip
        when a car pulls in close at the same speed; a driver does not,
        because the car ahead is not braking. CAH bounds that overreaction."""
        s = max(gap, 0.1)
        s_star = self.s0 + max(0.0, v * self.T + v * (v - v_front)
                               / (2.0 * math.sqrt(self.a_max * self.b)))
        a_idm = self.a_max * (1.0 - (max(v, 0.0) / v_des) ** self.delta
                              - (s_star / s) ** 2)
        al = min(a_front, self.a_max)
        den = v_front**2 - 2.0 * s * al
        if v_front * (v_front - v) <= -2.0 * s * al and den > 1e-9:
            a_cah = v * v * al / den
        else:  # (also the limit of a stopped, non-braking car ahead)
            a_cah = al - max(v - v_front, 0.0) ** 2 / (2.0 * s)
        if a_idm >= a_cah:
            return a_idm
        c = 0.99
        return (1 - c) * a_idm + c * (a_cah + self.b * math.tanh((a_idm - a_cah)
                                                                / self.b))

    def eq_gap(self, v: float, v_des: float) -> float:
        """Bumper gap at which the IDM cruises steadily at ``v``."""
        x = 1.0 - (v / v_des) ** self.delta
        return (self.s0 + v * self.T) / math.sqrt(max(x, 1e-6))


@dataclass(frozen=True)
class CutIn:
    t: float
    ahead_of: int  # follower number (1-based) that gets the new car in front
    gap_frac: float = 0.5
    dv: float = -3.0
    decel: float = 0.0
    duration: float = 0.0
    leave: float | None = None
    gap: float | str | None = None  # bumper gap [m] or "eq"; None = gap_frac
    blind: bool = False  # holds speed, does not follow its own leader

    def __post_init__(self):
        if self.gap is not None and self.gap != "eq" and (
                isinstance(self.gap, str) or self.gap <= 0):
            raise ValueError('cut_in gap must be > 0, "eq" or null')
        if not 0.05 <= self.gap_frac <= 0.95:
            raise ValueError("cut_in gap_frac must be in [0.05, 0.95]")
        if self.duration < 0 or self.decel > 0:
            raise ValueError("cut_in decel must be <= 0 and duration >= 0")
        if self.leave is not None and self.leave <= self.t:
            raise ValueError("cut_in leave must be after t")


@dataclass(frozen=True)
class TrafficConfig:
    humans: tuple[int, ...] = ()
    idm: IDMParams = IDMParams()
    cut_ins: tuple[CutIn, ...] = field(default_factory=tuple)

    @classmethod
    def from_dict(cls, raw: dict) -> "TrafficConfig":
        extra = set(raw) - {"humans", "idm", "cut_ins"}
        if extra:
            raise ValueError(f"unknown traffic key(s) {sorted(extra)}; use "
                             "humans, idm, cut_ins")
        return cls(tuple(int(i) for i in raw.get("humans") or ()),
                   IDMParams(**(raw.get("idm") or {})),
                   tuple(CutIn(**c) for c in raw.get("cut_ins") or ()))


class _Car:
    """A cut-in car: point mass integrated once per accepted step."""

    def __init__(self, spec: CutIn):
        self.spec = spec
        self.on = False
        self.done = False
        self.p = self.v = self.a = 0.0
        self.t0 = 0.0
        self.t_on = math.nan

    def at(self, t: float) -> tuple[float, float, float]:
        d = t - self.t0
        return self.p + self.v * d + 0.5 * self.a * d * d, self.v + self.a * d, self.a


class Traffic:
    """What the simulator asks while integrating a mixed-traffic platoon."""

    def __init__(self, cfg: TrafficConfig, n: int, v0: float, length: float,
                 mu: float, dt: float):
        for i in cfg.humans:
            if not 1 <= i <= n:
                raise ValueError(f"traffic humans: follower {i} not in 1..{n}")
        for c in cfg.cut_ins:
            if not 1 <= c.ahead_of <= n:
                raise ValueError(f"cut_in ahead_of {c.ahead_of} not in 1..{n}")
        idm = cfg.idm
        if 0.0 < idm.reaction < dt:
            raise ValueError("idm reaction must be 0 or >= dt")
        self.cfg, self.n, self.L, self.dt = cfg, n, length, dt
        self.v_des = idm.v_des or max(1.3 * v0, v0 + 3.0)
        self.a_floor = -mu * G
        self.human = [i + 1 in cfg.humans for i in range(n)]
        self.lines = [DelayLine(idm.reaction) if h and idm.reaction > 0 else None
                      for h in self.human]
        self.cmd = np.zeros(n)  # fresh IDM command per human (stage k1)
        self.cars = [_Car(c) for c in cfg.cut_ins]
        self.policy = (2.5, 0.7)  # (r, h) of the controller, for gap="eq"

    @classmethod
    def build(cls, config) -> "Traffic | None":
        tc = getattr(config, "traffic", None)
        if tc is None or (not tc.humans and not tc.cut_ins):
            return None
        ev = config.vehicle.ev
        mu = ev.mu if ev is not None else SURFACES["dry"]
        tf = cls(tc, config.n_followers, config.v0, config.vehicle.length,
                 mu, config.dt)
        tf.policy = (config.control.r, config.control.h)
        return tf

    # ---------------------------------------------------------- initial state
    def gap0(self, i: int, v0: float, pitch_gap: float) -> float:
        """Initial bumper gap ahead of follower ``i`` (0-based)."""
        return (self.cfg.idm.eq_gap(v0, self.v_des) if self.human[i]
                else pitch_gap)

    # -------------------------------------------------------------- dynamics
    def front(self, i: int, t: float, p, v, a, pred_human: bool):
        """What follower ``i`` (0-based) has in front at ``t``: ``(p, v, a,
        connected)``; ``connected`` is False when the V2V source is not the
        car ahead (a human, or a cut-in car)."""
        for car in self.cars:
            if car.on and car.spec.ahead_of == i + 1:
                cp, cv, ca = car.at(t)
                return cp, cv, ca, False
        return p, v, a, not pred_human

    def human_u(self, i: int, t: float, gap: float, v: float, v_front: float,
                a_front: float = 0.0) -> float:
        """Human command (IDM, acting ``reaction`` s late)."""
        self.cmd[i] = self.cfg.idm.accel(gap, v, v_front, self.v_des, a_front)
        line = self.lines[i]
        return self.cmd[i] if line is None else line.read(t)

    # ----------------------------------------------------- per accepted step
    def step(self, t: float, pos: np.ndarray, vel: np.ndarray, acc: np.ndarray) -> None:
        """Record human commands and move the cut-in cars to ``t + dt``.

        ``pos/vel/acc`` hold the leader at 0 and followers at 1..n."""
        for i, line in enumerate(self.lines):
            if line is not None:
                line.record(t, self.cmd[i])
        for car in self.cars:
            s = car.spec
            j = s.ahead_of  # follower column; its predecessor is j - 1
            if car.done:
                continue
            if not car.on:
                if t + 1e-9 < s.t:
                    continue
                free = pos[j - 1] - pos[j] - 2.0 * self.L
                car.on, car.t_on = True, t
                if s.gap is None:
                    car.p = pos[j] + s.gap_frac * max(free, 0.0) + self.L
                else:
                    r, h = self.policy
                    car.p = pos[j] + self.L + (r + h * vel[j] if s.gap == "eq"
                                               else s.gap)
                car.v, car.a, car.t0 = max(0.0, vel[j] + s.dv), 0.0, t
            elif s.leave is not None and t >= s.leave:
                car.on, car.done = False, True
                continue
            p, v, _ = car.at(t)
            gap = pos[j - 1] - p - self.L
            a = (0.0 if s.blind else
                 self.cfg.idm.accel(gap, v, vel[j - 1], self.v_des, acc[j - 1]))
            if t < s.t + s.duration:
                a = min(a, s.decel)
            a = max(a, self.a_floor)
            if v <= 0.0 and a < 0.0:
                v, a = 0.0, 0.0
            car.p, car.v, car.a, car.t0 = p, v, a, t

    def tracks(self, t_arr: np.ndarray, rec: list) -> list[dict]:
        """Recorded cut-in cars for :class:`SimResult` (NaN when absent)."""
        out = []
        for car, r in zip(self.cars, rec, strict=True):
            on = np.isfinite(r[:, 0])
            lat = np.full(t_arr.size, np.nan)
            if on.any():
                dt_on = t_arr - car.t_on
                lat[on] = LANE_WIDTH * (1.0 - np.clip(dt_on[on] / LANE_CHANGE, 0, 1))
            out.append({"ahead_of": car.spec.ahead_of, "pos": r[:, 0],
                        "vel": r[:, 1], "acc": r[:, 2], "lateral": lat})
        return out


def snapshot(traffic: Traffic | None, t: float) -> list:
    """One row per cut-in car at accepted time ``t`` (NaN = absent)."""
    if traffic is None:
        return []
    return [car.at(t) if car.on else (math.nan,) * 3 for car in traffic.cars]


def front_tracks(res) -> tuple[np.ndarray, np.ndarray]:
    """Gaps (S, m) and closing speeds (S, m) of every car to what is really
    ahead of it: followers to their predecessor or a cut-in car, plus each
    cut-in car to its own leader (m = n + number of cut-ins)."""
    L = res.config.vehicle.length
    gaps = np.diff(-res.pos, axis=1) - L
    closing = res.vel[:, 1:] - res.vel[:, :-1]
    tr = getattr(res, "traffic", None) or {}
    extra_g, extra_c = [], []
    for car in tr.get("cut_ins", []):
        j = car["ahead_of"]
        on = np.isfinite(car["pos"])
        gaps[on, j - 1] = car["pos"][on] - L - res.pos[on, j]
        closing[on, j - 1] = res.vel[on, j] - car["vel"][on]
        g = np.full(res.t.size, np.inf)
        c = np.zeros(res.t.size)
        g[on] = res.pos[on, j - 1] - L - car["pos"][on]
        c[on] = car["vel"][on] - res.vel[on, j - 1]
        extra_g.append(g)
        extra_c.append(c)
    if extra_g:
        gaps = np.column_stack([gaps, *extra_g])
        closing = np.column_stack([closing, *extra_c])
    return gaps, closing
