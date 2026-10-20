"""Electric-vehicle powertrain and road model (scenario ``ev:`` block).

Two effects, one on the dynamics and one on the bookkeeping:

* **Limits.** With an ``ev`` block the commanded acceleration is bounded by
  what the motor and the tyres can deliver at the current speed::

      a_max(v) = min(u_max, mu*g, P_max / (m * v) - F_res(v) / m)
      a_min    = max(u_min, -mu*g)

  ``F_res`` is aerodynamic drag plus rolling resistance. Braking is limited
  by grip only: friction brakes blend in wherever regeneration runs out, so
  the regen limit costs energy, never stopping distance.

* **Energy.** Battery power is computed after the run from each car's
  speed and realized acceleration: traction draws ``P_wheel / eta_drive``,
  braking recovers ``min(|P_wheel|, P_regen) * eta_regen`` and the rest is
  lost in the friction brakes. :func:`energy_report` integrates it into
  kWh/100 km, regen recovery, end state of charge and the energy the
  slipstream of the car ahead saved, per follower.

Defaults describe a mid-size electric sedan (about 1.9 t, 150 kW).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

G = 9.81
RHO_AIR = 1.2  # air density [kg/m^3]
#: speed floor of the power limit [m/s]; below it grip and u_max bind anyway
V_FLOOR = 1.0
_trapz = getattr(np, "trapezoid", None) or np.trapz  # numpy < 2 has trapz only

#: tyre-road friction coefficients for the named road surfaces
SURFACES = {"dry": 0.9, "wet": 0.5, "snow": 0.3, "ice": 0.1}


@dataclass(frozen=True)
class EVParams:
    """Powertrain, body and road parameters of one electric vehicle.

    Attributes
    ----------
    mass:
        Vehicle mass with occupants and load [kg].
    power_kw:
        Peak traction power at the wheels [kW] (derate it for a cold or
        nearly empty battery).
    regen_kw:
        Peak regenerative braking power [kW].
    mu:
        Tyre-road friction coefficient (dry 0.9, wet 0.5, snow 0.3, ice 0.1).
    drag_area:
        Drag coefficient times frontal area, Cd*A [m^2].
    crr:
        Rolling-resistance coefficient.
    eta_drive, eta_regen:
        Battery-to-wheel and wheel-to-battery efficiencies.
    aux_kw:
        Constant auxiliary load (HVAC, computers) [kW].
    battery_kwh, soc0:
        Usable battery capacity [kWh] and initial state of charge (0..1).
    draft_gain, draft_len:
        Slipstream model of the energy bookkeeping: a follower at gap ``d``
        sees its drag scaled by ``1 - draft_gain * exp(-d / draft_len)``.
        A coarse fit to the 20-40 % drag reductions reported for gaps under
        one car length; set ``draft_gain = 0`` to ignore drafting.
    """

    mass: float = 1900.0
    power_kw: float = 150.0
    regen_kw: float = 60.0
    mu: float = 0.9
    drag_area: float = 0.6
    crr: float = 0.010
    eta_drive: float = 0.90
    eta_regen: float = 0.70
    aux_kw: float = 0.5
    battery_kwh: float = 75.0
    soc0: float = 0.8
    draft_gain: float = 0.3
    draft_len: float = 10.0

    def __post_init__(self):
        for name in ("mass", "power_kw", "mu", "battery_kwh"):
            if getattr(self, name) <= 0.0:
                raise ValueError(f"ev {name} must be > 0")
        for name in ("regen_kw", "drag_area", "crr", "aux_kw"):
            if getattr(self, name) < 0.0:
                raise ValueError(f"ev {name} must be >= 0")
        for name in ("eta_drive", "eta_regen", "soc0"):
            if not 0.0 < getattr(self, name) <= 1.0:
                raise ValueError(f"ev {name} must be in (0, 1]")

    def resistance(self, v, drag_scale=1.0):
        """Drag + rolling resistance force [N] at speed ``v`` [m/s]."""
        v = np.abs(v)
        return (0.5 * RHO_AIR * self.drag_area * drag_scale * v * v
                + self.crr * self.mass * G)

    def draft_scale(self, gap):
        """Drag multiplier behind a car ``gap`` metres ahead (1 = no draft)."""
        return 1.0 - self.draft_gain * np.exp(-np.maximum(gap, 0.0) / self.draft_len)

    def accel_bounds(self, v: float, u_min: float, u_max: float) -> tuple[float, float]:
        """Achievable ``(a_min, a_max)`` at speed ``v`` [m/s^2]."""
        grip = self.mu * G
        vv = max(abs(v), V_FLOOR)
        traction = (self.power_kw * 1e3 / vv - float(self.resistance(vv))) / self.mass
        hi = min(u_max, grip, traction)
        lo = max(u_min, -grip)
        return lo, max(hi, lo)


def wheel_power(ev: EVParams, v, a, drag_scale=1.0):
    """Power at the wheels [W] (negative while braking)."""
    return (ev.mass * a + ev.resistance(v, drag_scale)) * np.maximum(v, 0.0)


def battery_power(ev: EVParams, v, a, drag_scale=1.0):
    """Battery power [W] (positive = discharge) for speed / accel series."""
    p_wheel = wheel_power(ev, v, a, drag_scale)
    regen = np.minimum(-p_wheel, ev.regen_kw * 1e3) * ev.eta_regen
    return np.where(p_wheel >= 0.0, p_wheel / ev.eta_drive, -regen) + ev.aux_kw * 1e3


def energy_report(res) -> dict:
    """Per-follower energy figures of a :class:`cacc.platoon.SimResult`.

    A run without an ``ev`` block is scored with the default :class:`EVParams`
    so energy criteria work on any scenario.
    """
    ev = res.config.vehicle.ev or EVParams()
    t = res.t
    v, a = res.vel[:, 1:], res.acc[:, 1:]
    gaps = -np.diff(res.pos, axis=1) - res.config.vehicle.length
    draft = ev.draft_scale(gaps)
    p_batt = battery_power(ev, v, a, draft)
    e_batt = _trapz(p_batt, t, axis=0) / 3.6e6  # kWh
    dist_km = _trapz(np.maximum(v, 0.0), t, axis=0) / 1e3
    p_wheel = wheel_power(ev, v, a, draft)
    braking = _trapz(np.maximum(-p_wheel, 0.0), t, axis=0)  # J at wheel
    recovered = _trapz(np.minimum(np.maximum(-p_wheel, 0.0),
                                        ev.regen_kw * 1e3), t, axis=0)
    with np.errstate(invalid="ignore", divide="ignore"):
        per_100 = np.where(dist_km > 1e-3, 100.0 * e_batt / dist_km, np.nan)
        regen_share = np.where(braking > 1.0, recovered / braking, 1.0)
    soc_end = ev.soc0 - e_batt / ev.battery_kwh
    solo = _trapz(battery_power(ev, v, a), t, axis=0) / 3.6e6
    return {"kwh": e_batt, "kwh_solo": solo, "kwh_per_100km": per_100,
            "distance_km": dist_km,
            "regen_share": regen_share, "soc_end": soc_end}


def energy_per_100km(res) -> float:
    """Highest follower consumption [kWh/100 km]."""
    return float(np.nanmax(energy_report(res)["kwh_per_100km"]))


def regen_share(res) -> float:
    """Lowest follower share of braking energy sent back to the battery."""
    return float(energy_report(res)["regen_share"].min())
