"""Crash-database scenarios: how real rear-end crashes start.

Two public sources define the cases:

* the **NHTSA pre-crash scenario typology** (Najm, Smith & Yanagisawa,
  *Pre-Crash Scenario Typology for Crash Avoidance Research*, DOT HS 810 767,
  2007), built from US police-reported crashes. Its rear-end scenarios are
  lead vehicle stopped (LVS), decelerating (LVD) and moving at a lower
  constant speed (LVM), and the lane-change crash "vehicle(s) changing
  lanes, same direction";
* the **Euro NCAP AEB Car-to-Car Rear** test protocol: a stationary target
  (CCRs), a slower target at 20 km/h (CCRm) and a target braking at -2 or
  -6 m/s^2 from 50 km/h with 12 m or 40 m headway (CCRb).

Each entry is a campaign maneuver (lead car, start and end time) plus the
scenario edits that set the scene: the lead car's own initial speed
(``platoon.lead_v0``), the first follower's starting gap
(``platoon.gap0``, from a 4 s time-to-collision at the start of a Euro NCAP
run) and any traffic. The platoon behind car 1 starts at its own
equilibrium, so the grid shows how the shock travels down the string.

The NHTSA "lead vehicle stopped" case is staged the way it happens on a
highway: a car in front of car 1 swerves out of the lane at ``REVEAL`` s
and uncovers a stopped car 3 s (TTC) ahead.
"""

from __future__ import annotations

KMH = 1 / 3.6
REVEAL = 6.0  # [s] when the car in front swerves out (LVS reveal)
V_HWY = 25.0  # [m/s] highway speed of the NHTSA cases (90 km/h)


def _ncap_stationary(kmh: float) -> dict:
    v = kmh * KMH
    return {"platoon.v0": v, "platoon.lead_v0": 0.0, "platoon.gap0": 4.0 * v}


def _ncap_moving(kmh: float) -> dict:
    v = kmh * KMH
    return {"platoon.v0": v, "platoon.lead_v0": 20 * KMH,
            "platoon.gap0": 4.0 * (v - 20 * KMH)}


def _brake_to_stop(decel: float, v: float, t0: float = 2.0) -> dict:
    return {"profile": "brake", "t_start": t0, "duration": v / -decel, "decel": decel}


_STILL = {"profile": "constant", "value": 0.0}
_V50 = 50 * KMH

#: key -> (label, leader block, t_start, t_final, scenario edits)
MANEUVERS: dict[str, tuple] = {
    "ncap-ccrs-50": ("Euro NCAP CCRs: stopped car, 50 km/h", _STILL, 0.0, 25.0,
                     _ncap_stationary(50)),
    "ncap-ccrs-80": ("Euro NCAP CCRs: stopped car, 80 km/h", _STILL, 0.0, 25.0,
                     _ncap_stationary(80)),
    "ncap-ccrm-80": ("Euro NCAP CCRm: 20 km/h car, 80 km/h", _STILL, 0.0, 30.0,
                     _ncap_moving(80)),
}
for _dec in (-2.0, -6.0):
    for _gap in (12.0, 40.0):
        MANEUVERS[f"ncap-ccrb{int(-_dec)}-{int(_gap)}"] = (
            f"Euro NCAP CCRb: brakes {_dec:g} m/s², {_gap:g} m gap",
            _brake_to_stop(_dec, _V50), 2.0, 30.0,
            {"platoon.v0": _V50, "platoon.gap0": _gap})
MANEUVERS.update({
    "nhtsa-lvs": ("NHTSA LVS: stopped car revealed by a lane change",
                  _STILL, REVEAL, 30.0,
                  {"platoon.v0": V_HWY, "platoon.lead_v0": 0.0,
                   "platoon.gap0": V_HWY * (REVEAL + 3.0),
                   "traffic": {"cut_ins": [{"t": 0.0, "ahead_of": 1, "gap": "eq",
                                            "dv": 0.0, "blind": True,
                                            "leave": REVEAL}]}}),
    "nhtsa-lvd": ("NHTSA LVD: lead car brakes −6 m/s² to a stop",
                  _brake_to_stop(-6.0, V_HWY, 5.0), 5.0, 35.0, {"platoon.v0": V_HWY}),
    "nhtsa-lvm": ("NHTSA LVM: catching a 36 km/h car", _STILL, 0.0, 35.0,
                  {"platoon.v0": V_HWY, "platoon.lead_v0": 10.0,
                   "platoon.gap0": 4.0 * (V_HWY - 10.0)}),
    "nhtsa-lane-change": ("NHTSA lane change: slower car cuts in ahead of car 1",
                          {"profile": "constant", "value": 0.0}, 8.0, 35.0,
                          {"platoon.v0": V_HWY,
                           "traffic": {"cut_ins": [{"t": 8.0, "ahead_of": 1,
                                                    "gap_frac": 0.4, "dv": -5.0,
                                                    "decel": -3.0,
                                                    "duration": 2.0}]}}),
})

SOURCES = {k: ("Euro NCAP AEB Car-to-Car protocol" if k.startswith("ncap")
               else "NHTSA pre-crash typology (DOT HS 810 767)") for k in MANEUVERS}
