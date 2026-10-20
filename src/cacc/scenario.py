"""Scenario files: YAML -> :class:`cacc.platoon.Scenario` (schema: R-11).

A scenario is the full description of one simulation case — platoon,
vehicle, controller gains, V2V channel, optional adaptation / attack / trust
blocks, leader maneuver and integration settings. Test plans (D-025) edit the
parsed mapping (overrides, matrix axes) and rebuild it with
:func:`scenario_from_dict`, so every scenario feature is available to a plan
without a second schema.
"""

from __future__ import annotations

import logging
from pathlib import Path

import yaml

from cacc.aeb import AEBConfig
from cacc.controllers import ControllerParams
from cacc.estimation import AdaptConfig
from cacc.ev import SURFACES, EVParams
from cacc.faults import parse as parse_faults
from cacc.network import LinkAttack
from cacc.platoon import PlatoonConfig, Scenario, make_leader_profile
from cacc.radar import RadarConfig
from cacc.traffic import TrafficConfig
from cacc.trust import TrustConfig
from cacc.vehicle import VehicleParams

log = logging.getLogger(__name__)


#: top-level blocks of the scenario YAML schema (docs/reference/R-11)
SCENARIO_BLOCKS = ("name", "description", "platoon", "vehicle", "ev", "controller",
                   "network", "adaptation", "attack", "trust", "radar",
                   "traffic", "aeb", "faults", "leader", "sim")


def load_scenario(path: str | Path) -> Scenario:
    """Load a YAML scenario file into a :class:`Scenario`."""
    path = Path(path)
    with open(path, encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    sc = scenario_from_dict(raw, default_name=path.stem)
    log.info("loaded scenario %r from %s", sc.name, path)
    return sc


def scenario_from_dict(raw: dict, default_name: str = "scenario") -> Scenario:
    """Build a :class:`Scenario` from a parsed scenario mapping.

    Unknown top-level blocks are rejected so a typo in a test-plan override
    (``netwrk.delay``) fails loudly instead of being silently ignored.
    """
    unknown = sorted(set(raw) - set(SCENARIO_BLOCKS))
    if unknown:
        raise ValueError(f"unknown scenario block(s) {unknown}; "
                         f"expected a subset of {list(SCENARIO_BLOCKS)}")
    net = dict(raw.get("network") or {})
    if net.get("rho_schedule") is not None:
        net["rho_schedule"] = tuple(tuple(x) for x in net["rho_schedule"])
    if net.get("qos_map") is not None:
        net["qos_map"] = tuple(tuple(x) for x in net["qos_map"])
    extra: dict = {}
    if "attack" in raw:  # malicious V2V injection (D-022)
        extra["attack"] = LinkAttack(**raw["attack"])
    if "trust" in raw:  # physics-consistency gate (D-022)
        extra["trust"] = TrustConfig(**raw["trust"])
    if "radar" in raw:  # modelled radar accel estimate (lag + noise)
        extra["radar"] = RadarConfig(**raw["radar"])
    if raw.get("traffic"):  # human drivers / cut-ins (mixed traffic)
        extra["traffic"] = TrafficConfig.from_dict(raw["traffic"])
    if raw.get("aeb"):  # emergency-braking safety layer
        extra["aeb"] = AEBConfig(**raw["aeb"])
    if raw.get("faults"):  # sensor / powertrain faults
        extra["faults"] = parse_faults(raw["faults"])
    veh = dict(raw.get("vehicle") or {})
    if raw.get("ev") is not None:  # electric powertrain + road grip
        ev = dict(raw["ev"])
        surface = ev.pop("surface", None)
        if surface is not None:
            if surface not in SURFACES:
                raise ValueError(f"unknown ev surface {surface!r}; use one of "
                                 f"{', '.join(SURFACES)}")
            ev.setdefault("mu", SURFACES[surface])
        veh["ev"] = EVParams(**ev)
    cfg = PlatoonConfig(
        vehicle=VehicleParams(**veh),
        control=ControllerParams(**raw.get("controller", {})),
        adapt=AdaptConfig(**raw.get("adaptation", {})),
        **raw.get("platoon", {}),
        **net,
        **extra,
        **raw.get("sim", {}),
    )
    leader = make_leader_profile(raw["leader"])
    return Scenario(name=raw.get("name", default_name), config=cfg,
                    leader=leader, description=raw.get("description", ""))
