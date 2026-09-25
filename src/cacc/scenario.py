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

from cacc.controllers import ControllerParams
from cacc.estimation import AdaptConfig
from cacc.network import LinkAttack
from cacc.platoon import PlatoonConfig, Scenario, make_leader_profile
from cacc.trust import TrustConfig
from cacc.vehicle import VehicleParams

log = logging.getLogger(__name__)


#: top-level blocks of the scenario YAML schema (docs/reference/R-11)
SCENARIO_BLOCKS = ("name", "description", "platoon", "vehicle", "controller",
                   "network", "adaptation", "attack", "trust", "leader", "sim")


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
    cfg = PlatoonConfig(
        vehicle=VehicleParams(**raw.get("vehicle", {})),
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
