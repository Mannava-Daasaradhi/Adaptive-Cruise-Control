"""Simulation data types: :class:`PlatoonConfig`, :class:`SimResult`,
:class:`Scenario` (re-exported by :mod:`cacc.platoon`)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np

from cacc.aeb import AEBConfig
from cacc.controllers import ControllerParams
from cacc.estimation import AdaptConfig
from cacc.faults import Fault
from cacc.network import LinkAttack
from cacc.radar import RadarConfig
from cacc.traffic import TrafficConfig
from cacc.trust import TrustConfig
from cacc.vehicle import VehicleParams


@dataclass(frozen=True)
class PlatoonConfig:
    """Full configuration of one platoon simulation."""

    n_followers: int = 5
    v0: float = 20.0  # initial platoon cruise speed [m/s]
    vehicle: VehicleParams = VehicleParams()
    control: ControllerParams = ControllerParams()
    delay: float = 0.1  # V2V latency theta [s] (minimum latency with jitter)
    loss_prob: float = 0.0
    msg_rate: float | None = None  # None = continuous link
    delay_jitter: float = 0.0  # mean extra latency per message [s] (sampled)
    loss_burst: float | None = None  # mean loss-burst length [msgs] (sampled)
    noise_rho: float | None = None  # multiplicative channel noise (Ma 2025)
    noise_gammas: tuple[float, ...] | None = None  # None = paper's 16-bit channel
    noise_rate: float = 100.0  # noise-factor hold rate [Hz]
    rho_schedule: tuple | None = None  # [(t, rho), ...] time-varying channel
    qos_map: tuple | None = None  # [(x0, x1, rho), ...] spatial channel map (D-021)
    adapt: AdaptConfig = AdaptConfig()  # QoS-adaptive outer loop (D-016)
    attack: LinkAttack | None = None  # malicious V2V injection (D-022)
    trust: TrustConfig = TrustConfig()  # physics-consistency gate (D-022)
    radar: RadarConfig = RadarConfig()  # radar accel estimate (ideal by default)
    traffic: TrafficConfig | None = None  # human drivers / cut-ins (mixed traffic)
    aeb: AEBConfig = AEBConfig()  # emergency-braking safety layer (cacc.aeb)
    faults: tuple[Fault, ...] = ()  # sensor / powertrain faults (cacc.faults)
    gap0: float | None = None  # initial bumper gap of car 1 to the leader [m]
    lead_v0: float | None = None  # leader's initial speed [m/s] (None = v0)
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
    link_stats: list | None = None  # per-link sent/delivered/max PIR (sampled)
    traffic: dict | None = None  # humans + recorded cut-in cars (cacc.traffic)
    aeb: np.ndarray | None = None  # (S, n) AEB level 0/1/2 (cacc.aeb)


@dataclass
class Scenario:
    """A named simulation case: configuration + leader maneuver."""

    name: str
    config: PlatoonConfig
    leader: Callable[[float], float]
    description: str = ""
