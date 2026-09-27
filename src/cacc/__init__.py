"""cacc — string-stability and V2X-robustness test engine for longitudinal
vehicle controllers (ACC, CACC, platooning).

Bring a controller (built-in or a plugin, :mod:`cacc.plugins`), describe the
conditions in a test plan (:mod:`cacc.evaluate`), and get a verdict with
reproducible evidence (:mod:`cacc.reporting`); the black-box frequency sweep
(:mod:`cacc.stringstab`) measures string stability of controllers whose
transfer function is unknown. The physics core reproduces Ma, Pagilla &
Darbha, IEEE T-ITS 26(1), 2025 to its printed digits; classic foundation:
Ploeg et al., IEEE T-ITS 15(2), 2014.
"""

__version__ = "0.6.0"

from cacc.analysis import (
    cthp_gains_feasible,
    cthp_h_lb,
    cthp_optimal,
    expected_w,
    gamma,
    gamma_magnitude,
    hinf_norm,
    is_string_stable,
    min_stable_headway,
)
from cacc.certificate import (ChannelLaw, amp_at, chance_headway,
                              chebyshev_headway, certify_design, exceedance_prob,
                              mean_square_amp, meansquare_headway,
                              nominal_headway, worstcase_headway)
from cacc.controllers import ACC, CACC, CTHP, ControllerParams, make_controller
from cacc.estimation import (AdaptConfig, ChannelEstimator, GainScheduler,
                             HeadwayAdapter)
from cacc.logging_config import setup_logging
from cacc.network import MA2025_GAMMAS, LinkAttack, V2VLink
from cacc.qos_map import QoSMap
from cacc.platoon import (
    PlatoonConfig,
    PlatoonSim,
    Scenario,
    SimResult,
    make_leader_profile,
)
from cacc.plugins import LongitudinalController
from cacc.scenario import load_scenario, scenario_from_dict
from cacc.trust import TrustConfig, TrustGate
from cacc.vehicle import VehicleParams

__all__ = [
    "ACC",
    "AdaptConfig",
    "CACC",
    "CTHP",
    "ChannelEstimator",
    "ChannelLaw",
    "ControllerParams",
    "GainScheduler",
    "HeadwayAdapter",
    "LinkAttack",
    "LongitudinalController",
    "amp_at",
    "certify_design",
    "chance_headway",
    "chebyshev_headway",
    "exceedance_prob",
    "mean_square_amp",
    "meansquare_headway",
    "nominal_headway",
    "worstcase_headway",
    "MA2025_GAMMAS",
    "PlatoonConfig",
    "PlatoonSim",
    "QoSMap",
    "Scenario",
    "SimResult",
    "TrustConfig",
    "TrustGate",
    "V2VLink",
    "VehicleParams",
    "cthp_gains_feasible",
    "cthp_h_lb",
    "cthp_optimal",
    "expected_w",
    "gamma",
    "gamma_magnitude",
    "hinf_norm",
    "is_string_stable",
    "load_scenario",
    "make_controller",
    "make_leader_profile",
    "min_stable_headway",
    "scenario_from_dict",
    "setup_logging",
]
