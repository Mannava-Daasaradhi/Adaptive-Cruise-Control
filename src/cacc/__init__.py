"""cacc — string-stable Cooperative Adaptive Cruise Control platoon simulation.

Sem-5 control-systems project. Base paper: Ma, Pagilla, Darbha, "Selection of
Time Headway in Connected and Autonomous Vehicle Platoons Under Noisy V2V
Communication," IEEE Trans. Intelligent Transportation Systems 26(1), 2025.
Classic foundation: Ploeg et al., IEEE T-ITS 15(2), 2014.
"""

__version__ = "0.3.0"

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
    load_scenario,
    make_leader_profile,
)
from cacc.trust import TrustConfig, TrustGate
from cacc.vehicle import VehicleParams

__all__ = [
    "ACC",
    "AdaptConfig",
    "CACC",
    "CTHP",
    "ChannelEstimator",
    "ControllerParams",
    "GainScheduler",
    "HeadwayAdapter",
    "LinkAttack",
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
    "setup_logging",
]
