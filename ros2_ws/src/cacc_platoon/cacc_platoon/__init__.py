"""cacc_platoon — distributed ROS 2 simulation of a CACC vehicle platoon.

Node graph (N followers):

    leader ──/platoon/v0/state──────────────► vehicle_1 ── ... ──► vehicle_N
       │                 (radar surrogate)        │
       └──/platoon/v0/beacon──► channel ──/platoon/v1/v2v──┘
                                  (delay + Bernoulli loss + multiplicative
                                   noise w(t), per directed link)
    recorder ◄── /platoon/v*/state (all)  →  CSV

Physics and control laws are shared with the offline simulator: the nodes
import :mod:`cacc.vehicle` and :mod:`cacc.controllers` (add the repo's
``src/`` to PYTHONPATH — see scripts/ros2_run_demo.sh).
"""
