"""Bridges to the tools customers already use.

* :mod:`cacc.adapters.fmu` — run a controller exported as an FMI 2.0
  co-simulation FMU (Simulink, Modelica, CarMaker, ...) inside every cacc
  test plan and stress campaign.
* :mod:`cacc.adapters.carla_bridge` — drive a platoon in the CARLA
  simulator with any cacc controller and log it for ``cacc audit``.
* :mod:`cacc.adapters.ros2` — ROS 2 parameter files for ``ros2_ws`` from a
  scenario and a controller (built-in, plugin or Controller Lab design).
"""
