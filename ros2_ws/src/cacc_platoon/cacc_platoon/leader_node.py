"""Leader vehicle node: prescribed acceleration profile, no controller.

Publishes its state (radar surrogate for vehicle 1) and its V2V beacon.
Profiles mirror :func:`cacc.platoon.make_leader_profile`:

* ``sine``  — a0 = amplitude * sin(2*pi*freq_hz*(t - t_start)) for
  ``duration`` seconds from ``t_start`` (Ma et al. 2025 eq. (46) with
  amplitude 0.5, omega 0.1 rad/s, duration 20*pi)
* ``brake`` — constant ``decel`` for ``duration`` seconds from ``t_start``
* ``constant`` — fixed ``value``
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node

from cacc_platoon_msgs.msg import V2VBeacon, VehicleState


class LeaderNode(Node):
    def __init__(self) -> None:
        super().__init__("leader")
        p = self.declare_parameters("", [
            ("sim_rate_hz", 100.0),
            ("beacon_rate_hz", 25.0),
            ("v0", 20.0),
            ("profile", "sine"),
            ("t_start", 10.0),
            ("amplitude", 0.5),
            ("freq_hz", 0.1 / (2.0 * math.pi)),  # omega = 0.1 rad/s
            ("duration", 20.0 * math.pi),
            ("decel", -4.0),
            ("value", 0.0),
        ])
        g = {q.name: q.value for q in p}
        self.g = g
        self.dt = 1.0 / g["sim_rate_hz"]
        self.t = 0.0
        self._last_now = None  # wall-clock timebase (see D-014)
        self.pos = 0.0
        self.vel = float(g["v0"])
        self.pub_state = self.create_publisher(VehicleState, "/platoon/v0/state", 10)
        self.pub_beacon = self.create_publisher(V2VBeacon, "/platoon/v0/beacon", 10)
        self.create_timer(self.dt, self.step)
        self._beacon_period = 1.0 / g["beacon_rate_hz"]
        self._next_beacon = 0.0
        self.get_logger().info(
            f"leader: profile={g['profile']} v0={g['v0']} sim_rate={g['sim_rate_hz']} Hz")

    def accel(self, t: float) -> float:
        g = self.g
        kind = str(g["profile"]).lower()
        if kind == "sine":
            t0, t1 = g["t_start"], g["t_start"] + g["duration"]
            if t0 <= t < t1:
                return g["amplitude"] * math.sin(2.0 * math.pi * g["freq_hz"] * (t - t0))
            return 0.0
        if kind == "brake":
            t0 = g["t_start"]
            return g["decel"] if t0 <= t < t0 + g["duration"] else 0.0
        return float(g["value"])

    def step(self) -> None:
        # integrate over the *measured* elapsed wall time, not the nominal
        # tick: OS stalls then advance the state instead of freezing it,
        # so the published state is always current at its stamp (D-014)
        now = self.get_clock().now()
        dt = self.dt if self._last_now is None \
            else (now - self._last_now).nanoseconds * 1e-9
        # same clamp as vehicle_node: every node must bridge a stall fully
        dt = min(max(dt, 1e-6), 5.0)
        self._last_now = now
        # exact integration of piecewise-smooth profile over the step (midpoint)
        a_mid = self.accel(self.t + 0.5 * dt)
        self.pos += self.vel * dt + 0.5 * a_mid * dt**2
        self.vel += a_mid * dt
        self.t += dt
        a = self.accel(self.t)

        msg = VehicleState(index=0, t=self.t, position=self.pos,
                           velocity=self.vel, acceleration=a, u_cmd=a,
                           spacing_error=0.0)
        msg.stamp = now.to_msg()
        self.pub_state.publish(msg)
        if self.t >= self._next_beacon:
            b = V2VBeacon(sender=0, t=self.t, accel=a, u_cmd=a)
            b.stamp = msg.stamp
            self.pub_beacon.publish(b)
            self._next_beacon += self._beacon_period


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LeaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:  # context may already be torn down by the SIGINT handler
            node.destroy_node()
        except Exception:
            pass


if __name__ == "__main__":
    main()
