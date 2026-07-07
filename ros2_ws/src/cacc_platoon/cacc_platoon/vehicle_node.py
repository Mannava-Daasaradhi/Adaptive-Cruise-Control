"""Follower vehicle node: own longitudinal dynamics + shared controller.

Each follower integrates ONLY its own state (RK4 at ``sim_rate_hz``), using
the latest received predecessor state (onboard radar surrogate — direct
topic, no channel) and the latest V2V beacon delivered by the channel node
(delayed / lossy / noisy). Dynamics and control laws are imported from the
same ``cacc`` package as the offline simulator — single source of truth.
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from cacc.controllers import ControllerParams, make_controller
from cacc.vehicle import VehicleParams, vehicle_deriv

from cacc_platoon_msgs.msg import V2VBeacon, VehicleState


class VehicleNode(Node):
    def __init__(self) -> None:
        super().__init__("vehicle")
        p = self.declare_parameters("", [
            ("index", 1),
            ("controller", "cthp"),
            ("sim_rate_hz", 100.0),
            ("beacon_rate_hz", 25.0),
            ("v0", 20.0),
            ("kp", 0.009), ("kd", 0.7), ("kv", 0.63), ("ka", 0.5),
            ("h", 0.95), ("r", 1.0),
            ("tau", 0.5), ("length", 4.0), ("u_min", -8.0), ("u_max", 3.0),
        ])
        g = {q.name: q.value for q in p}
        self.i = int(g["index"])
        self.veh = VehicleParams(tau=g["tau"], length=g["length"],
                                 u_min=g["u_min"], u_max=g["u_max"])
        self.cprm = ControllerParams(kp=g["kp"], kd=g["kd"], kv=g["kv"],
                                     ka=g["ka"], h=g["h"], r=g["r"])
        self.ctrl = make_controller(str(g["controller"]), self.cprm)
        self.dt = 1.0 / g["sim_rate_hz"]
        self.t = 0.0
        # initial condition: exact desired spacing at cruise speed v0
        gap = self.cprm.r + self.cprm.h * g["v0"] + self.veh.length
        self.x = np.array([-self.i * gap, float(g["v0"]), 0.0])  # [p, v, a]
        self.xc = np.zeros(self.ctrl.n_states)
        self.pred_state: VehicleState | None = None
        self._anchor_hold = 2.0  # [s] pin exact spacing until streams are live
        self.v2v_ff = 0.0
        pre = self.i - 1
        self.create_subscription(VehicleState, f"/platoon/v{pre}/state",
                                 self._on_pred_state, 10)
        if self.ctrl.uses_v2v:
            self.create_subscription(V2VBeacon, f"/platoon/v{self.i}/v2v",
                                     self._on_beacon, 10)
        self.pub_state = self.create_publisher(VehicleState,
                                               f"/platoon/v{self.i}/state", 10)
        self.pub_beacon = self.create_publisher(V2VBeacon,
                                                f"/platoon/v{self.i}/beacon", 10)
        self.create_timer(self.dt, self.step)
        self._beacon_period = 1.0 / g["beacon_rate_hz"]
        self._next_beacon = 0.0
        self._last_now = None  # wall-clock timebase (see D-014)
        self._u = 0.0
        self._e = self._edot = self._uff = self._dv = 0.0
        self.get_logger().info(
            f"vehicle {self.i}: {g['controller']} h={self.cprm.h}s tau={self.veh.tau}s")

    # ------------------------------------------------------------ callbacks
    def _on_pred_state(self, msg: VehicleState) -> None:
        self.pred_state = msg

    def _on_beacon(self, msg: V2VBeacon) -> None:
        self.v2v_ff = msg.accel if self.ctrl.ff_signal == "a" else msg.u_cmd

    # ----------------------------------------------------------------- step
    def step(self) -> None:
        # integrate over the *measured* elapsed wall time so an OS stall
        # advances the state instead of freezing it — otherwise own state
        # (tick time) and the predecessor extrapolation (wall time) diverge
        # by v * stall, injecting metre-scale error spikes (D-014)
        now = self.get_clock().now()
        dt = self.dt if self._last_now is None \
            else (now - self._last_now).nanoseconds * 1e-9
        # stalls up to ~1.5 s occur; every node must advance its FULL
        # elapsed time with the same clamp, or the platoon's frames diverge
        dt = min(max(dt, 1e-6), 5.0)
        self._last_now = now

        # 1) advance own [p, v, a] (+ controller state) by RK4 with the
        #    input computed at the previous sample (ZOH actuation);
        #    substep so a long stall stays inside RK4's stability region
        #    (single-step limit for the 1/tau pole is ~1.4 s)
        u = self._u
        e0, edot0, uff0, dv0 = self._e, self._edot, self._uff, self._dv

        def f(x, xc):
            dx = vehicle_deriv(x, u, self.veh)
            dxc = self.ctrl.deriv(xc, e0, edot0, uff0, dv0)
            return dx, dxc

        n_sub = max(1, int(np.ceil(dt / self.dt)))
        h = dt / n_sub
        for _ in range(n_sub):
            k1, k1c = f(self.x, self.xc)
            k2, k2c = f(self.x + 0.5 * h * k1, self.xc + 0.5 * h * k1c)
            k3, k3c = f(self.x + 0.5 * h * k2, self.xc + 0.5 * h * k2c)
            k4, k4c = f(self.x + h * k3, self.xc + h * k3c)
            self.x = self.x + h / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
            if self.ctrl.n_states:
                self.xc = self.xc + h / 6.0 * (k1c + 2 * k2c + 2 * k3c + k4c)
        self.t += dt

        # 2) fresh measurement: own state and the extrapolated predecessor
        #    now both refer to wall-time `now`
        pre = self.pred_state
        if pre is None:  # nothing heard yet: hold cruise
            e = e_dot = dv = 0.0
        else:
            c = self.cprm
            # radar surrogate: a real sensor sees the *current* gap, so
            # extrapolate the last state message by its age (shared host
            # clock); the first messages can be 10-100 ms stale (DDS
            # discovery), which at v0 = 20 m/s is a multi-metre error
            age = (now - Time.from_msg(pre.stamp)).nanoseconds * 1e-9
            # clip matches the dt clamp: both sides of the gap measurement
            # must bridge the same stall, or e spikes by v*(mismatch);
            # extrapolation error stays ~jerk*age^2/2 (<1 m even at 5 s)
            age = min(max(age, 0.0), 5.0)
            p_pre = pre.position + age * (pre.velocity + 0.5 * age * pre.acceleration)
            v_pre = pre.velocity + age * pre.acceleration
            if self.t < self._anchor_hold:
                # spawn/discovery order is arbitrary; pin exact spacing until
                # every stream is live so no startup offset gets frozen into
                # the slow (kp-scale, ~70 s) spacing-error dynamics
                self.x[1] = v_pre
                self.x[0] = p_pre - self.veh.length - (c.r + c.h * self.x[1])
            e = (p_pre - self.x[0] - self.veh.length) - (c.r + c.h * self.x[1])
            e_dot = v_pre - self.x[1] - c.h * self.x[2]
            dv = v_pre - self.x[1]

        # 3) input for the next interval
        u_ff = self.v2v_ff if self.ctrl.uses_v2v else 0.0
        self._u = self.veh.clamp(self.ctrl.output(self.xc, e, e_dot, u_ff, dv))
        self._e, self._edot, self._uff, self._dv = e, e_dot, u_ff, dv

        msg = VehicleState(index=self.i, t=self.t, position=float(self.x[0]),
                           velocity=float(self.x[1]),
                           acceleration=float(self.x[2]), u_cmd=float(self._u),
                           spacing_error=float(e))
        msg.stamp = now.to_msg()
        self.pub_state.publish(msg)
        if self.t >= self._next_beacon:
            b = V2VBeacon(sender=self.i, t=self.t,
                          accel=float(self.x[2]), u_cmd=float(self._u))
            b.stamp = msg.stamp
            self.pub_beacon.publish(b)
            self._next_beacon += self._beacon_period


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VehicleNode()
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
