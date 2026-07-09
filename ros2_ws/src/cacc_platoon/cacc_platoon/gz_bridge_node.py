"""Kinematic bridge: drive Gazebo car poses from the CACC platoon state.

Subscribes to every ``/platoon/v{i}/state`` and writes each car's pose into
gz-sim through the world's ``set_pose`` service (``gz.msgs.Pose`` ->
``gz.msgs.Boolean``, provided by the ``UserCommands`` system). Cars are placed
in the **platoon (leader) frame** ::

    x_i = (position_i - position_leader) - length/2

so the leader sits near the origin and the followers trail behind it; what you
see move is exactly the inter-vehicle **spacing** breathing under the leader's
manoeuvre — the string-stability phenomenon — while the world scrolls past
only if ``moving_road`` is on. The CACC vehicle nodes remain the sole physics
authority; this node never integrates dynamics, it only renders them.

Requires the gz-sim Harmonic Python bindings (``python3-gz-transport13`` /
``python3-gz-msgs10``). If they are missing the node logs how to install them
and idles, so a launch that includes it still comes up.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from cacc_platoon_msgs.msg import VehicleState

from cacc_platoon.gz_assets import VEH_LEN, WORLD

try:  # gz-sim Harmonic transport + messages
    from gz.transport13 import Node as GzNode
    from gz.msgs10.pose_pb2 import Pose as GzPose
    from gz.msgs10.boolean_pb2 import Boolean as GzBool
    _GZ_IMPORT_ERR = None
except Exception as exc:  # pragma: no cover - depends on system gz install
    GzNode = GzPose = GzBool = None
    _GZ_IMPORT_ERR = exc


class GzBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("gz_bridge")
        p = self.declare_parameters("", [
            ("n_followers", 6),
            ("world", WORLD),
            ("viz_rate_hz", 50.0),
            ("length", VEH_LEN),
            ("lane_y", 0.0),
            ("timeout_ms", 30),
            ("moving_road", True),
            ("n_dashes", 0),        # >0 spawns/animates ground dashes (advanced)
            ("dash_spacing", 10.0),
        ])
        g = {q.name: q.value for q in p}
        self.n = int(g["n_followers"])
        self.world = str(g["world"])
        self.length = float(g["length"])
        self.lane_y = float(g["lane_y"])
        self.timeout_ms = int(g["timeout_ms"])
        self.svc = f"/world/{self.world}/set_pose"
        self.states: dict[int, VehicleState] = {}
        self._warned = False
        self._miss = 0

        if GzNode is None:
            self.get_logger().error(
                "gz-sim Python bindings not found (%s). Install with:\n"
                "  sudo apt install python3-gz-transport13 python3-gz-msgs10\n"
                "The bridge will idle until then." % _GZ_IMPORT_ERR)
            self.gz = None
        else:
            self.gz = GzNode()
            self.get_logger().info(
                f"gz bridge -> {self.svc} ({self.n} followers + leader)")

        for i in range(self.n + 1):
            self.create_subscription(
                VehicleState, f"/platoon/v{i}/state",
                lambda msg, idx=i: self.states.__setitem__(idx, msg), 10)
        self.create_timer(1.0 / float(g["viz_rate_hz"]), self.tick)

    def _set_pose(self, name: str, x: float, y: float, z: float) -> bool:
        req = GzPose()
        req.name = name
        req.position.x = float(x)
        req.position.y = float(y)
        req.position.z = float(z)
        req.orientation.w = 1.0
        # Harmonic Python signature (gz-transport13):
        #   result, response = node.request(service, request,
        #                                   request_type, response_type, timeout_ms)
        ok, _ = self.gz.request(self.svc, req, GzPose, GzBool, self.timeout_ms)
        return bool(ok)

    def tick(self) -> None:
        if self.gz is None or 0 not in self.states:
            return
        leader_x = self.states[0].position
        ok_any = False
        for idx in sorted(self.states):
            s = self.states[idx]
            x = (s.position - leader_x) - 0.5 * self.length
            try:
                ok_any |= self._set_pose(f"vehicle_{idx}", x, self.lane_y, 0.0)
            except Exception as exc:  # transport hiccup: count, warn once
                if not self._warned:
                    self.get_logger().warn(f"set_pose failed: {exc}")
                    self._warned = True
        if not ok_any:
            self._miss += 1
            if self._miss in (50, 250):  # ~1 s and ~5 s of no acks
                self.get_logger().warn(
                    "no set_pose acks yet — is gz-sim running and are the "
                    f"'vehicle_*' models spawned in world '{self.world}'?")
        else:
            self._miss = 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GzBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass


if __name__ == "__main__":
    main()
