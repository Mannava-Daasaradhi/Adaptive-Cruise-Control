"""rviz2 visualization node: the platoon as driving 3D cars.

Publishes a MarkerArray on ``/platoon/markers`` (CUBE car bodies colored by
spacing error, text labels, lane dashes) and a TF ``map -> platoon`` at the
leader's position. The shipped ``config/platoon.rviz`` uses ``platoon`` as
the Fixed Frame, so the camera rides along with the platoon while the
world-fixed lane dashes slide past — that relative motion is what makes it
read as driving (see docs/decisions/D-015).
"""

from __future__ import annotations

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from cacc_platoon_msgs.msg import VehicleState

CAR_W = 1.9  # [m] body width
CAR_H = 1.5  # [m] body height
LANE_HALF = 3.6  # [m] road half-width
DASH_SPACING = 10.0  # [m] world-fixed lane-dash grid
DASH_WINDOW = 300.0  # [m] dashes kept around the leader
ERR_SAT = 1.0  # [m] |e| that saturates the error color scale


def _error_color(e: float) -> tuple[float, float, float]:
    """Green (tight) -> red (|e| >= ERR_SAT) heat for the car body."""
    s = min(abs(e) / ERR_SAT, 1.0)
    return 0.15 + 0.75 * s, 0.75 - 0.65 * s, 0.15


class VizNode(Node):
    def __init__(self) -> None:
        super().__init__("viz")
        p = self.declare_parameters("", [
            ("n_followers", 6),
            ("viz_rate_hz", 20.0),
            ("length", 4.0),
        ])
        g = {q.name: q.value for q in p}
        self.n = int(g["n_followers"])
        self.length = float(g["length"])
        self.states: dict[int, VehicleState] = {}
        for i in range(self.n + 1):
            self.create_subscription(
                VehicleState, f"/platoon/v{i}/state",
                lambda msg, idx=i: self.states.__setitem__(idx, msg), 10)
        self.pub = self.create_publisher(MarkerArray, "/platoon/markers", 10)
        self.tf = TransformBroadcaster(self)
        self.create_timer(1.0 / float(g["viz_rate_hz"]), self.tick)
        self.get_logger().info(f"viz: {self.n} followers, markers on /platoon/markers")

    # ------------------------------------------------------------ markers
    def _car(self, idx: int, s: VehicleState, stamp) -> list[Marker]:
        body = Marker()
        body.header.frame_id = "map"
        body.header.stamp = stamp
        body.ns, body.id = "cars", idx
        body.type, body.action = Marker.CUBE, Marker.ADD
        # s.position is the front bumper; the box is centered on the body
        body.pose.position.x = s.position - 0.5 * self.length
        body.pose.position.z = 0.5 * CAR_H
        body.pose.orientation.w = 1.0
        body.scale.x, body.scale.y, body.scale.z = self.length, CAR_W, CAR_H
        if idx == 0:
            r, g, b = 0.95, 0.95, 0.95  # leader: white
        else:
            r, g, b = _error_color(s.spacing_error)
        body.color.r, body.color.g, body.color.b, body.color.a = r, g, b, 1.0
        body.lifetime.sec = 1

        label = Marker()
        label.header.frame_id = "map"
        label.header.stamp = stamp
        label.ns, label.id = "labels", idx
        label.type, label.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        label.pose.position.x = s.position - 0.5 * self.length
        label.pose.position.z = CAR_H + 1.0
        label.scale.z = 0.8  # text height
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        role = "LEAD" if idx == 0 else f"v{idx}"
        label.text = (f"{role}  {s.velocity:5.1f} m/s"
                      + ("" if idx == 0 else f"  e={s.spacing_error:+.2f} m"))
        label.lifetime.sec = 1
        return [body, label]

    def _road(self, leader_x: float, stamp) -> list[Marker]:
        out = []
        # asphalt slab, centered on the platoon (uniform -> motion-neutral)
        slab = Marker()
        slab.header.frame_id = "map"
        slab.header.stamp = stamp
        slab.ns, slab.id = "road", 0
        slab.type, slab.action = Marker.CUBE, Marker.ADD
        slab.pose.position.x = leader_x - 100.0
        slab.pose.position.z = -0.06
        slab.pose.orientation.w = 1.0
        slab.scale.x, slab.scale.y, slab.scale.z = 2.5 * DASH_WINDOW, 2 * LANE_HALF, 0.1
        slab.color.r = slab.color.g = slab.color.b = 0.22
        slab.color.a = 1.0
        slab.lifetime.sec = 1
        out.append(slab)
        # world-fixed lane dashes: these slide past in the platoon frame
        first = int((leader_x - DASH_WINDOW) // DASH_SPACING)
        last = int((leader_x + DASH_WINDOW / 3) // DASH_SPACING)
        for k in range(first, last + 1):
            for side, y in ((0, LANE_HALF - 0.15), (1, -(LANE_HALF - 0.15))):
                m = Marker()
                m.header.frame_id = "map"
                m.header.stamp = stamp
                m.ns, m.id = "dashes", 2 * (k % 64) + side  # ids reused
                m.type, m.action = Marker.CUBE, Marker.ADD
                m.pose.position.x = k * DASH_SPACING
                m.pose.position.y = y
                m.pose.position.z = 0.01
                m.pose.orientation.w = 1.0
                m.scale.x, m.scale.y, m.scale.z = 3.0, 0.25, 0.02
                m.color.r = m.color.g = m.color.b = 0.9
                m.color.a = 1.0
                m.lifetime.sec = 1
                out.append(m)
        return out

    # ------------------------------------------------------------------ tick
    def tick(self) -> None:
        if 0 not in self.states:
            return  # nothing to anchor the scene to yet
        stamp = self.get_clock().now().to_msg()
        leader_x = self.states[0].position

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = "map"
        tf.child_frame_id = "platoon"
        tf.transform.translation.x = leader_x
        tf.transform.rotation.w = 1.0
        self.tf.sendTransform(tf)

        arr = MarkerArray()
        arr.markers.extend(self._road(leader_x, stamp))
        for idx in sorted(self.states):
            arr.markers.extend(self._car(idx, self.states[idx], stamp))
        self.pub.publish(arr)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VizNode()
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
