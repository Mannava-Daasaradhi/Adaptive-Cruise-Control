"""V2V channel node: delay, Bernoulli packet loss, multiplicative noise.

One node models all N directed links (vehicle i-1 -> vehicle i). For each
incoming beacon on ``/platoon/v{i-1}/beacon`` it draws an i.i.d. loss
decision, scales the acceleration payload by the Ma-2025 n-bit
multiplicative noise factor w in [1 - 1/rho, 1 + 1/rho), and schedules
delivery on ``/platoon/v{i}/v2v`` after ``delay_s`` seconds (min-heap +
high-rate flush timer). ``noise_rho = 0`` disables noise.
"""

from __future__ import annotations

import heapq

import numpy as np
import rclpy
from rclpy.node import Node

from cacc.network import MA2025_GAMMAS

from cacc_platoon_msgs.msg import V2VBeacon


class ChannelNode(Node):
    def __init__(self) -> None:
        super().__init__("v2v_channel")
        p = self.declare_parameters("", [
            ("n_followers", 6),
            ("delay_s", 0.0),
            ("loss_prob", 0.0),
            ("noise_rho", 5.0),  # 0 = noiseless channel
            ("seed", 1),
        ])
        g = {q.name: q.value for q in p}
        self.n = int(g["n_followers"])
        self.delay = float(g["delay_s"])
        self.loss = float(g["loss_prob"])
        rho = float(g["noise_rho"])
        self.rho = rho if rho > 1.0 else None
        self._gammas = np.asarray(MA2025_GAMMAS)
        self._weights = 2.0 ** (-np.arange(self._gammas.size))
        self._rngs = [np.random.default_rng([int(g["seed"]), i, 0xCACC])
                      for i in range(self.n)]
        self._heap: list[tuple[float, int, int, V2VBeacon]] = []
        self._count = 0
        self.pubs = [self.create_publisher(V2VBeacon, f"/platoon/v{i + 1}/v2v", 10)
                     for i in range(self.n)]
        for i in range(self.n):
            self.create_subscription(
                V2VBeacon, f"/platoon/v{i}/beacon",
                lambda msg, link=i: self._on_beacon(link, msg), 10)
        self.create_timer(0.002, self._flush)  # 500 Hz delivery resolution
        self.get_logger().info(
            f"channel: {self.n} links, delay={self.delay}s, loss={self.loss}, "
            f"rho={self.rho}")

    def _w(self, link: int) -> float:
        """Draw one n-bit multiplicative noise factor for this link."""
        if self.rho is None:
            return 1.0
        bits = self._rngs[link].random(self._gammas.size) < self._gammas
        return float((1.0 - 1.0 / self.rho) + (bits @ self._weights) / self.rho)

    def _on_beacon(self, link: int, msg: V2VBeacon) -> None:
        if self.loss > 0.0 and self._rngs[link].random() < self.loss:
            return  # packet lost
        w = self._w(link)
        out = V2VBeacon(sender=msg.sender, t=msg.t,
                        accel=w * msg.accel, u_cmd=w * msg.u_cmd)
        out.stamp = msg.stamp
        due = self.get_clock().now().nanoseconds * 1e-9 + self.delay
        self._count += 1
        heapq.heappush(self._heap, (due, self._count, link, out))

    def _flush(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        while self._heap and self._heap[0][0] <= now:
            _, _, link, msg = heapq.heappop(self._heap)
            self.pubs[link].publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ChannelNode()
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
