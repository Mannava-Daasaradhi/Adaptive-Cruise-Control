"""Recorder node: subscribes every vehicle's state and writes one CSV.

Columns: index, t, position, velocity, acceleration, u_cmd, spacing_error.
Rows are appended as messages arrive (per-node local sim time ``t``); the
file is flushed periodically and closed cleanly on shutdown, so a timed
``ros2 launch`` (SIGINT) always leaves a complete file behind.
"""

from __future__ import annotations

import csv
from pathlib import Path

import rclpy
from rclpy.node import Node

from cacc_platoon_msgs.msg import VehicleState


class RecorderNode(Node):
    def __init__(self) -> None:
        super().__init__("recorder")
        p = self.declare_parameters("", [
            ("n_followers", 6),
            ("outfile", "platoon_run.csv"),
        ])
        g = {q.name: q.value for q in p}
        self.path = Path(str(g["outfile"])).expanduser()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = open(self.path, "w", newline="", encoding="utf-8")
        self._csv = csv.writer(self._fh)
        self._csv.writerow(["index", "t", "position", "velocity",
                            "acceleration", "u_cmd", "spacing_error"])
        self._rows = 0
        n = int(g["n_followers"])
        for i in range(n + 1):
            self.create_subscription(
                VehicleState, f"/platoon/v{i}/state", self._on_state, 50)
        self.get_logger().info(f"recording {n + 1} vehicles -> {self.path}")

    def _on_state(self, m: VehicleState) -> None:
        self._csv.writerow([m.index, f"{m.t:.4f}", f"{m.position:.6f}",
                            f"{m.velocity:.6f}", f"{m.acceleration:.6f}",
                            f"{m.u_cmd:.6f}", f"{m.spacing_error:.6f}"])
        self._rows += 1
        if self._rows % 500 == 0:
            self._fh.flush()

    def close(self) -> None:
        self._fh.flush()
        self._fh.close()
        self.get_logger().info(f"wrote {self._rows} rows to {self.path}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        try:  # context may already be torn down by the SIGINT handler
            node.destroy_node()
        except Exception:
            pass


if __name__ == "__main__":
    main()
