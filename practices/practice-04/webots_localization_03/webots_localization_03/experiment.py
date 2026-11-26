import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener, TransformException


def yaw(transform):
    q = transform.transform.rotation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class Experiment(Node):
    def __init__(self):
        super().__init__("localization_experiment")
        defaults = {
            "duration": 60.0,
            "position_threshold": 0.10,
            "yaw_threshold": 0.05,
            "confirmation_duration": 10.0,
            "localization_mode": "initial",
            "run_id": "preliminary",
            "output_dir": (
                "/home/ezh/practices_ws/src/practice-04/results"
            ),
            "min_particles": 500,
            "max_particles": 1000,
            "error_x": 0.0,
            "error_y": 0.0,
            "error_yaw": 0.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.settings = {
            name: self.get_parameter(name).value for name in defaults
        }
        self.duration = float(self.settings["duration"])
        self.position_limit = float(self.settings["position_threshold"])
        self.angle_limit = float(self.settings["yaw_threshold"])
        self.hold = float(self.settings["confirmation_duration"])
        self.mode = self.settings["localization_mode"]

        if (
            self.duration <= 0 or self.hold <= 0
            or self.position_limit <= 0 or self.angle_limit <= 0
            or self.duration < self.hold
        ):
            raise ValueError("Invalid duration or convergence thresholds")
        if self.mode not in ("initial", "global"):
            raise ValueError("Unknown localization_mode")

        run_id = self.settings["run_id"]
        if not run_id or Path(run_id).name != run_id or run_id in (".", ".."):
            raise ValueError("run_id must be a single directory name")
        self.output = Path(self.settings["output_dir"]) / run_id
        self.output.mkdir(parents=True, exist_ok=False)

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.on_pose, 10
        )
        self.state_client = self.create_client(GetState, "/amcl/get_state")
        self.global_client = self.create_client(
            Empty, "/reinitialize_global_localization"
        )
        self.future = None
        self.phase = "wait_active"
        self.have_pose = False
        self.t0 = None
        self.last_stamp = None
        self.interval_start = None
        self.convergence = None
        self.rows = []
        self.done = False
        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().info(
            f"Waiting for AMCL; results directory: {self.output}"
        )

    def on_pose(self, message):
        self.have_pose = True

    def initialize(self):
        if self.phase == "wait_active":
            if self.future is None:
                if self.state_client.service_is_ready():
                    self.future = self.state_client.call_async(
                        GetState.Request()
                    )
                return
            if not self.future.done():
                return
            active = self.future.result().current_state.id == 3
            self.future = None
            if active:
                self.phase = (
                    "global_init" if self.mode == "global" else "collect"
                )
            return

        if self.phase == "global_init":
            if self.future is None:
                if self.global_client.service_is_ready():
                    self.future = self.global_client.call_async(
                        Empty.Request()
                    )
                return
            if not self.future.done():
                return
            self.future.result()
            self.future = None
            self.have_pose = False
            self.phase = "collect"
            self.get_logger().info("Global initialization completed")

    def tick(self):
        if self.done:
            return
        if self.phase != "collect":
            self.initialize()
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if self.t0 is not None and now - self.t0 >= self.duration:
            self.save()
            return
        if not self.have_pose:
            return

        try:
            # Берём время последней одометрии и сравниваем обе позы
            # именно в этот момент, а не в разные моменты времени.
            odom = self.buffer.lookup_transform(
                "odom", "base_link", Time()
            )
            stamp = Time.from_msg(odom.header.stamp)
            sample_time = stamp.nanoseconds / 1e9
            if now - sample_time > 0.3 or sample_time > now:
                self.interval_start = None
                return
            if self.last_stamp is not None and sample_time <= self.last_stamp:
                if now - self.last_stamp > 0.3:
                    self.interval_start = None
                return
            gt = self.buffer.lookup_transform(
                "map", "base_link_gt", stamp
            )
            estimate = self.buffer.lookup_transform(
                "map", "base_link", stamp
            )
        except TransformException:
            if self.last_stamp is not None and now - self.last_stamp > 0.3:
                self.interval_start = None
            return

        if self.t0 is None:
            self.t0 = sample_time
            self.get_logger().info("First AMCL estimate received: t0 set")
        if self.last_stamp is not None and sample_time - self.last_stamp > 0.3:
            self.interval_start = None
        self.last_stamp = sample_time

        t = sample_time - self.t0
        g = gt.transform.translation
        e = estimate.transform.translation
        gyaw, eyaw = yaw(gt), yaw(estimate)
        position_error = math.hypot(e.x - g.x, e.y - g.y)
        angle_error = abs(
            math.atan2(math.sin(eyaw - gyaw), math.cos(eyaw - gyaw))
        )
        self.rows.append([
            t, g.x, g.y, gyaw, e.x, e.y, eyaw,
            position_error, angle_error,
        ])

        inside = (
            position_error <= self.position_limit
            and angle_error <= self.angle_limit
        )
        if not inside:
            self.interval_start = None
        elif self.interval_start is None:
            self.interval_start = t
        elif self.convergence is None and t - self.interval_start >= self.hold:
            self.convergence = self.interval_start
            self.get_logger().info(
                f"Convergence confirmed; convergence time = "
                f"{self.convergence:.2f} s"
            )

    def save(self):
        self.done = True
        self.timer.cancel()
        fields = [
            "time_s", "gt_x_m", "gt_y_m", "gt_yaw_rad",
            "amcl_x_m", "amcl_y_m", "amcl_yaw_rad",
            "position_error_m", "yaw_error_rad",
        ]
        with (self.output / "data.csv").open("w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(fields)
            writer.writerows(self.rows)

        summary = dict(self.settings)
        summary.update({
            "t0_simulation_s": self.t0,
            "sample_count": len(self.rows),
            "convergence_time_s": self.convergence,
            "sampling_period_s": 0.1,
            "maximum_allowed_sample_gap_s": 0.3,
        })
        (self.output / "summary.json").write_text(
            json.dumps(summary, indent=2, ensure_ascii=False) + "\n"
        )

        times = [row[0] for row in self.rows]
        figure, axes = plt.subplots(3, 1, figsize=(10, 13))
        axes[0].plot(
            [r[1] for r in self.rows], [r[2] for r in self.rows],
            label="Ground truth",
        )
        axes[0].plot(
            [r[4] for r in self.rows], [r[5] for r in self.rows],
            label="AMCL",
        )
        axes[0].set(
            xlabel="X (m)", ylabel="Y (m)", title="Robot trajectories"
        )
        axes[0].set_aspect("equal", adjustable="box")
        for axis, column, limit, label in [
            (axes[1], 7, self.position_limit, "Position error (m)"),
            (axes[2], 8, self.angle_limit, "Yaw error (rad)"),
        ]:
            axis.plot(times, [r[column] for r in self.rows], label="Error")
            axis.axhline(
                limit, color="red", linestyle="--", label="Threshold"
            )
            if self.convergence is not None:
                axis.axvline(
                    self.convergence, color="green", linestyle=":",
                    label=f"Convergence: {self.convergence:.2f} s",
                )
            axis.set(xlabel="Time since t0 (s)", ylabel=label)

        for axis in axes:
            axis.grid(True, alpha=0.3)
            axis.legend()
        status = (
            "not confirmed" if self.convergence is None
            else f"{self.convergence:.2f} s"
        )
        figure.suptitle(
            f"{self.settings['run_id']} | Convergence: {status}"
        )
        figure.tight_layout()
        figure.savefig(self.output / "plots.png", dpi=160)
        plt.close(figure)
        self.get_logger().info(
            f"Experiment finished. Convergence: {status}. "
            f"Saved CSV, JSON and plots: {self.output}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Experiment()
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().warning(
                "Experiment interrupted; a complete result was not saved"
            )
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
