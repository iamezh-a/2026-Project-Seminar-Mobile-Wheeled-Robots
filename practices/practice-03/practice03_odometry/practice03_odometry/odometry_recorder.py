from bisect import bisect_right
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.clock import ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer, TransformException, TransformListener

from .motion_model import integrate_pose
from .results import save_results


def stamp_to_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def planar_pose(transform):
    position = transform.transform.translation
    q = transform.transform.rotation

    yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
    return position.x, position.y, yaw


class OdometryRecorder(Node):
    def __init__(self):
        super().__init__("odometry_recorder")

        self.declare_parameter("duration", 120.0)
        self.declare_parameter("output_dir", "results")
        self.declare_parameter("experiment_name", "noise_0")

        self.duration_s = float(self.get_parameter("duration").value)
        if not math.isfinite(self.duration_s) or self.duration_s <= 0:
            raise ValueError("duration must be a positive finite number")

        self.duration_ns = int(round(self.duration_s * 1_000_000_000))
        if self.duration_ns < 1:
            raise ValueError("duration is too small")

        self.output_dir = str(Path(
            self.get_parameter("output_dir").value
        ).expanduser().resolve())
        self.experiment_name = self.get_parameter("experiment_name").value

        self.latest_clock_ns = None
        self.command_times = []
        self.commands = []
        self.first_motion_ns = None

        self.start_ns = None
        self.last_sample_ns = None
        self.initial_pose = None
        self.command_pose = None
        self.samples = []

        self.finished = False
        self.failed = False
        self.next_progress_s = 10.0

        self.tf_buffer = Buffer(
            cache_time=Duration(seconds=30.0),
            node=self,
        )
        self.tf_listener = TransformListener(self.tf_buffer, self)

        clock_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.clock_subscription = self.create_subscription(
            Clock, "/clock", self.on_clock, clock_qos
        )
        self.command_subscription = self.create_subscription(
            TwistStamped, "/cmd_vel", self.on_command, 1000
        )
        self.timer = self.create_timer(0.1, self.sample)

        self.get_logger().info(
            "Waiting for /clock, movement commands and both TF branches..."
        )

    def fail(self, message):
        self.get_logger().error(message)
        self.failed = True
        self.finished = True
        self.timer.cancel()

    def on_clock(self, message):
        now_ns = stamp_to_ns(message.clock)

        if (
            self.latest_clock_ns is not None
            and now_ns < self.latest_clock_ns
        ):
            self.fail(
                "Simulation time moved backwards. Restart the experiment."
            )
            return

        self.latest_clock_ns = now_ns

    def on_command(self, message):
        if self.finished:
            return

        stamp_ns = stamp_to_ns(message.header.stamp)
        v = float(message.twist.linear.x)
        omega = float(message.twist.angular.z)

        if not math.isfinite(v) or not math.isfinite(omega):
            self.fail("Received a non-finite velocity command.")
            return

        if self.command_times and stamp_ns < self.command_times[-1]:
            self.fail("Command timestamps moved backwards.")
            return

        if self.command_times and stamp_ns == self.command_times[-1]:
            self.commands[-1] = (v, omega)
        else:
            self.command_times.append(stamp_ns)
            self.commands.append((v, omega))

        if self.first_motion_ns is None and (v != 0.0 or omega != 0.0):
            self.first_motion_ns = stamp_ns

    def integrate_commands(self, start_ns, end_ns):
        # Находим команду, действовавшую в начале интервала.
        index = bisect_right(self.command_times, start_ns) - 1
        v, omega = self.commands[index]

        pose = self.command_pose
        cursor_ns = start_ns

        # Если внутри интервала менялась команда, интегрируем по частям.
        for next_index in range(index + 1, len(self.command_times)):
            event_ns = self.command_times[next_index]
            if event_ns > end_ns:
                break

            dt = (event_ns - cursor_ns) / 1_000_000_000
            # *pose передаёт x, y, yaw отдельными аргументами.
            pose = integrate_pose(*pose, v, omega, dt)

            cursor_ns = event_ns
            v, omega = self.commands[next_index]

        dt = (end_ns - cursor_ns) / 1_000_000_000
        return integrate_pose(*pose, v, omega, dt)

    def sample(self):
        if self.finished:
            return

        if self.latest_clock_ns is None or self.first_motion_ns is None:
            return

        try:
            # Нулевое время в запросе TF означает последнюю доступную позу.
            latest = Time(clock_type=ClockType.ROS_TIME)
            latest_gt = self.tf_buffer.lookup_transform(
                "map", "base_link_gt", latest
            )
            latest_encoder = self.tf_buffer.lookup_transform(
                "map", "base_link", latest
            )

            # Выбираем время, уже достигнутое всеми источниками данных.
            sample_ns = min(
                stamp_to_ns(latest_gt.header.stamp),
                stamp_to_ns(latest_encoder.header.stamp),
                self.latest_clock_ns,
                self.command_times[-1],
            )

            if sample_ns <= 0 or sample_ns < self.first_motion_ns:
                return

            if self.start_ns is not None:
                sample_ns = min(
                    sample_ns,
                    self.start_ns + self.duration_ns,
                )
                if sample_ns <= self.last_sample_ns:
                    return

            sample_time = Time(
                nanoseconds=sample_ns,
                clock_type=ClockType.ROS_TIME,
            )

            # Обе оценки запрашиваются в map на одну временную метку.
            gt = self.tf_buffer.lookup_transform(
                "map", "base_link_gt", sample_time
            )
            encoder = self.tf_buffer.lookup_transform(
                "map", "base_link", sample_time
            )

        except TransformException as error:
            self.get_logger().warning(
                f"Waiting for synchronized TF: {error}",
                throttle_duration_sec=5.0,
            )
            return

        gt_pose = planar_pose(gt)
        encoder_pose = planar_pose(encoder)

        if self.start_ns is None:
            self.start_ns = sample_ns
            self.initial_pose = gt_pose

            # Единственное использование GT для командной траектории.
            self.command_pose = gt_pose

            self.get_logger().info(
                f"Recording started at simulation time "
                f"{self.start_ns / 1_000_000_000:.3f} s; "
                f"duration: {self.duration_s:.1f} s"
            )
        else:
            self.command_pose = self.integrate_commands(
                self.last_sample_ns, sample_ns
            )

        elapsed_s = (sample_ns - self.start_ns) / 1_000_000_000
        self.samples.append((
            elapsed_s,
            *gt_pose,
            *encoder_pose,
            *self.command_pose,
        ))
        self.last_sample_ns = sample_ns

        if elapsed_s >= self.next_progress_s:
            self.get_logger().info(
                f"Recorded {elapsed_s:.1f}/{self.duration_s:.1f} s; "
                f"samples: {len(self.samples)}"
            )
            self.next_progress_s += 10.0

        if sample_ns == self.start_ns + self.duration_ns:
            metadata = {
                "experiment_name": self.experiment_name,
                "requested_duration_s": self.duration_s,
                "start_sim_time_s": self.start_ns / 1_000_000_000,
                "initial_gt_pose": list(self.initial_pose),
                "frame": "map",
                "time_alignment": "TF lookups at a common timestamp",
            }
            paths = save_results(
                self.samples,
                self.output_dir,
                self.experiment_name,
                metadata,
            )

            self.finished = True
            self.timer.cancel()

            for path in paths:
                self.get_logger().info(f"Saved: {path}")

            self.get_logger().info("Experiment completed.")


def main(args=None):
    rclpy.init(args=args)
    node = None
    failed = False

    try:
        node = OdometryRecorder()
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
        failed = node.failed

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if failed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
