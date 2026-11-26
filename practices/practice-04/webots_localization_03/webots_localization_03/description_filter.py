import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String


class DescriptionFilter(Node):
    def __init__(self):
        super().__init__("description_filter")
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(
            String, "/robot_description", qos
        )
        self.subscription = self.create_subscription(
            String, "/robot_description_raw", self.receive, qos
        )

    def receive(self, message):
        try:
            root = ET.fromstring(message.data)
        except ET.ParseError:
            self.get_logger().warning("Invalid robot description skipped")
            return
        if root.find("ros2_control") is None:
            self.get_logger().info("Temporary robot description skipped")
            return
        self.publisher.publish(message)
        self.get_logger().info("Robot description with ros2_control forwarded")


def main(args=None):
    rclpy.init(args=args)
    node = DescriptionFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
