import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker


class HandoverZoneNode(Node):
    def __init__(self) -> None:
        super().__init__("handover_zone_node")

        self.subscription = self.create_subscription(
            PointStamped,
            "/handover/palm_center_3d",
            self.palm_callback,
            10,
        )

        self.zone_pub = self.create_publisher(
            Bool,
            "/handover/hand_in_zone",
            10,
        )

        self.marker_pub = self.create_publisher(
            Marker,
            "/handover/markers",
            10,
        )

        # Camera-frame handover zone bounds
        self.x_min = self.declare_parameter("x_min", -0.30).value
        self.x_max = self.declare_parameter("x_max", -0.10).value
        self.y_min = self.declare_parameter("y_min", -0.30).value
        self.y_max = self.declare_parameter("y_max", -0.10).value
        self.z_min = self.declare_parameter("z_min",  0.60).value
        self.z_max = self.declare_parameter("z_max",  0.80).value

        self.required_consecutive_frames = int(
            self.declare_parameter("required_consecutive_frames", 10).value
        )

        self.in_zone_count = 0
        self.current_state = False

        self.get_logger().info(
            f"Handover zone node started. "
            f"x:[{self.x_min}, {self.x_max}] "
            f"y:[{self.y_min}, {self.y_max}] "
            f"z:[{self.z_min}, {self.z_max}] "
            f"stable_frames={self.required_consecutive_frames}"
        )

    def palm_callback(self, msg: PointStamped) -> None:
        x = msg.point.x
        y = msg.point.y
        z = msg.point.z

        self.publish_zone_marker(msg.header.frame_id)

        inside = (
            self.x_min <= x <= self.x_max and
            self.y_min <= y <= self.y_max and
            self.z_min <= z <= self.z_max
        )

        if inside:
            self.in_zone_count += 1
        else:
            self.in_zone_count = 0

        new_state = self.in_zone_count >= self.required_consecutive_frames

        if new_state != self.current_state:
            self.current_state = new_state
            self.get_logger().info(
                f"hand_in_zone -> {self.current_state} "
                f"(frame={msg.header.frame_id}, "
                f"x={x:.3f}, y={y:.3f}, z={z:.3f}, count={self.in_zone_count})"
            )

        out = Bool()
        out.data = self.current_state
        self.zone_pub.publish(out)

    def publish_zone_marker(self, frame_id: str) -> None:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "handover_zone"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = (self.x_min + self.x_max) / 2.0
        marker.pose.position.y = (self.y_min + self.y_max) / 2.0
        marker.pose.position.z = (self.z_min + self.z_max) / 2.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = self.x_max - self.x_min
        marker.scale.y = self.y_max - self.y_min
        marker.scale.z = self.z_max - self.z_min

        marker.color.a = 0.20
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0

        self.marker_pub.publish(marker)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HandoverZoneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()