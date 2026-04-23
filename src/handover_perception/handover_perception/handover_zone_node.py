import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool


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

        # Tune these later based on your setup
        self.x_min = self.declare_parameter("x_min", -0.20).value
        self.x_max = self.declare_parameter("x_max",  0.20).value
        self.y_min = self.declare_parameter("y_min", -0.20).value
        self.y_max = self.declare_parameter("y_max",  0.20).value
        self.z_min = self.declare_parameter("z_min",  0.30).value
        self.z_max = self.declare_parameter("z_max",  0.50).value

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
                f"(x={x:.3f}, y={y:.3f}, z={z:.3f}, count={self.in_zone_count})"
            )

        out = Bool()
        out.data = self.current_state
        self.zone_pub.publish(out)


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