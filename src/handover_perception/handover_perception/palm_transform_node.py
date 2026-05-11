import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point


class PalmTransformNode(Node):
    def __init__(self) -> None:
        super().__init__("palm_transform_node")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.palm_sub = self.create_subscription(
            PointStamped,
            "/handover/palm_center_3d",
            self.palm_callback,
            10,
        )

        self.palm_base_pub = self.create_publisher(
            PointStamped,
            "/handover/palm_center_base",
            10,
        )

        self.target_pub = self.create_publisher(
            PointStamped,
            "/handover/approach_target",
            10,
        )

        self.marker_pub = self.create_publisher(
            Marker,
            "/handover/markers",
            10,
        )

        self.offset_m = float(self.declare_parameter("offset_m", 0.10).value)
        self.target_frame = str(self.declare_parameter("target_frame", "base_link").value)

        self.get_logger().info(
            f"Palm transform node started. target_frame={self.target_frame}, offset_m={self.offset_m}"
        )

    def palm_callback(self, msg: PointStamped) -> None:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                rclpy.time.Time(),
            )
        except Exception as exc:
            self.get_logger().warn(f"TF lookup failed: {exc}", throttle_duration_sec=2.0)
            return

        try:
            palm_base = do_transform_point(msg, transform)
        except Exception as exc:
            self.get_logger().error(f"Point transform failed: {exc}")
            return

        self.palm_base_pub.publish(palm_base)

        px = palm_base.point.x
        py = palm_base.point.y
        pz = palm_base.point.z

        target = PointStamped()
        target.header = palm_base.header
        target.point.x = px
        target.point.y = py
        target.point.z = pz

        self.target_pub.publish(target)

        self.publish_marker(
            palm_base,
            marker_id=1,
            ns="palm",
            rgb=(0.0, 1.0, 0.0),
            scale=0.05,
        )

        # self.publish_marker(
        #     target,
        #     marker_id=1,
        #     ns="handover",
        #     rgb=(1.0, 0.0, 0.0),
        #     scale=0.05,
        # )

        self.get_logger().info(
            f"Palm(base): ({px:.3f}, {py:.3f}, {pz:.3f})  "
            f"Target: ({target.point.x:.3f}, {target.point.y:.3f}, {target.point.z:.3f})",
            throttle_duration_sec=2.0,
        )

    def publish_marker(
        self,
        point_msg: PointStamped,
        marker_id: int,
        ns: str,
        rgb: tuple[float, float, float],
        scale: float,
    ) -> None:
        marker = Marker()
        marker.header = point_msg.header
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = point_msg.point.x
        marker.pose.position.y = point_msg.point.y
        marker.pose.position.z = point_msg.point.z
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.a = 1.0
        marker.color.r = rgb[0]
        marker.color.g = rgb[1]
        marker.color.b = rgb[2]
        self.marker_pub.publish(marker)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PalmTransformNode()
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