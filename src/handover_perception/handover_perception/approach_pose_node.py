import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker

import tf2_ros
from tf2_ros import TransformException


class ApproachPoseNode(Node):
    def __init__(self) -> None:
        super().__init__("approach_pose_node")

        self.target_sub = self.create_subscription(
            PointStamped,
            "/handover/approach_target",
            self.target_callback,
            10,
        )

        self.zone_sub = self.create_subscription(
            Bool,
            "/handover/hand_in_zone",
            self.zone_callback,
            10,
        )

        self.pose_pub = self.create_publisher(
            PoseStamped,
            "/handover/approach_pose",
            10,
        )

        self.marker_pub = self.create_publisher(
            Marker,
            "/handover/markers",
            10,
        )

        self.hand_in_zone = False

        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.tcp_frame = self.declare_parameter("tcp_frame", "tool0_controller").value
        self.offset_distance = float(
            self.declare_parameter("offset_distance", 0.10).value
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info(
            f"Approach pose node started. "
            f"base_frame={self.base_frame}, "
            f"tcp_frame={self.tcp_frame}, "
            f"offset_distance={self.offset_distance:.3f} m"
        )

    def zone_callback(self, msg: Bool) -> None:
        self.hand_in_zone = msg.data

    def target_callback(self, msg: PointStamped) -> None:
        try:
            tf = self.tf_buffer.lookup_transform(
                msg.header.frame_id,
                self.tcp_frame,
                rclpy.time.Time(),
            )
        except TransformException as ex:
            self.get_logger().warn(
                f"Could not get transform from {msg.header.frame_id} "
                f"to {self.tcp_frame}: {ex}",
                throttle_duration_sec=2.0,
            )
            return

        tcp_x = tf.transform.translation.x
        tcp_y = tf.transform.translation.y
        tcp_z = tf.transform.translation.z

        target_x = msg.point.x
        target_y = msg.point.y
        target_z = msg.point.z

        dx = tcp_x - target_x
        dy = tcp_y - target_y
        dz = tcp_z - target_z

        length = math.sqrt(dx * dx + dy * dy + dz * dz)

        if length < 1e-6:
            self.get_logger().warn(
                "TCP and target are effectively at the same point; cannot compute approach offset.",
                throttle_duration_sec=2.0,
            )
            return

        ux = dx / length
        uy = dy / length
        uz = dz / length

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = msg.header.frame_id

        pose.pose.position.x = target_x + self.offset_distance * ux
        pose.pose.position.y = target_y + self.offset_distance * uy
        pose.pose.position.z = target_z + self.offset_distance * uz

        pose.pose.orientation.x = 0.7152253
        pose.pose.orientation.y = 0.0085253
        pose.pose.orientation.z = -0.0130123
        pose.pose.orientation.w = 0.6987208

        marker = Marker()
        marker.header = pose.header
        marker.ns = "approach_pose"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = pose.pose.position.x
        marker.pose.position.y = pose.pose.position.y
        marker.pose.position.z = pose.pose.position.z
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.04
        marker.scale.y = 0.04
        marker.scale.z = 0.04
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.marker_pub.publish(marker)

        if self.hand_in_zone:
            self.pose_pub.publish(pose)

        self.get_logger().info(
            f"Updated approach marker: "
            f"target=({target_x:.3f}, {target_y:.3f}, {target_z:.3f}), "
            f"tcp=({tcp_x:.3f}, {tcp_y:.3f}, {tcp_z:.3f}), "
            f"approach=({pose.pose.position.x:.3f}, "
            f"{pose.pose.position.y:.3f}, "
            f"{pose.pose.position.z:.3f}), "
            f"hand_in_zone={self.hand_in_zone}",
            throttle_duration_sec=2.0,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ApproachPoseNode()
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