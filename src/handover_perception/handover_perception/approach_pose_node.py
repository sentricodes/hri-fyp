import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Bool


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

        self.hand_in_zone = False

        self.get_logger().info("Approach pose node started.")

    def zone_callback(self, msg: Bool) -> None:
        self.hand_in_zone = msg.data

    def target_callback(self, msg: PointStamped) -> None:
        if not self.hand_in_zone:
            return

        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x = msg.point.x
        pose.pose.position.y = msg.point.y
        pose.pose.position.z = msg.point.z

        pose.pose.orientation.x = 0.7152253
        pose.pose.orientation.y = 0.0085253
        pose.pose.orientation.z = -0.0130123
        pose.pose.orientation.w = 0.6987208

        self.pose_pub.publish(pose)

        self.get_logger().info(
            f"Published approach pose: "
            f"x={pose.pose.position.x:.3f}, "
            f"y={pose.pose.position.y:.3f}, "
            f"z={pose.pose.position.z:.3f}",
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