#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class MoveSequenceNode(Node):
    def __init__(self) -> None:
        super().__init__("move_sequence_node")

        self.publisher = self.create_publisher(
            JointTrajectory,
            "/scaled_joint_trajectory_controller/joint_trajectory",
            10,
        )

        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        self.home = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        self.handover = [0.4, -1.2, 1.0, -1.4, -1.57, 0.0]

        self.timer = self.create_timer(2.0, self.run_once)
        self.sent = False

    def make_point(self, positions: list[float], sec: int) -> JointTrajectoryPoint:
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=sec, nanosec=0)
        return point

    def run_once(self) -> None:
        if self.sent:
            return
        
        target = self.handover 

        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        msg.points = [
            self.make_point(target, 4),
        ]

        self.publisher.publish(msg)
        self.get_logger().info("Published home -> handover -> home trajectory")
        self.sent = True


def main() -> None:
    rclpy.init()
    node = MoveSequenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()