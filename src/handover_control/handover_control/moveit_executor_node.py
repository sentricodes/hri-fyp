import math
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

from pymoveit2 import MoveIt2


class MoveItExecutorNode(Node):
    def __init__(self) -> None:
        super().__init__("moveit_executor_node")

        self.callback_group = ReentrantCallbackGroup()

        self.pose_sub = self.create_subscription(
            PoseStamped,
            "/handover/approach_pose",
            self.pose_callback,
            10,
            callback_group=self.callback_group,
        )

        self.zone_sub = self.create_subscription(
            Bool,
            "/handover/hand_in_zone",
            self.zone_callback,
            10,
            callback_group=self.callback_group,
        )

        self.hand_in_zone = False
        self.executing = False
        self.last_executed_pose = None

        # These should match your UR MoveIt setup
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        self.base_link_name = "base_link"
        self.end_effector_name = "tool0"
        self.group_name = "ur_manipulator"

        self.min_replan_distance = float(
            self.declare_parameter("min_replan_distance", 0.03).value
        )
        self.cartesian = bool(
            self.declare_parameter("cartesian", False).value
        )

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=self.joint_names,
            base_link_name=self.base_link_name,
            end_effector_name=self.end_effector_name,
            group_name=self.group_name,
            callback_group=self.callback_group,
            use_move_group_action=True,
        )

        self.get_logger().info(
            f"MoveIt executor started. group={self.group_name}, ee={self.end_effector_name}"
        )

    def zone_callback(self, msg: Bool) -> None:
        self.hand_in_zone = msg.data

    def pose_callback(self, msg: PoseStamped) -> None:
        if not self.hand_in_zone:
            return

        if self.executing:
            return

        if self.last_executed_pose is not None:
            dx = msg.pose.position.x - self.last_executed_pose.pose.position.x
            dy = msg.pose.position.y - self.last_executed_pose.pose.position.y
            dz = msg.pose.position.z - self.last_executed_pose.pose.position.z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if dist < self.min_replan_distance:
                return

        self.executing = True
        threading.Thread(
            target=self.move_to_pose_thread,
            args=(msg,),
            daemon=True,
        ).start()

    def move_to_pose_thread(self, msg: PoseStamped) -> None:
        try:
            position = [
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            ]
            quat_xyzw = [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ]

            self.get_logger().info(
                f"Moving to x={position[0]:.3f}, y={position[1]:.3f}, z={position[2]:.3f}",
                throttle_duration_sec=1.0,
            )

            self.moveit2.move_to_pose(
                position=position,
                quat_xyzw=quat_xyzw,
                cartesian=self.cartesian,
            )
            self.moveit2.wait_until_executed()

            self.last_executed_pose = msg

        except Exception as exc:
            self.get_logger().error(f"MoveIt execution failed: {exc}")
        finally:
            self.executing = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MoveItExecutorNode()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()