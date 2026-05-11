import math
import time
import threading

import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sensor_msgs.msg import JointState

from handover_interfaces.action import MoveToTarget
from pymoveit2 import MoveIt2


class MoveItMotionServerNode(Node):
    def __init__(self) -> None:
        super().__init__("moveit_motion_server_node")

        self.callback_group = ReentrantCallbackGroup()
        self.motion_lock = threading.Lock()
        self.busy = False

        self.current_joint_positions = {}

        self.joint_goal_tolerance = float(
            self.declare_parameter("joint_goal_tolerance", 0.03).value
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
            callback_group=self.callback_group,
        )

        # These should match your UR MoveIt setup.
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

        self.default_velocity_scaling = float(
            self.declare_parameter("max_velocity_scaling", 0.5).value
        )

        self.default_acceleration_scaling = float(
            self.declare_parameter("max_acceleration_scaling", 0.5).value
        )

        self.planner_id = str(
            self.declare_parameter(
                "planner_id",
                "RRTConnectkConfigDefault",
            ).value
        )

        # Keep a separate internal node for pymoveit2, matching your existing setup.
        self.moveit_node = Node("moveit2_interface_node")
        self.moveit_callback_group = ReentrantCallbackGroup()

        self.moveit2 = MoveIt2(
            node=self.moveit_node,
            joint_names=self.joint_names,
            base_link_name=self.base_link_name,
            end_effector_name=self.end_effector_name,
            group_name=self.group_name,
            callback_group=self.moveit_callback_group,
            use_move_group_action=True,
        )

        self.moveit2.planner_id = self.planner_id

        self.motion_action_server = ActionServer(
            self,
            MoveToTarget,
            "/motion/move_to_target",
            execute_callback=self.execute_motion_callback,
            goal_callback=self.goal_callback,
            callback_group=self.callback_group,
        )

        self.get_logger().info(
            f"MoveIt motion server started. group={self.group_name}, "
            f"ee={self.end_effector_name}, planner={self.planner_id}"
        )

    def goal_callback(self, goal_request):
        if self.busy:
            self.get_logger().warn("Rejecting motion goal: robot is already busy.")
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def execute_motion_callback(self, goal_handle):
        goal = goal_handle.request
        result = MoveToTarget.Result()

        with self.motion_lock:
            self.busy = True

            try:
                feedback = MoveToTarget.Feedback()
                feedback.state = "Preparing motion target"
                feedback.elapsed_time = 0.0
                goal_handle.publish_feedback(feedback)

                velocity_scaling = (
                    goal.velocity_scaling
                    if goal.velocity_scaling > 0.0
                    else self.default_velocity_scaling
                )

                acceleration_scaling = (
                    goal.acceleration_scaling
                    if goal.acceleration_scaling > 0.0
                    else self.default_acceleration_scaling
                )

                self.moveit2.max_velocity = velocity_scaling
                self.moveit2.max_acceleration = acceleration_scaling
                self.moveit2.planner_id = self.planner_id

                feedback.state = "Executing motion"
                goal_handle.publish_feedback(feedback)

                if goal.target_type == MoveToTarget.Goal.TARGET_JOINTS:
                    if len(goal.joint_target) != len(self.joint_names):
                        goal_handle.abort()
                        result.success = False
                        result.message = (
                            f"Expected {len(self.joint_names)} joint values, "
                            f"got {len(goal.joint_target)}."
                        )
                        return result

                    self.get_logger().info("Moving to explicit joint target.")

                    joint_positions = list(goal.joint_target)

                    self.moveit2.move_to_configuration(
                        joint_positions=joint_positions,
                    )

                    feedback.state = "Motion command sent; monitoring joint target"
                    goal_handle.publish_feedback(feedback)

                    timeout_sec = goal.timeout_sec if goal.timeout_sec > 0.0 else 20.0

                    if not self.wait_for_joint_goal(joint_positions, timeout_sec):
                        goal_handle.abort()
                        result.success = False
                        result.message = "Timed out waiting for joint target."
                        return result

                elif goal.target_type == MoveToTarget.Goal.TARGET_POSE:
                    pose = goal.pose_target.pose

                    position = [
                        pose.position.x,
                        pose.position.y,
                        pose.position.z,
                    ]

                    quat_xyzw = [
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w,
                    ]

                    cartesian = goal.cartesian

                    self.get_logger().info(
                        "Moving to pose target: "
                        f"x={position[0]:.3f}, "
                        f"y={position[1]:.3f}, "
                        f"z={position[2]:.3f}, "
                        f"cartesian={cartesian}"
                    )

                    if cartesian:
                        self.moveit2.move_to_pose(
                            position=position,
                            quat_xyzw=quat_xyzw,
                            cartesian=True,
                            cartesian_max_step=0.005,
                            cartesian_fraction_threshold=0.95,
                        )
                    else:
                        self.moveit2.move_to_pose(
                            position=position,
                            quat_xyzw=quat_xyzw,
                            cartesian=True,
                            cartesian_max_step=0.005,
                            cartesian_fraction_threshold=0.95,
                        )
                        # self.moveit2.move_to_pose(
                        #     position=position,
                        #     quat_xyzw=quat_xyzw,
                        #     cartesian=False,
                        # )

                else:
                    goal_handle.abort()
                    result.success = False
                    result.message = f"Unknown target_type: {goal.target_type}"
                    return result

                goal_handle.succeed()
                result.success = True

                if goal.target_type == MoveToTarget.Goal.TARGET_JOINTS:
                    result.message = "Joint target reached."
                    feedback.state = "Joint target reached"
                else:
                    result.message = "Pose motion command sent to MoveIt."
                    feedback.state = "Pose motion command sent"

                goal_handle.publish_feedback(feedback)

                return result

            except Exception as exc:
                self.get_logger().error(f"MoveIt motion failed: {exc}")
                goal_handle.abort()
                result.success = False
                result.message = f"MoveIt motion failed: {exc}"
                return result

            finally:
                self.busy = False

    def joint_state_callback(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            self.current_joint_positions[name] = position


    def get_current_joint_positions_ordered(self):
        try:
            return [
                self.current_joint_positions[name]
                for name in self.joint_names
            ]
        except KeyError:
            return None


    def angle_difference(self, actual: float, target: float) -> float:
        return math.atan2(
            math.sin(actual - target),
            math.cos(actual - target),
        )


    def joint_goal_reached(self, target_positions) -> bool:
        for joint_name, target_position in zip(self.joint_names, target_positions):
            if joint_name not in self.current_joint_positions:
                return False

            actual_position = self.current_joint_positions[joint_name]
            error = abs(self.angle_difference(actual_position, target_position))

            if error > self.joint_goal_tolerance:
                return False

        return True


    def robot_is_stationary(
        self,
        duration_sec: float = 0.3,
        tol_rad: float = 0.005,
    ) -> bool:
        start = time.time()
        previous = self.get_current_joint_positions_ordered()

        if previous is None:
            return False

        while time.time() - start < duration_sec:
            time.sleep(0.05)

            current = self.get_current_joint_positions_ordered()

            if current is None:
                return False

            max_delta = max(
                abs(self.angle_difference(c, p))
                for c, p in zip(current, previous)
            )

            if max_delta > tol_rad:
                return False

            previous = current

        return True


    def wait_for_joint_goal(self, target_positions, timeout_sec: float) -> bool:
        start_time = self.get_clock().now()

        while rclpy.ok():
            if self.joint_goal_reached(target_positions):
                if self.robot_is_stationary():
                    return True

            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9

            if elapsed > timeout_sec:
                return False

            time.sleep(0.05)

        return False


def main(args=None) -> None:
    rclpy.init(args=args)

    node = MoveItMotionServerNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.add_node(node.moveit_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.moveit_node.destroy_node()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()