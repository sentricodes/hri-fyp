import math
from enum import Enum, auto

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, WrenchStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import String
from ur_msgs.srv import SetIO

from handover_interfaces.action import MoveToTarget


class HandoverState(Enum):
    STARTUP = auto()
    GOING_TO_START = auto()

    WAITING_FOR_HAND_R2H = auto()
    GOING_TO_R2H_POSE = auto()
    WAITING_FOR_PULL = auto()
    RELEASING_OBJECT = auto()
    RETURNING_TO_START_AFTER_R2H = auto()

    WAITING_FOR_HAND_H2R = auto()
    GOING_TO_H2R_POSE = auto()
    WAITING_FOR_OBJECT_INSERTION = auto()
    CLOSING_GRIPPER = auto()
    CONFIRMING_GRASP = auto()
    RETURNING_TO_START_AFTER_H2R = auto()

    ERROR = auto()


class HandoverStateNode(Node):
    def __init__(self) -> None:
        super().__init__("handover_state_node")

        self.callback_group = ReentrantCallbackGroup()
        self.state = HandoverState.STARTUP

        self.hand_in_zone = False
        self.motion_goal_active = False
        self.current_joint_positions = {}

        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        self.start_joint_target = [
            math.radians(-28.86),
            math.radians(-136.76),
            math.radians(-55.97),
            math.radians(-167.05),
            math.radians(146.47),
            math.radians(-1.65),
        ]

        self.r2h_joint_target = [
            math.radians(20.57),
            math.radians(-100.95),
            math.radians(-97.73),
            math.radians(-155.32),
            math.radians(200.38),
            math.radians(4.51),
        ]

        self.h2r_joint_target = [
            math.radians(20.57),
            math.radians(-100.95),
            math.radians(-97.73),
            math.radians(-155.32),
            math.radians(200.38),
            math.radians(4.51),
        ]

        self.go_to_start_on_startup = bool(
            self.declare_parameter("go_to_start_on_startup", True).value
        )

        self.start_joint_tolerance = float(
            self.declare_parameter("start_joint_tolerance", 0.05).value
        )

        self.velocity_scaling = float(
            self.declare_parameter("velocity_scaling", 0.1).value
        )

        self.acceleration_scaling = float(
            self.declare_parameter("acceleration_scaling", 0.1).value
        )

        self.motion_timeout = float(
            self.declare_parameter("motion_timeout", 20.0).value
        )

        self.start_velocity_scaling = float(
            self.declare_parameter("start_velocity_scaling", 0.1).value
        )

        self.start_acceleration_scaling = float(
            self.declare_parameter("start_acceleration_scaling", 0.1).value
        )

        self.latest_force_magnitude = None
        self.force_baseline = None
        self.force_delta_threshold = 5.0
        self.force_required_samples = 8
        self.force_trigger_count = 0

        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
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

        self.wrench_sub = self.create_subscription(
            WrenchStamped,
            "/force_torque_sensor_broadcaster/wrench",
            self.wrench_callback,
            10,
            callback_group=self.callback_group,
        )

        self.motion_client = ActionClient(
            self,
            MoveToTarget,
            "/motion/move_to_target",
            callback_group=self.callback_group,
        )

        self.startup_timer = self.create_timer(
            0.5,
            self.startup_timer_callback,
            callback_group=self.callback_group,
        )

        self.get_logger().info("Handover state node started. Current state: STARTUP.")

        self.set_io_client = self.create_client(
            SetIO,
            "/io_and_status_controller/set_io",
            callback_group=self.callback_group,
        )

        self.gripper_open_pin = 2
        self.gripper_close_pin = 3

        self.gripper_settle_sec = float(
            self.declare_parameter("gripper_settle_sec", 1.5).value
        )

        self.gripper_wait_timer = None
        self.gripper_settle_sec = 3.0

        self.state_pub = self.create_publisher(
            String,
            "/handover/state",
            10,
        )

    def set_state(self, new_state: HandoverState) -> None:
        if self.state == new_state:
            return

        old_state = self.state
        self.get_logger().info(
            f"State transition: {old_state.name} -> {new_state.name}"
        )
        self.state = new_state

        msg = String()
        msg.data = new_state.name
        self.state_pub.publish(msg)

    def startup_timer_callback(self) -> None:
        if self.state != HandoverState.STARTUP:
            return

        if not self.go_to_start_on_startup:
            self.set_state(HandoverState.WAITING_FOR_HAND_R2H)
            self.startup_timer.cancel()
            return

        if self.get_current_joint_positions_ordered() is None:
            self.get_logger().info(
                "Waiting for joint states before startup check.",
                throttle_duration_sec=2.0,
            )
            return

        if self.robot_at_joint_target(
            self.start_joint_target,
            self.start_joint_tolerance,
        ):
            self.get_logger().info("Robot is already at start pose.")
            self.set_state(HandoverState.WAITING_FOR_HAND_R2H)
            self.startup_timer.cancel()
            return

        if not self.motion_client.server_is_ready():
            self.get_logger().info(
                "Waiting for /motion/move_to_target action server.",
                throttle_duration_sec=2.0,
            )
            return

        self.get_logger().info("Robot is not at start pose. Sending start motion.")
        self.send_start_motion_goal()
        self.startup_timer.cancel()

    def send_joint_motion_goal(
        self,
        joint_target,
        active_state: HandoverState,
        result_callback,
        velocity_scaling=None,
        acceleration_scaling=None,
    ) -> None:
        if not self.motion_client.server_is_ready():
            self.get_logger().warn("Motion action server is not available.")
            return

        goal = MoveToTarget.Goal()
        goal.target_type = MoveToTarget.Goal.TARGET_JOINTS
        goal.pose_target = PoseStamped()
        goal.joint_target = joint_target

        goal.cartesian = False
        goal.velocity_scaling = velocity_scaling or self.velocity_scaling
        goal.acceleration_scaling = acceleration_scaling or self.acceleration_scaling
        goal.timeout_sec = self.motion_timeout

        self.motion_goal_active = True
        self.set_state(active_state)

        send_goal_future = self.motion_client.send_goal_async(
            goal,
            feedback_callback=self.motion_feedback_callback,
        )

        send_goal_future.add_done_callback(
            lambda future: self.joint_goal_response_callback(
                future,
                result_callback,
            )
        )

    def joint_goal_response_callback(self, future, result_callback) -> None:
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("Joint motion goal was rejected.")
            self.motion_goal_active = False
            self.set_state(HandoverState.ERROR)
            return

        self.get_logger().info("Joint motion goal accepted.")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(result_callback)

    def send_start_motion_goal(self) -> None:
        self.send_joint_motion_goal(
            self.start_joint_target,
            HandoverState.GOING_TO_START,
            self.start_motion_result_callback,
            velocity_scaling=self.start_velocity_scaling,
            acceleration_scaling=self.start_acceleration_scaling,
        )

    def send_r2h_motion_goal(self) -> None:
        self.send_joint_motion_goal(
            self.r2h_joint_target,
            HandoverState.GOING_TO_R2H_POSE,
            self.r2h_result_callback,
        )

    def send_h2r_motion_goal(self) -> None:
        self.send_joint_motion_goal(
            self.h2r_joint_target,
            HandoverState.GOING_TO_H2R_POSE,
            self.h2r_result_callback,
        )

    def return_to_start_after_r2h(self) -> None:
        self.send_joint_motion_goal(
            self.start_joint_target,
            HandoverState.RETURNING_TO_START_AFTER_R2H,
            self.return_after_r2h_result_callback,
            velocity_scaling=self.start_velocity_scaling,
            acceleration_scaling=self.start_acceleration_scaling,
        )

    def return_to_start_after_h2r(self) -> None:
        self.send_joint_motion_goal(
            self.start_joint_target,
            HandoverState.RETURNING_TO_START_AFTER_H2R,
            self.return_after_h2r_result_callback,
            velocity_scaling=self.start_velocity_scaling,
            acceleration_scaling=self.start_acceleration_scaling,
        )

    def start_motion_result_callback(self, future) -> None:
        wrapped_result = future.result()
        result = wrapped_result.result

        self.motion_goal_active = False

        if result.success:
            self.get_logger().info(f"Start motion succeeded: {result.message}")
            self.set_state(HandoverState.WAITING_FOR_HAND_R2H)
        else:
            self.get_logger().warn(f"Start motion failed: {result.message}")
            self.set_state(HandoverState.ERROR)

    def r2h_result_callback(self, future) -> None:
        wrapped_result = future.result()
        result = wrapped_result.result

        self.motion_goal_active = False

        if result.success:
            if self.capture_force_baseline():
                self.set_state(HandoverState.WAITING_FOR_PULL)
        else:
            self.get_logger().warn(f"R2H motion failed: {result.message}")
            self.set_state(HandoverState.ERROR)

    def h2r_result_callback(self, future) -> None:
        wrapped_result = future.result()
        result = wrapped_result.result

        self.motion_goal_active = False

        if result.success:
            if self.capture_force_baseline():
                self.set_state(HandoverState.WAITING_FOR_OBJECT_INSERTION)
        else:
            self.get_logger().warn(f"H2R motion failed: {result.message}")
            self.set_state(HandoverState.ERROR)

    def return_after_r2h_result_callback(self, future) -> None:
        wrapped_result = future.result()
        result = wrapped_result.result

        self.motion_goal_active = False

        if result.success:
            self.get_logger().info("Returned to start after R2H.")
            self.set_state(HandoverState.WAITING_FOR_HAND_H2R)
        else:
            self.get_logger().warn(f"Return after R2H failed: {result.message}")
            self.set_state(HandoverState.ERROR)

    def return_after_h2r_result_callback(self, future) -> None:
        wrapped_result = future.result()
        result = wrapped_result.result

        self.motion_goal_active = False

        if result.success:
            self.get_logger().info("Returned to start after H2R.")
            self.set_state(HandoverState.WAITING_FOR_HAND_R2H)
        else:
            self.get_logger().warn(f"Return after H2R failed: {result.message}")
            self.set_state(HandoverState.ERROR)

    def zone_callback(self, msg: Bool) -> None:
        previous = self.hand_in_zone
        self.hand_in_zone = msg.data

        if self.state in [
            HandoverState.STARTUP,
            HandoverState.GOING_TO_START,
            HandoverState.GOING_TO_R2H_POSE,
            HandoverState.GOING_TO_H2R_POSE,
            HandoverState.RETURNING_TO_START_AFTER_R2H,
            HandoverState.RETURNING_TO_START_AFTER_H2R,
            HandoverState.WAITING_FOR_PULL,
            HandoverState.WAITING_FOR_OBJECT_INSERTION,
            HandoverState.RELEASING_OBJECT,
            HandoverState.CLOSING_GRIPPER,
            HandoverState.CONFIRMING_GRASP,
            HandoverState.ERROR,
        ]:
            return

        if self.hand_in_zone and not previous:
            self.get_logger().info("Hand entered handover zone.")

            if self.state == HandoverState.WAITING_FOR_HAND_R2H:
                self.send_r2h_motion_goal()

            elif self.state == HandoverState.WAITING_FOR_HAND_H2R:
                self.send_h2r_motion_goal()

        if previous and not self.hand_in_zone:
            self.get_logger().info("Hand left handover zone.")

    def wrench_callback(self, msg: WrenchStamped) -> None:
        force = msg.wrench.force
        force_mag = math.sqrt(force.x**2 + force.y**2 + force.z**2)

        self.latest_force_magnitude = force_mag

        if self.force_baseline is None:
            return

        delta = abs(force_mag - self.force_baseline)

        if self.state == HandoverState.WAITING_FOR_PULL:
            if delta > self.force_delta_threshold:
                self.force_trigger_count += 1
            else:
                self.force_trigger_count = 0

            if self.force_trigger_count >= self.force_required_samples:
                self.get_logger().info("Consistent pull detected. Releasing object.")
                self.force_trigger_count = 0
                self.force_baseline = None
                self.set_state(HandoverState.RELEASING_OBJECT)
                self.open_gripper()

        elif self.state == HandoverState.WAITING_FOR_OBJECT_INSERTION:
            if delta > self.force_delta_threshold:
                self.force_trigger_count += 1
            else:
                self.force_trigger_count = 0

            if self.force_trigger_count >= self.force_required_samples:
                self.get_logger().info(
                    "Object insertion/contact detected. Closing gripper."
                )
                self.force_trigger_count = 0
                self.force_baseline = None
                self.set_state(HandoverState.CLOSING_GRIPPER)
                self.close_gripper()

    def capture_force_baseline(self) -> bool:
        if self.latest_force_magnitude is None:
            self.get_logger().warn(
                "Cannot set force baseline: no wrench data received yet."
            )
            self.set_state(HandoverState.ERROR)
            return False

        self.force_baseline = self.latest_force_magnitude
        self.force_trigger_count = 0

        self.get_logger().info(
            f"Force baseline set to {self.force_baseline:.2f} N"
        )

        return True

    def call_later_once(self, delay_sec: float, callback) -> None:
        if self.gripper_wait_timer is not None:
            self.gripper_wait_timer.cancel()
            self.destroy_timer(self.gripper_wait_timer)
            self.gripper_wait_timer = None

        def wrapped_callback():
            self.get_logger().info("Gripper wait complete. Continuing state machine.")

            if self.gripper_wait_timer is not None:
                self.gripper_wait_timer.cancel()
                self.destroy_timer(self.gripper_wait_timer)
                self.gripper_wait_timer = None

            callback()

        self.gripper_wait_timer = self.create_timer(
            delay_sec,
            wrapped_callback,
            callback_group=self.callback_group,
        )


    def trigger_gripper_output(self, pin: int, after_done) -> None:
        if not self.set_io_client.service_is_ready():
            self.get_logger().warn("/io_and_status_controller/set_io is not available.")
            self.set_state(HandoverState.ERROR)
            return

        request = SetIO.Request()
        request.fun = 1
        request.pin = pin
        request.state = 1.0

        future = self.set_io_client.call_async(request)

        def done_callback(future):
            try:
                response = future.result()
            except Exception as exc:
                self.get_logger().warn(f"Failed to set DO{pin}: {exc}")
                self.set_state(HandoverState.ERROR)
                return

            if not response.success:
                self.get_logger().warn(f"SetIO for DO{pin} returned success=False.")
                self.set_state(HandoverState.ERROR)
                return

            self.get_logger().info(f"Triggered DO{pin}. Waiting for gripper motion.")
            self.call_later_once(self.gripper_settle_sec, after_done)

        future.add_done_callback(done_callback)


    def open_gripper(self) -> None:
        self.get_logger().info("Opening gripper using DO2.")
        self.trigger_gripper_output(
            self.gripper_open_pin,
            self.return_to_start_after_r2h,
        )


    def close_gripper(self) -> None:
        self.get_logger().info("Closing gripper using DO3.")
        self.trigger_gripper_output(
            self.gripper_close_pin,
            self.after_gripper_closed,
        )


    def after_gripper_closed(self) -> None:
        self.set_state(HandoverState.CONFIRMING_GRASP)
        self.return_to_start_after_h2r()


    def motion_feedback_callback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback

        self.get_logger().info(
            f"Motion feedback: {feedback.state}",
            throttle_duration_sec=2.0,
        )

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

    def robot_at_joint_target(self, target_positions, tolerance: float) -> bool:
        current_positions = self.get_current_joint_positions_ordered()

        if current_positions is None:
            return False

        for actual, target in zip(current_positions, target_positions):
            error = abs(self.angle_difference(actual, target))

            if error > tolerance:
                return False

        return True

    def angle_difference(self, actual: float, target: float) -> float:
        return math.atan2(
            math.sin(actual - target),
            math.cos(actual - target),
        )


def main(args=None) -> None:
    rclpy.init(args=args)

    node = HandoverStateNode()

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