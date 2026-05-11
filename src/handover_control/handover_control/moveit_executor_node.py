import math
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from pymoveit2 import MoveIt2


class MoveItExecutorNode(Node):
    def __init__(self) -> None:
        super().__init__("moveit_executor_node")

        self.callback_group = ReentrantCallbackGroup()

        self.current_joint_positions = {}

        self.pose_sub = self.create_subscription(
            PoseStamped,
            "/handover/approach_pose",
            self.pose_callback,
            10,
            callback_group=self.callback_group,
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
            callback_group=self.callback_group,
        )

        self.joint_goal_tolerance = float(
            self.declare_parameter("joint_goal_tolerance", 0.03).value
        )

        self.motion_timeout = float(
            self.declare_parameter("motion_timeout", 20.0).value
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

        self.enable_pose_goals = bool(
            self.declare_parameter("enable_pose_goals", True).value
        )

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

        self.moveit_lock = threading.Lock()

        self.max_velocity_scaling = float(
        self.declare_parameter("max_velocity_scaling", 0.05).value
        )
        self.max_acceleration_scaling = float(
            self.declare_parameter("max_acceleration_scaling", 0.05).value
        )

        self.planner_id = str(
            self.declare_parameter("planner_id", "RRTConnectkConfigDefault").value
        )

        self.moveit2.planner_id = self.planner_id

        self.get_logger().info(
            f"Using MoveIt planner={self.planner_id} with default planning pipeline"
        )

        self.get_logger().info(
            "Motion scaling: "
            f"max_velocity={self.max_velocity_scaling}, "
            f"max_acceleration={self.max_acceleration_scaling}"
        )

        self.named_joint_poses = {
            "start": [
                math.radians(-28.86),    # shoulder_pan_joint / Base
                math.radians(-136.76),   # shoulder_lift_joint / Shoulder
                math.radians(-55.97),    # elbow_joint
                math.radians(-167.05),   # wrist_1_joint
                math.radians(146.47),    # wrist_2_joint
                math.radians(-1.65),     # wrist_3_joint
            ],
            "ready": [
                math.radians(-124),     # shoulder_pan_joint / Base
                math.radians(-129),     # shoulder_lift_joint / Shoulder
                math.radians(-80),      # elbow_joint
                math.radians(-148),     # wrist_1_joint
                math.radians(146),      # wrist_2_joint
                math.radians(-86),      # wrist_3_joint
            ],
        }

        self.start_pose_srv = self.create_service(
            Trigger,
            "/handover/go_to_start_pose",
            self.go_to_start_pose_callback,
            callback_group=self.callback_group,
        )

        self.ready_pose_srv = self.create_service(
            Trigger,
            "/handover/go_to_ready_pose",
            self.go_to_ready_pose_callback,
            callback_group=self.callback_group,
        )

        self.get_logger().info(
            f"MoveIt executor started. group={self.group_name}, ee={self.end_effector_name}"
        )

        self.approach_pose_consumed = False

    def zone_callback(self, msg: Bool) -> None:
        previous = self.hand_in_zone
        self.hand_in_zone = msg.data

        if previous and not self.hand_in_zone:
            self.approach_pose_consumed = False
            self.last_executed_pose = None
            self.get_logger().info("Hand left zone; approach trigger reset.")

    def pose_callback(self, msg: PoseStamped) -> None:
        if not self.enable_pose_goals:
            return

        if not self.hand_in_zone:
            return
        
        if self.approach_pose_consumed:
            return  
        
        if self.executing:
            self.get_logger().warn(
                "Ignoring approach pose: robot is already executing.",
                throttle_duration_sec=2.0,
            )
            return

        if self.last_executed_pose is not None:
            dx = msg.pose.position.x - self.last_executed_pose.pose.position.x
            dy = msg.pose.position.y - self.last_executed_pose.pose.position.y
            dz = msg.pose.position.z - self.last_executed_pose.pose.position.z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if dist < self.min_replan_distance:
                return

        self.approach_pose_consumed = True
        self.executing = True
        threading.Thread(
            target=self.move_to_pose_thread,
            args=(msg,),
            daemon=True,
        ).start()

    def pose_inside_handover_workspace(self, msg: PoseStamped) -> bool:
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        # Tune these for your actual setup.
        min_x = -0.85
        max_x = -0.55

        min_y = -0.5
        max_y = 0.00

        min_z = 0.20
        max_z = 0.35

        if not (min_x <= x <= max_x):
            self.get_logger().warn(f"Rejecting approach pose: x={x:.3f} outside [{min_x}, {max_x}]")
            return False

        if not (min_y <= y <= max_y):
            self.get_logger().warn(f"Rejecting approach pose: y={y:.3f} outside [{min_y}, {max_y}]")
            return False

        if not (min_z <= z <= max_z):
            self.get_logger().warn(f"Rejecting approach pose: z={z:.3f} outside [{min_z}, {max_z}]")
            return False

        return True
    
    def robot_is_stationary(self, duration_sec: float = 0.25, tol_rad: float = 0.01) -> bool:
        start = time.time()
        prev = self.get_current_joint_positions_ordered()
        if prev is None:
            return False

        while time.time() - start < duration_sec:
            time.sleep(0.05)
            curr = self.get_current_joint_positions_ordered()
            if curr is None:
                return False

            max_delta = max(
                abs(self.angle_difference(c, p))
                for c, p in zip(curr, prev)
            )
            if max_delta > tol_rad:
                return False

            prev = curr

        return True

    def apply_motion_scaling(self) -> None:
        """
        Apply MoveIt velocity/acceleration scaling.

        These values are fractions of the configured MoveIt limits:
        1.0 = full speed, 0.05 = 5% speed.
        """
        self.moveit2.max_velocity = self.max_velocity_scaling
        self.moveit2.max_acceleration = self.max_acceleration_scaling

    def move_to_pose_thread(self, msg: PoseStamped) -> None:
        try:
            if not self.pose_inside_handover_workspace(msg):
                self.approach_pose_consumed = False
                return

            if not self.wait_for_joint_state():
                self.approach_pose_consumed = False
                return
        
            if not self.robot_is_stationary():
                self.get_logger().warn("Rejecting approach pose: robot is not stationary yet.")
                self.approach_pose_consumed = False
                return

            time.sleep(0.15)

            if not self.hand_in_zone:
                self.get_logger().info(
                    "Aborting approach: hand left zone before execution."
                )
                self.approach_pose_consumed = False
                return
            
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

            with self.moveit_lock:
                self.moveit2.planner_id = self.planner_id
                self.apply_motion_scaling()

                if self.cartesian:
                    self.moveit2.move_to_pose(
                        position=position,
                        quat_xyzw=quat_xyzw,
                        cartesian=True,
                        cartesian_max_step=0.005,
                        cartesian_fraction_threshold=0.95
                    )
                else:
                    self.moveit2.move_to_pose(
                        position=position,
                        quat_xyzw=quat_xyzw,
                        cartesian=False,
                    )

            # Temporary: do not call wait_until_executed(), it is crashing rclpy.
            time.sleep(self.motion_timeout)

            self.last_executed_pose = msg

        except Exception as exc:
            self.get_logger().error(f"MoveIt execution failed: {exc}")
        finally:
            self.executing = False

    def go_to_start_pose_callback(self, request, response):
        return self.go_to_pose("start", response)

    def go_to_ready_pose_callback(self, request, response):
        return self.go_to_pose("ready", response)

    def go_to_pose(self, pose_name: str, response):
        if self.executing:
            self.get_logger().warn(
                f"Rejected request for '{pose_name}': robot is already executing."
            )
            response.success = False
            response.message = "Robot is already executing a motion."
            return response

        if pose_name not in self.named_joint_poses:
            response.success = False
            response.message = f"Unknown named joint pose: {pose_name}"
            return response

        self.executing = True

        try:
            if not self.wait_for_joint_state():
                response.success = False
                response.message = "No current joint state available; refusing named pose motion."
                return response

            raw_joint_positions = self.named_joint_poses[pose_name]
            joint_positions = self.unwrap_joint_goal_to_current(raw_joint_positions)

            joint_positions_deg = [round(math.degrees(j), 2) for j in joint_positions]
            self.get_logger().info(
                f"Moving to named joint pose '{pose_name}' deg: {joint_positions_deg}"
            )

            with self.moveit_lock:
                self.moveit2.planner_id = self.planner_id
                self.apply_motion_scaling()

                self.moveit2.move_to_configuration(
                    joint_positions=joint_positions,
                )

            self.get_logger().info(
                f"Motion command sent for named pose '{pose_name}'. "
                "Monitoring actual joint state..."
            )

            start_time = self.get_clock().now()

            while rclpy.ok():
                if self.joint_goal_reached(joint_positions):
                    if self.robot_is_stationary(duration_sec=0.3, tol_rad=0.005):
                        response.success = True
                        response.message = f"Reached named joint pose: {pose_name}"
                        return response

                elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
                if elapsed > self.motion_timeout:
                    response.success = False
                    response.message = (
                        f"Timed out waiting for robot to reach named pose: {pose_name}"
                    )
                    return response

                time.sleep(0.05)

            response.success = False
            response.message = "ROS shutdown while waiting for motion."

        except Exception as exc:
            self.get_logger().error(f"MoveIt named pose execution failed: {exc}")
            response.success = False
            response.message = f"MoveIt named pose execution failed: {exc}"

        finally:
            self.executing = False
            self.get_logger().info("Motion executor is ready for next command.")

        return response

    def joint_state_callback(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            self.current_joint_positions[name] = position

    def nearest_equivalent_angle(self, target: float, current: float) -> float:
        """
        Shift target by multiples of 2*pi so that it is closest to current.
        This prevents unnecessary full revolutions on continuous/wrapped joints.
        """
        return current + math.atan2(
            math.sin(target - current),
            math.cos(target - current),
        )


    def get_current_joint_positions_ordered(self):
        try:
            return [self.current_joint_positions[name] for name in self.joint_names]
        except KeyError:
            return None


    def unwrap_joint_goal_to_current(self, target_positions):
        current_positions = self.get_current_joint_positions_ordered()

        if current_positions is None:
            self.get_logger().warn(
                "Current joint positions unavailable; using raw named pose target."
            )
            return target_positions

        return [
            self.nearest_equivalent_angle(target, current)
            for target, current in zip(target_positions, current_positions)
        ]


    def joint_goal_reached(self, target_positions) -> bool:
        for joint_name, target_position in zip(self.joint_names, target_positions):
            if joint_name not in self.current_joint_positions:
                return False

            actual_position = self.current_joint_positions[joint_name]
            error = abs(self.angle_difference(actual_position, target_position))

            if error > self.joint_goal_tolerance:
                return False

        return True
        
    def angle_difference(self, actual: float, target: float) -> float:
        return math.atan2(
            math.sin(actual - target),
            math.cos(actual - target),
        )
    
    def wait_for_joint_state(self, timeout_sec: float = 3.0) -> bool:
        start_time = self.get_clock().now()

        while rclpy.ok():
            if self.get_current_joint_positions_ordered() is not None:
                return True

            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > timeout_sec:
                return False

            time.sleep(0.05)

        return False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MoveItExecutorNode()

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