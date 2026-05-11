import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_srvs.srv import SetBool
from ur_msgs.srv import SetIO


class OnRobotRG2Node(Node):
    def __init__(self) -> None:
        super().__init__("onrobot_rg2_node")

        self.open_pin = 2
        self.close_pin = 3

        self.io_client = self.create_client(
            SetIO,
            "/io_and_status_controller/set_io",
        )

        self.command_sub = self.create_subscription(
            String,
            "/handover/gripper_command",
            self.command_callback,
            10,
        )

        self.set_closed_srv = self.create_service(
            SetBool,
            "/handover/set_gripper_closed",
            self.set_closed_callback,
        )

        self.state_pub = self.create_publisher(
            String,
            "/handover/gripper_state",
            10,
        )

        self.get_logger().info("OnRobot RG2 node started. OPEN=DO2, CLOSE=DO3")

    def command_callback(self, msg: String) -> None:
        command = msg.data.strip().upper()

        if command in ("OPEN", "RELEASE"):
            self.open_gripper()
        elif command == "CLOSE":
            self.close_gripper()
        else:
            self.get_logger().warn(f"Unknown gripper command: {msg.data}")

    def set_closed_callback(
        self,
        request: SetBool.Request,
        response: SetBool.Response,
    ) -> SetBool.Response:
        if request.data:
            self.close_gripper()
            response.message = "Close command sent."
        else:
            self.open_gripper()
            response.message = "Open command sent."

        response.success = True
        return response

    def open_gripper(self) -> None:
        self.get_logger().info("Opening gripper.")
        self.set_output(self.open_pin)
        self.publish_state("OPEN")

    def close_gripper(self) -> None:
        self.get_logger().info("Closing gripper.")
        self.set_output(self.close_pin)
        self.publish_state("CLOSED")

    def set_output(self, pin: int) -> None:
        if not self.io_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("SetIO service not available.")
            return

        request = SetIO.Request()
        request.fun = SetIO.Request.FUN_SET_DIGITAL_OUT
        request.pin = pin
        request.state = 1.0

        self.io_client.call_async(request)

    def publish_state(self, state: str) -> None:
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OnRobotRG2Node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()