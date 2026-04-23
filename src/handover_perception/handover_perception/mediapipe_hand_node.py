import rclpy
import numpy as np
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge

import cv2
import mediapipe as mp


# MediaPipe hand landmark connections (same hand skeleton topology)
HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),          # thumb
    (0, 5), (5, 6), (6, 7), (7, 8),          # index
    (5, 9), (9, 10), (10, 11), (11, 12),     # middle
    (9, 13), (13, 14), (14, 15), (15, 16),   # ring
    (13, 17), (17, 18), (18, 19), (19, 20),  # pinky
    (0, 17)                                  # palm edge
]


class MediaPipeHandNode(Node):
    def __init__(self) -> None:
        super().__init__("mediapipe_hand_node")

        self.bridge = CvBridge()

        self.rgb_subscription = self.create_subscription(
            Image,
            "/vzense/rgb/image_raw",
            self.image_callback,
            10,
        )

        self.debug_pub = self.create_publisher(
            Image,
            "/handover/rgb_hand_annotated",
            10,
        )

        self.palm_pub_2d = self.create_publisher(
            PointStamped,
            "/handover/palm_center_2d",
            10,
        )

        self.depth_subscription = self.create_subscription(
            Image,
            "/vzense/depth_registered/image_raw",
            self.depth_callback,
            10,
        )

        self.palm_pub_3d = self.create_publisher(
            PointStamped,
            "/handover/palm_center_3d",
            10,
        )

        self.latest_depth = None
        self.latest_depth_header = None

        # RGB intrinsics from SDK
        self.fx = 532.654
        self.fy = 535.208
        self.cx = 310.849
        self.cy = 184.457

        model_path = str(self.declare_parameter(
            "model_path",
            "/home/conall/fyp_ws/models/hand_landmarker.task"
        ).value)

        BaseOptions = mp.tasks.BaseOptions
        HandLandmarker = mp.tasks.vision.HandLandmarker
        HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
        VisionRunningMode = mp.tasks.vision.RunningMode

        options = HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=model_path),
            running_mode=VisionRunningMode.IMAGE,
            num_hands=2,
            min_hand_detection_confidence=0.3,
            min_hand_presence_confidence=0.3,
            min_tracking_confidence=0.3,
        )

        self.landmarker = HandLandmarker.create_from_options(options)

        self.get_logger().info(
            f"MediaPipe Hand Landmarker started with model: {model_path}"
        )

    def image_callback(self, msg: Image) -> None:
        try:
            # DCAM710 node publishes in bgr8, noted from cam's SDK
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"cv_bridge conversion failed: {exc}")
            return

        # Convert to RGB for MediaPipe
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame_rgb)
        result = self.landmarker.detect(mp_image)

        annotated_bgr = frame_bgr.copy()
        hand_count = len(result.hand_landmarks) if result.hand_landmarks else 0

        if result.hand_landmarks:
            h, w, _ = annotated_bgr.shape

            for hand_landmarks in result.hand_landmarks:
                points = []
                for landmark in hand_landmarks:
                    cx = int(landmark.x * w)
                    cy = int(landmark.y * h)
                    points.append((cx, cy))

                # draw skeleton lines first
                for start_idx, end_idx in HAND_CONNECTIONS:
                    x1, y1 = points[start_idx]
                    x2, y2 = points[end_idx]
                    cv2.line(annotated_bgr, (x1, y1), (x2, y2), (0, 255, 255), 2)

                # draw landmark circles
                for i, (cx, cy) in enumerate(points):
                    cv2.circle(annotated_bgr, (cx, cy), 4, (0, 255, 0), -1)
                    cv2.putText(
                        annotated_bgr,
                        str(i),
                        (cx + 4, cy - 4),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.35,
                        (255, 255, 255),
                        1,
                        cv2.LINE_AA,
                    )

                # simple palm center estimate:
                # average wrist + MCP joints of index/middle/ring/pinky
                palm_indices = [0, 5, 9, 13, 17]
                palm_x = int(sum(points[i][0] for i in palm_indices) / len(palm_indices))
                palm_y = int(sum(points[i][1] for i in palm_indices) / len(palm_indices))

                cv2.circle(annotated_bgr, (palm_x, palm_y), 8, (0, 0, 255), -1)
                cv2.putText(
                    annotated_bgr,
                    "palm",
                    (palm_x + 8, palm_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )

                palm_msg = PointStamped()
                palm_msg.header = msg.header
                palm_msg.point.x = float(palm_x)
                palm_msg.point.y = float(palm_y)
                palm_msg.point.z = 0.0
                self.palm_pub_2d.publish(palm_msg)

                depth_m = self.sample_depth_m(palm_x, palm_y)

                if depth_m is not None:
                    x, y, z = self.pixel_to_3d(palm_x, palm_y, depth_m)

                    palm_3d_msg = PointStamped()
                    palm_3d_msg.header = msg.header
                    palm_3d_msg.point.x = float(x)
                    palm_3d_msg.point.y = float(y)
                    palm_3d_msg.point.z = float(z)
                    self.palm_pub_3d.publish(palm_3d_msg)

                    cv2.putText(
                        annotated_bgr,
                        f"Z={z:.2f}m",
                        (palm_x + 8, palm_y + 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 0, 255),
                        2,
                        cv2.LINE_AA,
                    )

        self.get_logger().info(
            f"Detected hands: {hand_count}",
            throttle_duration_sec=2.0,
        )

        try:
            out_msg = self.bridge.cv2_to_imgmsg(annotated_bgr, encoding="bgr8")
            out_msg.header = msg.header
            self.debug_pub.publish(out_msg)
        except Exception as exc:
            self.get_logger().error(f"cv_bridge publish conversion failed: {exc}")

    def depth_callback(self, msg: Image) -> None:
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().error(f"depth conversion failed: {exc}")
            return

        self.latest_depth = depth
        self.latest_depth_header = msg.header

    def sample_depth_m(self, u: int, v: int):
        if self.latest_depth is None:
            return None

        h, w = self.latest_depth.shape[:2]
        if u < 0 or v < 0 or u >= w or v >= h:
            return None

        half = 2
        u0 = max(0, u - half)
        u1 = min(w, u + half + 1)
        v0 = max(0, v - half)
        v1 = min(h, v + half + 1)

        patch = self.latest_depth[v0:v1, u0:u1]

        # assume uint16 depth in mm
        valid = patch[(patch > 0) & (patch < 10000)]
        if valid.size == 0:
            return None

        depth_mm = float(np.median(valid))
        return depth_mm / 1000.0


    def pixel_to_3d(self, u: int, v: int, z: float):
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        return x, y, z


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MediaPipeHandNode()
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