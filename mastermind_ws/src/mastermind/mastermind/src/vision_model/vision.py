#!/usr/bin/env python3
import json

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from ..game_state.game_state import COLOR_TO_NUM

# --- Import Custom Interfaces ---
try:
    from mastermind_interfaces.msg import Code, Status
except ImportError:
    print("CRITICAL ERROR: Could not import 'mastermind_interfaces'.")

    # Dummy classes for syntax check
    class Status:
        pass

    class Code:
        pass


class VisionNode(Node):
    """
    Vision node for Mastermind game.
    Detects 6 possible colors using HSV.
    Strictly requires exactly 4 detected blocks.
    Listens for Status(status=4) -> Publishes Code(player_name, code).
    """

    ERROR_INDEX = 255

    def __init__(self) -> None:
        super().__init__("vision_node")
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_stamp = None
        self.scan_requested = False

        # --- Parameters ---
        self.declare_parameter("image_topic", "/mastermind/camera/image_raw")
        self.declare_parameter("game_status_topic", "/game_status")
        self.declare_parameter("submit_code_topic", "/submit_code")
        self.declare_parameter("debug_topic", "mastermind/scanned_guess/debug_text")
        self.declare_parameter("player_name", "computer_vision")
        self.declare_parameter("auto_scan_timer", 0.0)

        image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        game_status_topic = (
            self.get_parameter("game_status_topic").get_parameter_value().string_value
        )
        self.submit_code_topic = (
            self.get_parameter("submit_code_topic").get_parameter_value().string_value
        )
        self.player_name = (
            self.get_parameter("player_name").get_parameter_value().string_value
        )
        self.debug_topic = (
            self.get_parameter("debug_topic").get_parameter_value().string_value
        )

        # --- HSV Color Ranges ---
        self.color_ranges = {
            "red": ([0, 100, 100], [10, 255, 255]),
            "red2": ([170, 100, 100], [180, 255, 255]),
            "green": ([40, 100, 100], [80, 255, 255]),
            "blue": ([100, 100, 100], [140, 255, 255]),
            "yellow": ([20, 100, 100], [35, 255, 255]),
            "purple": ([140, 100, 100], [160, 255, 255]),
            "black": ([0, 0, 0], [180, 255, 40]),
        }

        # --- ROS Setup ---
        self.image_subscription = self.create_subscription(
            Image, image_topic, self._image_callback, 10
        )

        # 1. Subscribe to game_status (Custom Message)
        self.status_subscription = self.create_subscription(
            Status, game_status_topic, self._status_callback, 10
        )

        # 2. Publish Code (Custom Message)
        self.code_pub = self.create_publisher(Code, self.submit_code_topic, 10)

        self.debug_pub = self.create_publisher(String, self.debug_topic, 10)

        auto_scan_period = (
            self.get_parameter("auto_scan_timer").get_parameter_value().double_value
        )
        if auto_scan_period > 0.0:
            self.create_timer(auto_scan_period, self._auto_scan)

        self.get_logger().info(f"Vision Node ready. Waiting for Status=4.")

    def _image_callback(self, msg: Image) -> None:
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.latest_stamp = msg.header.stamp
        except CvBridgeError as exc:
            self.get_logger().error(f"Failed to convert image: {exc}")
            return

        if self.scan_requested:
            self.scan_requested = False
            self._process_frame()

    def _status_callback(self, msg: Status) -> None:
        """
        Callback for /game_status.
        Wait for status == 4 to trigger scan.
        """
        if msg.status == 4:
            if self.latest_image is not None:
                self.get_logger().info(
                    f"Received Status 4 from '{msg.sender}'. Scanning..."
                )
                self._process_frame()
            else:
                self.get_logger().warn(
                    "Received Status 4, but no image yet. Queuing scan."
                )
                self.scan_requested = True

    def _auto_scan(self) -> None:
        if self.latest_image is not None:
            self._process_frame()

    def _process_frame(self) -> None:
        if self.latest_image is None:
            return

        image = self.latest_image.copy()
        height, width, _ = image.shape

        # --- 1. ROI Cropping (Updated for closer camera) ---
        roi_y_start = int(height * 0.1)
        roi_y_end = int(height * 0.99)
        roi_x_start = int(width * 0.05)
        roi_x_end = int(width * 0.95)
        roi_img = image[roi_y_start:roi_y_end, roi_x_start:roi_x_end]

        hsv = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)
        detected_blocks = []

        # --- 2. Color Detection ---
        for color_name, (lower, upper) in self.color_ranges.items():
            if color_name == "red":
                l2, u2 = self.color_ranges["red2"]
                mask = cv2.inRange(hsv, np.array(lower), np.array(upper)) + cv2.inRange(
                    hsv, np.array(l2), np.array(u2)
                )
            elif color_name == "red2":
                continue
            else:
                mask = cv2.inRange(hsv, np.array(lower), np.array(upper))

            mask = cv2.erode(mask, None, iterations=1)
            mask = cv2.dilate(mask, None, iterations=2)
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            for c in contours:
                # Increased area limit for closer camera view
                if 800 < cv2.contourArea(c) < 150000:
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        detected_blocks.append((cx, color_name))

        # --- 3. Strict Validation ---
        count = len(detected_blocks)

        # If NOT exactly 4 blocks, return ERROR and do NOT publish Code
        if count != 4:
            self.get_logger().error(
                f"DETECTION ERROR: Found {count} blocks. Expected exactly 4."
            )

            # Publish debug info about the failure
            debug_payload = {
                "valid": False,
                "error": f"Count mismatch: {count} != 4",
                "detected_raw": [b[1] for b in detected_blocks],
            }
            self.debug_pub.publish(String(data=json.dumps(debug_payload)))
            return

        # --- 4. Sorting and Output (Only if count == 4) ---
        detected_blocks.sort(key=lambda x: x[0])

        result_labels = [b[1] for b in detected_blocks]
        result_indices = [COLOR_TO_NUM.get(lbl, 0) for lbl in result_labels]

        self.get_logger().info(f"Success: {result_labels} -> {result_indices}")

        # --- Publish Code Message ---
        msg = Code()
        msg.player_name = self.player_name
        msg.code = result_indices

        self.code_pub.publish(msg)

        # Debug
        debug_payload = {
            "labels": result_labels,
            "indices": result_indices,
            "valid": True,
        }
        self.debug_pub.publish(String(data=json.dumps(debug_payload)))


def main() -> None:
    rclpy.init()
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
