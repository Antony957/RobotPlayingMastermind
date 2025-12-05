#!/usr/bin/env python3
import json

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

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

    # Color to Index Mapping
    COLOR_TO_INDEX = {
        "unknown": 0,
        "red": 1,
        "blue": 2,
        "green": 3,
        "yellow": 4,
        "purple": 5,
        "black": 6,
    }

    ERROR_INDEX = 255

    def __init__(self) -> None:
        super().__init__("vision_node")
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_stamp = None
        self.scan_requested = False

        # --- Parameters ---
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("game_status_topic", "/game_status")
        self.declare_parameter("submit_code_topic", "/submit_code")
        self.declare_parameter("debug_topic", "mastermind/scanned_guess/debug_text")
        self.declare_parameter("player_name", "VisionBot")
        self.declare_parameter("auto_scan_timer", 0.0)

        image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        # game_status_topic = self.get_parameter("game_status_topic").get_parameter_value().string_value
        self.game_status_pub = self.create_publisher(Status, "game_status", 10)

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
            "yellow": ([15, 80, 60], [40, 255, 255]),
            "blue": ([95, 80, 40], [125, 255, 255]),
            "purple": ([120, 20, 40], [150, 255, 255]),
            "red": ([0, 45, 35], [20, 250, 250]),
            "red2": ([160, 45, 35], [190, 255, 255]),
            "green": ([40, 40, 40], [70, 255, 255]),
            "black": ([0, 0, 0], [180, 80, 70]),
        }

        # --- ROS Setup ---
        self.image_subscription = self.create_subscription(
            Image, image_topic, self._image_callback, 10
        )

        # 1. Subscribe to game_status (Custom Message)
        self.status_subscription = self.create_subscription(
            Status, "game_status", self._status_callback, 10
        )

        # 2. Publish Code (Custom Message)
        self.code_pub = self.create_publisher(Code, self.submit_code_topic, 10)

        # self.debug_pub = self.create_publisher(String, self.debug_topic, 10)

        # auto_scan_period = self.get_parameter("auto_scan_timer").get_parameter_value().double_value
        # if auto_scan_period > 0.0:
        #     self.create_timer(auto_scan_period, self._auto_scan)

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
        if msg.status == 3:
            if self.latest_image is not None:
                self.get_logger().info(
                    f"Received Status 3 from '{msg.sender}'. Scanning..."
                )
                self._process_frame()
            else:
                self.get_logger().warn(
                    "Received Status 3, but no image yet. Queuing scan."
                )
                self.scan_requested = True

    # def _auto_scan(self) -> None:
    #     if self.latest_image is not None:
    #         self._process_frame()

    def _process_frame(self) -> None:
        self.get_logger().info("process frame")
        if self.latest_image is None:
            return

        image = self.latest_image.copy()

        try:
            ok = cv2.imwrite("./debug.png", image)
            if not ok:
                self.get_logger().warn("Failed to write debug image to ./debug.png")
        except Exception as e:
            self.get_logger().error(f"Exception while saving debug image: {e}")

        height, width, _ = image.shape

        # --- ROI Cropping  ---
        # TODO: CALLIBRATE FOR FINAL SET UP
        roi_y_start = int(height * 0.5)
        roi_y_end = int(height * 0.7)
        roi_x_start = int(width * 0.15)
        roi_x_end = int(width * 0.95)
        roi_img = image[roi_y_start:roi_y_end, roi_x_start:roi_x_end]

        hsv = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)
        detected_blocks = []

        blur_amt = 17
        hsv = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)
        if blur_amt:
            hsv = cv2.GaussianBlur(hsv, (blur_amt, blur_amt), 0)
        detected_blocks = []

        for color_name, (lower, upper) in self.color_ranges.items():
            if color_name == "red2":
                color_name = "red"

            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))

            mask = cv2.erode(mask, None, iterations=1)
            mask = cv2.dilate(mask, None, iterations=2)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            largest_contour = max(contours, key=cv2.contourArea)
            largest_area = cv2.contourArea(largest_contour)

            if largest_area >= 4000:
                M = cv2.moments(largest_contour)

                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])

                # Append centroid x position for sorting
                detected_blocks.append({"color": color_name, "cx": cx})

            largest_contour = max(contours, key=cv2.contourArea)
            largest_area = cv2.contourArea(largest_contour)

            if largest_area >= 4000:
                M = cv2.moments(largest_contour)

                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])

                # Append centroid x position for sorting
                detected_blocks.append({"color": color_name, "cx": cx})

        # Sort left to right
        detected_blocks.sort(key=lambda b: b["cx"])
        self.get_logger().info(f"DETECTED BLOCKS {detected_blocks}")

        # get colors only
        detected_colors = [k["color"] for k in detected_blocks]

        # --- 3. Strict Validation ---
        count = len(detected_colors)

        # If NOT exactly 4 blocks, return ERROR and do NOT publish Code
        if count != 4:
            self.get_logger().error(
                f"DETECTION ERROR: Found {count} blocks. Expected exactly 4."
            )

            # # Publish debug info about the failure
            # debug_payload = {
            #     "valid": False,
            #     "error": f"Count mismatch: {count} != 4",
            #     "detected_raw": [b[1] for b in detected_blocks]
            # }
            # self.debug_pub.publish(String(data=json.dumps(debug_payload)))
            # return

        result_indices = [self.COLOR_TO_INDEX.get(lbl, 0) for lbl in detected_colors]
        self.get_logger().info(f"Success: {detected_colors} -> {result_indices}")

        # --- Publish Code Message ---
        msg = Code()
        msg.player_name = self.player_name
        msg.code = result_indices

        self.code_pub.publish(msg)

        # Debug
        # debug_payload = {
        #     "labels": result_labels,
        #     "indices": result_indices,
        #     "valid": True
        # }
        # self.get_logger().info(json.dumps(debug_payload))
