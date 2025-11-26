#!/usr/bin/env python3
import json
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


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
        self.declare_parameter("image_topic", "/mastermind/camera/image_raw")
        self.declare_parameter("game_status_topic", "/game_status")
        self.declare_parameter("submit_code_topic", "/submit_code")
        self.declare_parameter("debug_topic", "mastermind/scanned_guess/debug_text")
        self.declare_parameter("player_name", "VisionBot") 
        self.declare_parameter("auto_scan_timer", 0.0) 

        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.player_name = self.get_parameter("player_name").get_parameter_value().string_value
        self.debug_topic = self.get_parameter("debug_topic").get_parameter_value().string_value

        # --- HSV Color Ranges ---
        self.color_ranges = {
            'red':    ([0, 100, 100], [10, 255, 255]),
            'red2':   ([170, 100, 100], [180, 255, 255]),
            'green':  ([40, 100, 100], [80, 255, 255]),
            'blue':   ([100, 100, 100], [140, 255, 255]),
            'yellow': ([20, 100, 100], [35, 255, 255]),
            'purple': ([140, 100, 100], [160, 255, 255]),
            'black':  ([0, 0, 0], [180, 255, 40])
        }

        # --- ROS Setup ---
        self.image_subscription = self.create_subscription(
            Image, image_topic, self._image_callback, 10
        )
        
        
        # 2. Publish Code (Custom Message)
        
        self.debug_pub = self.create_publisher(String, self.debug_topic, 10)

        auto_scan_period = self.get_parameter("auto_scan_timer").get_parameter_value().double_value
        if auto_scan_period > 0.0:
            self.create_timer(auto_scan_period, self._auto_scan)

        self.get_logger().info(f"Vision Node ready. Waiting for Status=4.")

        #if self.latest_image is not None:
        self.get_logger().info("Received Status 4. Scanning...")
        self._process_frame()

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

    # def _status_callback(self, msg: Status) -> None:
    #     """
    #     Callback for /game_status.
    #     Wait for status == 4 to trigger scan.
    #     """
    #     if msg.status == 4:
    #         if self.latest_image is not None:
    #             self.get_logger().info(f"Received Status 4 from '{msg.sender}'. Scanning...")
    #             self._process_frame()
    #         else:
    #             self.get_logger().warn("Received Status 4, but no image yet. Queuing scan.")
    #             self.scan_requested = True

    def _auto_scan(self) -> None:
        if self.latest_image is not None:
            self._process_frame()

    def _process_frame(self) -> None:
        if self.latest_image is None: return

        image = self.latest_image.copy()
        height, width, _ = image.shape

        # --- 1. ROI Cropping (Updated for closer camera) ---
        roi_y_start = int(height * 0.1) 
        roi_y_end   = int(height * 0.99)
        roi_x_start = int(width * 0.05)
        roi_x_end   = int(width * 0.95)
        roi_img = image[roi_y_start:roi_y_end, roi_x_start:roi_x_end]
        
        hsv = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)
        detected_blocks = [] 

        # --- 2. Color Detection ---
        for color_name, (lower, upper) in self.color_ranges.items():
            if color_name == 'red':
                l2, u2 = self.color_ranges['red2']
                mask = cv2.inRange(hsv, np.array(lower), np.array(upper)) + \
                       cv2.inRange(hsv, np.array(l2), np.array(u2))
            elif color_name == 'red2':
                continue
            else:
                mask = cv2.inRange(hsv, np.array(lower), np.array(upper))

            mask = cv2.erode(mask, None, iterations=1)
            mask = cv2.dilate(mask, None, iterations=2)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

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
            self.get_logger().error(f"DETECTION ERROR: Found {count} blocks. Expected exactly 4.")
            
            # Publish debug info about the failure
            debug_payload = {
                "valid": False,
                "error": f"Count mismatch: {count} != 4",
                "detected_raw": [b[1] for b in detected_blocks]
            }
            self.debug_pub.publish(String(data=json.dumps(debug_payload)))
            return

        # --- 4. Sorting and Output (Only if count == 4) ---
        detected_blocks.sort(key=lambda x: x[0])
        
        result_labels = [b[1] for b in detected_blocks]
        result_indices = [self.COLOR_TO_INDEX.get(lbl, 0) for lbl in result_labels]

        self.get_logger().info(f"Success: {result_labels} -> {result_indices}")

        # Debug
        debug_payload = {
            "labels": result_labels,
            "indices": result_indices,
            "valid": True
        }
        self.debug_pub.publish(String(data=json.dumps(debug_payload)))

def main() -> None:
    rclpy.init()
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node

# from sensor_msgs.msg import Image
# from std_msgs.msg import String

# from cv_bridge import CvBridge, CvBridgeError
# import cv2
# import numpy as np


# class BlockColorScanner(Node):
#     """
#     Subscribe to an overhead camera image and classify 4 blocks on the white plate
#     by sampling 4 points along one image row.

#     Parameters:
#       - image_topic (string): ROS image topic from ros_gz_bridge
#       - row_fraction (double): 0.0..1.0, vertical position of the sampling row
#       - x_fracs (double array): 4 fractions in 0.0..1.0 for each block along width
#       - debug_hsv (bool): if true, draw debug overlay and show with OpenCV
#     """

#     def __init__(self) -> None:
#         super().__init__('block_color_scanner')

#         # --- Parameters ---
#         self.declare_parameter(
#             'image_topic',
#             '/world/empty/model/lab06_overhead_camera/link/camera_link/sensor/overhead_camera/image'
#         )
#         self.declare_parameter('row_fraction', 0.40)
#         self.declare_parameter('x_fracs', [0.25, 0.45, 0.65, 0.85])
#         self.declare_parameter('debug_hsv', False)

#         image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
#         self.row_fraction = float(self.get_parameter('row_fraction').get_parameter_value().double_value)
#         self.x_fracs = list(self.get_parameter('x_fracs').get_parameter_value().double_array_value)
#         self.debug_hsv = bool(self.get_parameter('debug_hsv').get_parameter_value().bool_value)

#         if len(self.x_fracs) != 4:
#             self.get_logger().warn(
#                 f"x_fracs has {len(self.x_fracs)} entries, expected 4; will still try to use them."
#             )

#         self.get_logger().info(f"Subscribing to image topic: {image_topic}")
#         self.get_logger().info(f"row_fraction={self.row_fraction}, x_fracs={self.x_fracs}")
#         self.get_logger().info(f"debug_hsv={self.debug_hsv}")

#         # --- ROS setup ---
#         self.bridge = CvBridge()
#         self.image_sub = self.create_subscription(
#             Image, image_topic, self._image_callback, 10
#         )
#         self.result_pub = self.create_publisher(String, 'lab06/block_colors', 10)

#         # HSV color ranges (from mastermind vision)
#         self.color_ranges = {
#             'red':    ([0,   100, 100], [10,  255, 255]),
#             'red2':   ([170, 100, 100], [180, 255, 255]),
#             'green':  ([40,  100, 100], [80,  255, 255]),
#             'blue':   ([100, 100, 100], [140, 255, 255]),
#             'yellow': ([20,  100, 100], [35,  255, 255]),
#             'purple': ([140, 100, 100], [160, 255, 255]),
#             'black':  ([0,   0,   0],   [180, 255, 40]),
#         }

#     # ------------------------------------------------------------------
#     # Image callback
#     # ------------------------------------------------------------------
#     def _image_callback(self, msg: Image) -> None:
#         try:
#             cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         except CvBridgeError as exc:
#             self.get_logger().error(f"Failed to convert image: {exc}")
#             return

#         labels, debug_img = self.process_image(cv_img)

#         if labels:
#             text = ', '.join(labels)
#             self.get_logger().info(f"Detected colors (left→right): {text}")
#             self.result_pub.publish(String(data=text))

#         if self.debug_hsv and debug_img is not None:
#             try:
#                 cv2.imshow('lab06_overhead_debug', debug_img)
#                 cv2.waitKey(1)
#             except Exception as exc:
#                 self.get_logger().warn(f"OpenCV imshow failed: {exc}")

#     # ------------------------------------------------------------------
#     # Core processing
#     # ------------------------------------------------------------------
#     def process_image(self, bgr_img):
#         h, w, _ = bgr_img.shape
#         if h == 0 or w == 0:
#             return [], None

#         row = int(self.row_fraction * h)
#         row = max(0, min(h - 1, row))

#         hsv = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)

#         patch_half = 10  # half-size of sampling window
#         labels = []
#         debug_img = bgr_img.copy()

#         for i, xf in enumerate(self.x_fracs):
#             col = int(xf * w)
#             col = max(0, min(w - 1, col))

#             r0 = max(0, row - patch_half)
#             r1 = min(h, row + patch_half)
#             c0 = max(0, col - patch_half)
#             c1 = min(w, col + patch_half)

#             patch = hsv[r0:r1, c0:c1]
#             if patch.size == 0:
#                 labels.append('unknown')
#                 continue

#             color_label = self._classify_patch(patch)
#             labels.append(color_label)

#             # debug rectangle & label
#             cv2.rectangle(debug_img, (c0, r0), (c1, r1), (0, 255, 0), 1)
#             cv2.putText(
#                 debug_img, color_label, (c0, max(0, r0 - 5)),
#                 cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1, cv2.LINE_AA
#             )

#         return labels, debug_img

#     # ------------------------------------------------------------------
#     # Color classification
#     # ------------------------------------------------------------------
#     def _classify_patch(self, patch_hsv):
#         best_color = 'unknown'
#         best_score = 0

#         for cname, (lower, upper) in self.color_ranges.items():
#             if cname == 'red2':
#                 # handled as part of 'red' combined, skip
#                 continue

#             lower_np = np.array(lower, dtype=np.uint8)
#             upper_np = np.array(upper, dtype=np.uint8)

#             if cname == 'red':
#                 l2, u2 = self.color_ranges['red2']
#                 lower2_np = np.array(l2, dtype=np.uint8)
#                 upper2_np = np.array(u2, dtype=np.uint8)
#                 mask = cv2.inRange(patch_hsv, lower_np, upper_np) + \
#                        cv2.inRange(patch_hsv, lower2_np, upper2_np)
#             else:
#                 mask = cv2.inRange(patch_hsv, lower_np, upper_np)

#             score = int(cv2.countNonZero(mask))
#             if score > best_score:
#                 best_score = score
#                 best_color = cname

#         # very small score → treat as unknown
#         if best_score < 10:
#             return 'unknown'
#         return best_color


# def main(args=None):
#     rclpy.init(args=args)
#     node = BlockColorScanner()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()


