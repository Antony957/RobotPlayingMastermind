import json
from dataclasses import dataclass
from typing import Dict, List, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, UInt8MultiArray


@dataclass(frozen=True)
class SlotDefinition:
    """Normalized slot position (u, v) with sampling half-width in pixels."""

    u: float
    v: float
    half_width: int = 12


class GuessScanner(Node):
    """Computer vision node that decodes Mastermind guess colors from camera images."""

    COLOR_TO_INDEX: Dict[str, int] = {
        "unknown": 0,
        "red": 1,
        "blue": 2,
        "green": 3,
        "yellow": 4,
        "orange": 5,
        "black": 6,
        "empty": 7,
    }

    COLOR_REFERENCE_BGR: Dict[str, Tuple[int, int, int]] = {
        "red": (0, 0, 255),
        "blue": (255, 0, 0),
        "green": (0, 255, 0),
        "yellow": (0, 255, 255),
        "orange": (0, 165, 255),
        "black": (20, 20, 20),
        "empty": (40, 40, 40),  # Approximate board color
    }

    def __init__(self) -> None:
        super().__init__("guess_scanner")
        self.bridge = CvBridge()
        self.latest_image: np.ndarray | None = None
        self.latest_stamp = None
        self.scan_requested = False

        self.declare_parameter("image_topic", "/mastermind/overhead_camera/image_raw")
        self.declare_parameter("robot_status_topic", "/robot_status")
        self.declare_parameter("scanned_guess_topic", "/scanned_guess")
        self.declare_parameter("debug_topic", "/scanned_guess/debug_text")
        default_slots = [
            json.dumps({"u": 0.365, "v": 0.47, "half_width": 14}),
            json.dumps({"u": 0.45, "v": 0.47, "half_width": 14}),
            json.dumps({"u": 0.535, "v": 0.47, "half_width": 14}),
            json.dumps({"u": 0.62, "v": 0.47, "half_width": 14}),
        ]
        self.declare_parameter("slots", default_slots)
        self.declare_parameter("distance_threshold", 20.0)
        self.declare_parameter("auto_scan_timer", 0.0)

        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        robot_status_topic = (
            self.get_parameter("robot_status_topic").get_parameter_value().string_value
        )
        self.scanned_guess_topic = (
            self.get_parameter("scanned_guess_topic").get_parameter_value().string_value
        )
        self.debug_topic = self.get_parameter("debug_topic").get_parameter_value().string_value
        self.distance_threshold = (
            self.get_parameter("distance_threshold").get_parameter_value().double_value
        )

        slot_defs_raw = list(
            self.get_parameter("slots").get_parameter_value().string_array_value
        )
        if not slot_defs_raw:
            slot_defs_raw = default_slots
        self.slots = self._parse_slots(slot_defs_raw)

        self.image_subscription = self.create_subscription(
            Image, image_topic, self._image_callback, 10
        )
        self.status_subscription = self.create_subscription(
            String, robot_status_topic, self._status_callback, 10
        )
        self.result_pub = self.create_publisher(UInt8MultiArray, self.scanned_guess_topic, 10)
        self.debug_pub = self.create_publisher(String, self.debug_topic, 10)

        auto_scan_period = (
            self.get_parameter("auto_scan_timer").get_parameter_value().double_value
        )
        if auto_scan_period > 0.0:
            self.create_timer(auto_scan_period, self._auto_scan)

        self.reference_lab = self._build_reference_lab()

        self.get_logger().info(
            "Guess scanner ready. Subscribed to %s, watching %d slots.",
            image_topic,
            len(self.slots),
        )

    def _build_reference_lab(self) -> Dict[str, np.ndarray]:
        result: Dict[str, np.ndarray] = {}
        for name, bgr in self.COLOR_REFERENCE_BGR.items():
            bgr_array = np.uint8([[bgr]])
            lab = cv2.cvtColor(bgr_array, cv2.COLOR_BGR2LAB)[0, 0].astype(np.float32)
            result[name] = lab
        return result

    def _parse_slots(self, slot_defs_raw: List[str]) -> List[SlotDefinition]:
        slots: List[SlotDefinition] = []
        for raw in slot_defs_raw:
            try:
                data = json.loads(raw)
            except json.JSONDecodeError:
                self.get_logger().error("Invalid slot definition: %s", raw)
                continue
            slots.append(
                SlotDefinition(
                    u=float(data["u"]),
                    v=float(data["v"]),
                    half_width=int(data.get("half_width", 12)),
                )
            )
        if not slots:
            raise ValueError("No valid slot definitions provided")
        return slots

    def _image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().error("Failed to convert image: %s", exc)
            return
        self.latest_image = frame
        self.latest_stamp = msg.header.stamp
        if self.scan_requested:
            self.scan_requested = False
            self._process_frame()

    def _status_callback(self, msg: String) -> None:
        if msg.data.lower() == "done":
            if self.latest_image is not None:
                self._process_frame()
            else:
                self.get_logger().warn("Scan requested but no image received yet")
                self.scan_requested = True

    def _auto_scan(self) -> None:
        if self.latest_image is not None:
            self._process_frame()

    def _process_frame(self) -> None:
        assert self.latest_image is not None
        image = self.latest_image.copy()
        h, w, _ = image.shape
        results: List[str] = []
        debug_overlay = image.copy()
        for idx, slot in enumerate(self.slots):
            cx = int(slot.u * w)
            cy = int(slot.v * h)
            hw = slot.half_width
            x0 = max(cx - hw, 0)
            x1 = min(cx + hw, w)
            y0 = max(cy - hw, 0)
            y1 = min(cy + hw, h)
            roi = image[y0:y1, x0:x1]
            if roi.size == 0:
                self.get_logger().warning("Empty ROI for slot %d", idx)
                results.append("unknown")
                continue
            label = self._classify_roi(roi)
            results.append(label)
            cv2.rectangle(debug_overlay, (x0, y0), (x1, y1), (0, 255, 0), 1)
            cv2.putText(
                debug_overlay,
                label,
                (x0, y0 - 4),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )

        indices = [
            self.COLOR_TO_INDEX.get(label, self.COLOR_TO_INDEX["unknown"]) for label in results
        ]
        msg = UInt8MultiArray()
        msg.data = indices
        self.result_pub.publish(msg)

        debug_text = String()
        debug_payload = {
            "stamp": {
                "sec": self.latest_stamp.sec if self.latest_stamp else 0,
                "nanosec": self.latest_stamp.nanosec if self.latest_stamp else 0,
            },
            "labels": results,
            "indices": indices,
        }
        debug_text.data = json.dumps(debug_payload)
        self.debug_pub.publish(debug_text)

        self.get_logger().info("Detected guess: %s", results)

    def _classify_roi(self, roi: np.ndarray) -> str:
        roi_lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB).astype(np.float32)
        avg_lab = roi_lab.reshape(-1, 3).mean(axis=0)
        best_color = "unknown"
        best_distance = float("inf")
        for name, ref_lab in self.reference_lab.items():
            dist = np.linalg.norm(avg_lab - ref_lab)
            if dist < best_distance:
                best_distance = dist
                best_color = name
        if best_distance > self.distance_threshold:
            return "unknown"
        return best_color


def main() -> None:
    rclpy.init()
    node = GuessScanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()