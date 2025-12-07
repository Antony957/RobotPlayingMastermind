#!/opt/ros_python/bin/python3

import os
import subprocess
import threading
import time
from typing import List

import rclpy
from mastermind_interfaces.msg import Code, Status
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from .game_state.game_state import COLOR_TO_NUM, GameState
from .robot_controller.pick_and_place import PickAndPlaceNode
from .vision_model.vision import VisionNode


class Mastermind(Node):
    """
    This node acts like Player 1 and runs everything else.
    """

    def __init__(self):
        super().__init__("mastermind")
        self.declare_parameter("secret", "")
        self.declare_parameter("sim", False)

        # Game nodes
        self.game_state = GameState()
        self.pick_and_place = PickAndPlaceNode()
        self.vision = VisionNode()

        # Pub/subs
        self.code_pub = self.create_publisher(Code, "submit_code", 10)
        self.game_status_sub = self.create_subscription(
            Status, "game_status", self.handle_status, 10
        )

        self.game_in_progress = False
        self.sim_mode = self.get_parameter("sim").get_parameter_value().bool_value

    def handle_status(self, msg: Status):
        sender = msg.sender
        status = msg.status

        self.get_logger().info(f"Sender {sender} status {status}")

        if sender == "game_state" and status == 0 and self.game_in_progress:
            if self.sim_mode:
                # Call a script to reset blocks in Gazebo
                self.reset_blocks()

            self.pick_and_place.reset_blocks()

    def check_secret(self, secret):
        if not secret:
            raise ValueError(f"No secret provided!")

        secret_list = [s for s in secret.split() if s]
        if len(secret_list) != len(set(secret_list)):
            raise ValueError(f"Secret must contain 4 different colors!")

        allowed = {"blue", "yellow", "green", "red", "purple", "black"}
        invalid = [c for c in secret_list if c not in allowed]
        if invalid:
            raise ValueError(
                "Colors must be four of this list: 'blue', 'yellow', 'green', 'red', 'purple', 'black'"
            )
        return secret_list

    def reset_blocks(self):
        script_path = (
            "/root/workspaces/mastermind_ws/src/mastermind/world/reset_blocks.sh"
        )
        subprocess.run(["bash", script_path], check=True)

    def run(self):
        secret = self.get_parameter("secret").get_parameter_value().string_value

        secret_list = self.check_secret(secret)

        executor = MultiThreadedExecutor(num_threads=8)
        executor.add_node(self)
        executor.add_node(self.game_state)
        executor.add_node(self.pick_and_place)
        executor.add_node(self.vision)

        try:
            spin_thread = threading.Thread(target=executor.spin, daemon=True)
            spin_thread.start()
            self.get_logger().info("Adding scene...")
            self.pick_and_place.add_scene()
            time.sleep(3)
            self.get_logger().info("Adding scene completed")

            # Wait until we have enough subscribers to submit_code
            # (i.e., that GameState is active)
            num_subs_submit_code = 1
            while self.code_pub.get_subscription_count() < num_subs_submit_code:
                self.get_logger().info("Waiting for /submit_code subscriber...")
                time.sleep(0.1)

            code = [COLOR_TO_NUM[c] for c in secret_list]
            self.publish_code(code)

            self.game_in_progress = True
            self.get_logger().info("Mastermind running. Press Ctrl+C to exit.")
            while rclpy.ok():
                time.sleep(0.1)

        except KeyboardInterrupt:
            self.get_logger().info("KeyboardInterrupt received. Shutting down...")
        finally:
            executor.shutdown()
            spin_thread.join(timeout=1.0)
            self.game_state.destroy_node()
            self.pick_and_place.destroy_node()
            self.vision.destroy_node()
            self.destroy_node()
            rclpy.shutdown()

    def publish_code(self, code: List[int]):
        """
        Publish secret code as Player 1
        """
        msg = Code()
        msg.player_name = "player_1"  # player_1, player_2, computer_vision, etc.
        msg.code = code

        self.code_pub.publish(msg)
        self.get_logger().info(f"Player 1 published secret {code}!")


def start_camera_bridge(logger):
    """
    bridge Gazebo & ROS2

      ros2 run ros_gz_bridge parameter_bridge \
        "/mastermind/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image"

    """
    cmd = [
        "ros2",
        "run",
        "ros_gz_bridge",
        "parameter_bridge",
        "/mastermind/camera/image_raw" "@sensor_msgs/msg/Image" "@gz.msgs.Image",
    ]
    logger.info("bridging images...")

    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    return proc


def main():
    rclpy.init()
    mastermind = Mastermind()
    bridge_proc = start_camera_bridge(mastermind.get_logger())
    try:
        mastermind.run()
    finally:
        if bridge_proc is not None:
            try:
                mastermind.get_logger().info("Stopping camera bridge...")
                bridge_proc.terminate()
                bridge_proc.wait(timeout=2.0)
            except Exception:
                bridge_proc.kill()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
