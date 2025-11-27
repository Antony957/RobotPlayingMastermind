#!/usr/bin/env python3

import threading
import time
from typing import List

import rclpy
from mastermind_interfaces.msg import Code
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from .game_state.game_state import COLOR_TO_NUM, GameState
from .robot_controller.pick_and_place import PickAndPlaceNode
from .vision_model.vision import VisionNode

import subprocess


class Mastermind(Node):
    """
    This node acts like Player 1 and runs everything else.
    """

    def __init__(self):
        super().__init__("mastermind")
        self.declare_parameter("secret", "")

        # Game nodes
        self.game_state = GameState()
        self.pick_and_place = PickAndPlaceNode()
        self.vision = VisionNode()

        # Pub/subs
        self.code_pub = self.create_publisher(Code, "submit_code", 10)


    def check_secret(self, secret):
        if not secret:
            raise ValueError(f"No secret provided!")
        
        secret_list = [s for s in secret.split() if s]
        if len(secret_list) != len(set(secret_list)):
            raise ValueError(f"Secret must contain 4 different colors!")
        
        allowed = {'blue', 'yellow', 'green', 'red', 'purple', 'black'}
        invalid = [c for c in secret_list if c not in allowed]
        if invalid:
            raise ValueError("Colors must be four of this list: 'blue', 'yellow', 'green', 'red', 'purple', 'black'")
        return secret_list
    

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
        "ros2", "run", "ros_gz_bridge", "parameter_bridge",
        "/mastermind/camera/image_raw"
        "@sensor_msgs/msg/Image"
        "@gz.msgs.Image",
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
