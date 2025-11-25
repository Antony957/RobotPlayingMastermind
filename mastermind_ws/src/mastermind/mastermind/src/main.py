import threading
from time import time
from typing import List

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from mastermind_interfaces.msg import Code

from .game_state.game_state import COLOR_TO_NUM, GameState
from .robot_controller.pick_and_place import PickAndPlaceNode


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

        # Pub/subs
        self.code_pub = self.create_publisher(Code, "submit_code", 10)

    def run(self):
        secret = self.get_parameter("secret").get_parameter_value().string_value

        # Make sure we don't forget to pass in secret
        if not secret:
            self.get_logger().error("No secret code provided!")
            return

        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(self)
        executor.add_node(self.game_state)
        executor.add_node(self.pick_and_place)

        try:
            spin_thread = threading.Thread(target=executor.spin, daemon=True)
            spin_thread.start()

            # Sleep for 1 second to make sure all our pub/subs are up and running
            time.sleep(1)

            secret_list = secret.split(" ")
            code = [COLOR_TO_NUM[c] for c in secret_list]
            self.publish_code(code)
        except KeyboardInterrupt:
            pass
        finally:
            executor.shutdown()
            spin_thread.join(timeout=1.0)
            self.game_state.destroy_node()
            self.pick_and_place.destroy_node()
            self.destroy_node()
            rclpy.shutdown()

    def publish_code(self, code: List[int]):
        """
        Publish secret code as Player 1
        """
        msg = Code()
        msg.sender = "player_1"  # player_1, player_2, computer_vision, etc.
        msg.code = code

        self.get_logger().log("Player 1 publishing secret!")

        self.code_pub.publish(msg)


def main():
    rclpy.init()
    mastermind = Mastermind()
    mastermind.run()


if __name__ == "__main__":
    main()
