import os
import threading
from time import time
from typing import List

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from mastermind_interfaces.msg import Code, GuessCheck, Status

from ..game_state.game_state import COLOR_TO_NUM
from ..robot_controller.pick_and_place import PickAndPlaceNode


class Player2(Node):
    """
    Player 2 node. This is where we can plug in the ChatGPT node,
    or use it as a standalone player 2.
    """

    def __init__(self):
        super().__init__("player_2")

        # Publishrs
        self.code_pub = self.create_publisher(Code, "submit_code", 10)

        # For input thread
        self.stop_flag = False
        self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.input_thread.start()

    def publish_code(self, code: List[int]):
        """
        Publish guess to /submit_code topic
        """
        msg = Code()
        msg.player_name = "player_2"  # player_1, player_2, computer_vision, etc.
        msg.code = code

        self.code_pub.publish(msg)
        self.publish_game_status(2)

    def input_loop(self):
        """
        Continuously prompt the user for guesses and publish them.
        """
        while not self.stop_flag:
            try:
                # Ask user for input
                user_input = input(
                    "\nEnter your guess (e.g. 'red blue red blue') or 'q' to quit: "
                ).strip()

                if user_input.lower() in ["q", "quit", "exit"]:
                    self.get_logger().info("Exiting Player2 input loop.")
                    self.stop_flag = True
                    break

                color_list = [c.lower() for c in user_input.split()]
                if len(color_list) != 4:
                    self.get_logger().warn("Enter 4 colors!")
                    break

                code = []
                for c in color_list:
                    if c not in COLOR_TO_NUM:
                        self.get_logger().warn(f"Unknown color '{c}' — try again.")
                        break
                    code.append(COLOR_TO_NUM[c])
                else:
                    self.publish_code(code)

            # Quit if player does Ctrl+D
            except EOFError:
                break

    def destroy_node(self):
        """
        Ensure the input thread stops on shutdown.
        """
        self.stop_flag = True
        time.sleep(0.2)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Player2()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
