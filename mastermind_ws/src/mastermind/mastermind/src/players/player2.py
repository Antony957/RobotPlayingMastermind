import os
import threading
import time
from typing import List
from gpt_client import Client

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from mastermind_interfaces.msg import Code, GuessCheck, Status

from ..game_state.game_state import COLOR_TO_NUM, GAME_STATUS


class Player2(Node):
    """
    Player 2 node. This is where we can plug in the ChatGPT node,
    or use it as a standalone player 2.
    """

    def __init__(self):
        super().__init__("player_2")
        self.declare_parameter("mode", "human")

        # Publishers
        self.code_pub = self.create_publisher(Code, "submit_code", 10)

        # For input thread
        self.stop_flag = False

        self.client = None


    def publish_code(self, code: List[int]):
        """
        Publish guess to /submit_code topic
        """
        msg = Code()
        msg.player_name = "player_2"  # player_1, player_2, computer_vision, etc.
        msg.code = code

        self.code_pub.publish(msg)

    def check_and_publish_color(self, color_list):
        if len(color_list) != 4:
            self.get_logger().warn("Enter 4 colors!")
            return False
        for c in color_list:
            if c not in COLOR_TO_NUM:
                self.get_logger().warn(f"Unknown color '{c}' — try again.")
                return False
        allowed = {'blue', 'yellow', 'green', 'red', 'purple', 'black'}
        invalid = [c for c in color_list if c not in allowed]
        if invalid:
            return False
        code = [COLOR_TO_NUM[c] for c in color_list]
        self.publish_code(code)



    def input_loop(self):
        """
        Continuously prompt the user for guesses and publish them.
        """
        while not self.stop_flag:
            try:
                # Ask user for input
                user_input = input(
                    # "\nEnter your guess (e.g. 'red blue red blue') or 'q' to quit:\n"
                    "\nEnter your guess (e.g. 'red blue red blue'):\n"
                ).strip()

                # if user_input.lower() in ["q", "quit", "exit"]:
                #     self.get_logger().info("Exiting Player2 input loop.")
                #     self.stop_flag = True
                #     break

                color_list = [c.lower() for c in user_input.split()]
                
                self.check_and_publish_color(color_list)
                    

            # Quit if player does Ctrl+D
            except EOFError:
                break

    def ai_loop(self):
        while not self.stop_flag:
            color_list = self.client.guess()
            self.check_and_publish_color(color_list)


    def destroy_node(self):
        """
        Ensure the input thread stops on shutdown.
        """
        self.stop_flag = True
        time.sleep(0.2)
        super().destroy_node()


    def run(self):
        mode = self.get_parameter("mode").get_parameter_value().string_value
        if mode != "human" and mode!= "ai":
            raise ValueError("Mode must either human or ai")
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(self)
        threading.Thread(target=executor.spin, daemon=True).start()

        try:

            if mode == "human":
                self.get_logger().info("Player 2 running in HUMAN mode.")
                self.stop_flag = False
                self.input_loop()
                
            else:
                self.get_logger().info("Player 2 running in AI mode.")
                self.stop_flag = False
                self.client = Client()
                self.ai_loop()
                
            
        finally:
            executor.shutdown()
            self.destroy_node()
            rclpy.shutdown()


def main():
    rclpy.init()
    node = Player2()
    node.run()
    

if __name__ == "__main__":
    main()
