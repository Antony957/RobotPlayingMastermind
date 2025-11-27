import os
import threading
import time
from typing import List
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from mastermind_interfaces.msg import Code, GuessCheck, Status

from ..game_state.game_state import COLOR_TO_NUM, GAME_STATUS
import os
from openai import OpenAI
from pydantic import BaseModel
from enum import Enum
from typing import List


class Client():
    def __init__(self):
        self.client = OpenAI()
        self.history = []

    def guess(self):
        system_prompt = """
You are a logical reasoning assistant playing MasterMind.
Rules and constraints:
There are exactly 6 possible colors with one each, the secret code length is 4. 
The secret uses distinct color only. Every guess you propose must contain 4 distinct number.
Feedback after each guess is given as exact match(A) and color match(B).
For example: 2 Exact Match and 3 Color Match will be represented by 2A3B.
Your job: Analyze guess and feedback history.
Maintain a consistent candidate set under constraint.
Propose the next guess that best narrows the search.
Eliminate impossible numbers.
Output your guess in JSON format.
"""
        user_prompt = """We are playing mastermind with no repetition. Possible colors: red, green, blue, yellow, purple, black. 
        You will explain the reasoning process, then give me your guess in this round."""

        if not len(self.history) == 0:
            user_prompt += "Guess History: "
            for item in self.history:
                user_prompt += f"Guess: {item.guess}; Feedback: {item.feedback} "

        user_prompt += """Example output: {
  \"reasoning\": \"Why this guess reduces the space under the no-duplicate constraint.\",
  \"guess\": [\"red\", \"green\", \"blue\", \"yellow\"]
}"""
        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_prompt}
        ]

        class GuessRes(str, Enum):
            red = "red"
            green = "green"
            blue = "blue"
            yellow = "yellow"
            purple = "purple"
            black = "black"

        class ResponseFormat(BaseModel):
            reasoning: str
            guess: List["GuessRes"]


        response = self.client.responses.parse(
            model="gpt-5-nano",
            input = messages,
            text_format=ResponseFormat
        )

        parsed: ResponseFormat = response.output_parsed

        guess_list = [g.value for g in parsed.guess]
        return guess_list




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



