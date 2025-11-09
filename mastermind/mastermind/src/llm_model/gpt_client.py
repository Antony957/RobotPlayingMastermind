import os
from openai import OpenAI
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pydantic import BaseModel
from enum import Enum
from typing import List


class Client(Node):
    def __init__(self):
        super().__init__('llm_service')
        self.toControl = self.create_publisher(String, 'LLM_to_Control', 10)
        self.fromCV = self.create_subscription(String, 'CV_to_LLM', self.firstRound, 10)
        self.fromUI = self.create_subscription(String, 'UI_to_LLM', self.checkWin, 10)
        self.client = OpenAI()
        self.history = []

    def guess(self):
        system_prompt = """
You are a logical reasoning assistant playing MasterMind.
Rules and constraints:
There are exactly 10 possible numbers (0-9) with one each, the secret code length is 4. 
The secret uses distinct color only. Every guess you propose must contain 4 distinct number.
Feedback after each guess is given as exact match(A) and color match(B).
For example: 2 Exact Match and 3 Color Match will be represented by 2A3B.
Your job: Analyze guess and feedback history.
Maintain a consistent candidate set under constraint.
Propose the next guess that best narrows the search.
Eliminate impossible numbers.
Output your guess in JSON format.
"""
        user_prompt = """We are playing mastermind with no repetition. Possible numbers: 0-9. 
        You will explain the reasoning process, then give me your guess in this round."""

        if not len(self.history) == 0:
            user_prompt += "Guess History: "
            for item in self.history:
                user_prompt += f"Guess: {item.guess}; Feedback: {item.feedback} "

        user_prompt += """Example output: {
  \"reasoning\": \"Why this guess reduces the space under the no-duplicate constraint.\",
  \"guess\": [\"1\", \"3\", \"4\", \"7\"]
}"""
        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_prompt}
        ]

        class GuessRes(str, Enum):
            a = "0"
            b = "1"
            c = "2"
            d = "3"
            e = "4"
            f = "5"
            g = "6"
            h = "7"
            i = "8"
            j = "9"

        class ResponseFormat(BaseModel):
            reasoning: str
            guess: List["GuessRes"]


        response = self.client.responses.parse(
            model="gpt-5-nano",
            input = messages,
            text_format=ResponseFormat
        )
        return response.output_parsed


    def firstRound(self, msg):
        if not msg == 'ok':
            return
            
        resp = self.guess()
        control_move = resp.guess[0].value+resp.guess[1].value+resp.guess[2].value+resp.guess[3].value
        self.toControl.publish(control_move)
        self.history.append({"guess": control_move})
        
        self.get_logger().info(resp)
        

    def checkWin(self, msg):
        if msg == "Win":
            return
        else:
            self.history[-1]["feedback"] = msg

        self.firstRound("ok")    

def main():
    rclpy.init()
    client = Client()
    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass
    finally:
        client.destory_node()
        rclpy.shutdown()