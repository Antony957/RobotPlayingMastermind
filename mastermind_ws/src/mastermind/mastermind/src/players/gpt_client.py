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

