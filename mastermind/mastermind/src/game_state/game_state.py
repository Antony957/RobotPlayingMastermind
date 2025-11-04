import os
from typing import List

import rclpy
from rclpy.node import Node

# TODO Need to check folder structure
from mastermind.msg import Code, GuessCheck, Status

COLOR_MAP = {
    0: "none",
    1: "red",
    2: "green",
    3: "blue",
    4: "yellow",
}

GAME_STATUS = {
    0: "waiting for player 1",  # player 1 watches for this to start game
    1: "waiting for player 2",  # player 2 watches for this to make guess
    2: "waiting for robot arm",  # player 2 publishes this, robot_arm watches for this
    3: "waiting for computer_vision",  # robot_arm publishes this, CV watches for this
    4: "player 2 wins",  # end state 1
    5: "player 2 loses",  # end state 2
}


class GameState(Node):
    """
    Node to keep track of game state and pass messages.

    Block diagram, with game status code (sent via /game_status topic)
    or trigger (sent via /submit_code or /guess_check topic):

             secret code              1            2             3
    Player1 -------------> GameState ---> Player2 ---> RobotArm ---> ComputerVision
                               ^                                           |
                               |                guessed code               |
                               |-------------------------------------------|
    """

    def __init__(self, max_guesses: int = 10):
        super().__init__("game_state")

        # Keep track for observability (i.e., logging)
        self.current_game_status: int = 0  # see GAME_STATUS

        # Code related stuff
        self.secret_code: List[int] = [0, 0, 0, 0]  # secret code from Player 1
        self.last_guess: List[int] = [0, 0, 0, 0]  # last guessed code

        # Guess related stuff
        self.num_guesses: int = 0  # number of current guess
        self.max_guesses: int = max_guesses  # maximum number of guesses
        self.num_correct_colors: int = 0  # number of correctly-guessed colors
        self.num_correct_pos: int = 0  # number of correctly-guessed positions

        # Subscriptions
        self.submit_code_sub = self.create_subscription(
            Code, "submit_code", self.handle_code, 10
        )

        # Publishers
        self.game_status_pub = self.create_publisher(Status, "game_status", 10)
        self.guess_check_pub = self.create_publisher(GuessCheck, "guess_check", 10)

    def publish_game_status(self, status: int):
        """
        Publish given status from sender "game_state".
        """
        msg = Status()
        msg.sender = "game_state"
        msg.status = status
        self.game_status_pub.publish(msg)

        # Keep track, just in case
        self.current_game_status = status
        self.get_logger().info(f"Game status: {status} - {GAME_STATUS[status]}")

    def publish_guess_check(self):
        """
        Publish how many colors and positions are correct
        to guess_check topic.
        """
        msg = GuessCheck()
        msg.num_correct_colors = self.num_correct_colors
        msg.num_correct_pos = self.num_correct_pos

        self.get_logger().info(
            f"Player 2 got {self.num_correct_colors} colors and {self.num_correct_pos} positions correct!"
        )

        # Publish to guess_check so player_2 has feedback
        self.guess_check_pub.publish(msg)

    def handle_code(self, msg: Code):
        """
        Store secret code if code message is from player_1.
        If sender is computer_vision, then we check whether
        the guess is correct and keep track of it.

        We don't accept anything from player_2 because
        player_2 only talks to robot_arm, then robot_arm talks
        to computer_vision.
        """
        # If code comes from player_1, keep track of it as secret
        # and publish game status 1 ("waiting for player 2")
        if msg.player_name == "player_1":
            self.get_logger().info("Secret code received from player 1!")

            self.secret_code = msg.code
            self.publish_game_status(1)

        # If code comes from computer_vision, check guess
        # and publish appropriate status and/or feedback for player_2
        elif msg.player_name == "computer_vision":
            self.get_logger().info(f"Code {msg.code} received from computer_vision")

            self.last_guess = msg.code
            self.num_guesses += 1

            guess_correct = self.check_guess()

            # If guess is correct and we haven't gone over max guess,
            # player 2 wins
            if guess_correct and self.num_guesses <= self.max_guesses:
                self.publish_game_status(4)

                self.get_logger().info("Correct guess...")
                self.get_logger().info("Player 2 wins!")

            elif not guess_correct:
                self.get_logger().info("Incorrect guess...")

                # If guess is not correct and we're at (or above) max,
                # player 2 loses
                if self.num_guesses >= self.max_guesses:
                    self.get_logger().info("Player 2 loses!")
                    self.publish_game_status(5)

                # Otherwise, send feedback and go to next round
                else:
                    self.get_logger().info("Try again!")

                    self.publish_guess_check()
                    self.publish_game_status(1)  # "waiting for player_2"

    def check_guess(self):
        """
        Check guess for number of correct colors and
        number of correct positions.
        """
        # Reset per check
        self.num_correct_colors = 0
        self.num_correct_pos = 0

        for i, c in enumerate(self.last_guess):
            # Check if each guessed color exists in secret code
            if c in self.secret_code:
                self.num_correct_colors += 1

            # Check if each guessed color is in the right place
            if c == self.secret_code[i]:
                self.num_correct_pos += 1

        # For convenience, return True if guess matches secret
        return self.num_correct_colors == 4 and self.num_correct_pos == 4


def main():
    rclpy.init()
    game_state = GameState()
    try:
        rclpy.spin(game_state)
    except KeyboardInterrupt:
        pass
    finally:
        game_state.destory_node()
        rclpy.shutdown()
