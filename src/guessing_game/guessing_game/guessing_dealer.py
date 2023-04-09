"""Creates a ROS Node that will be server side of the Guessing Action"""
from bisect import insort
from random import randrange
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from guessing_interfaces.action import Guessing
from guessing_interfaces.msg import Guess

class GuessingDealer(Node):
    """Defines the Node"""
    def __init__(self):
        super().__init__('guessing_dealer')
        self._action_server = ActionServer(
            self,
            Guessing,
            'guessing',
            self.action_callback)

        self.pub = self.create_publisher(Guess, 'guessing_round', 10)
        self.winner = randrange(1, 100)
        self.guessed = [0] # A list of all wrong guesses
        self.last_guess = time.time() # Keeps time of last guess

        self.timer_period = 5.0 # in seconds, period to check if we've had guesses
        self.check_for_guess = self.create_timer(self.timer_period, self.check_for_guess_callback)

    def check_for_guess_callback(self):
        """
            Every `self.last_guess` seconds check if we've had a guess
            and if not, send a message via topic to encourage players
        """
        if time.time() - self.last_guess > self.timer_period:
            self.publish_topic(0, False, True)

    def is_winner(self, number):
        """return true if the number matches the picked winner"""
        return number == self.winner

    def publish_topic(self, guess, is_winner, encourage=False):
        """publishes to the guessing_round topic using the Guess message type"""
        topic = Guess()

        if is_winner:
            self.get_logger().info('Posting winner')
            topic.msg = f'A player has won, the number was {guess}'
            topic.win = True
            self.get_logger().info(f'A player won: "{guess}"')
        elif encourage is True: # try to make the players start guessing again
            topic.msg = 'No guesses in a while'
            topic.win = False
        else:
            topic.msg = f'A player guessed {guess}, close but no cigar'
            topic.win = False

        topic.guessed = self.guessed
        self.get_logger().info('Publishing Topic')
        self.pub.publish(topic)

    def action_callback(self, goal_handle):
        """handles the Guessing Action goal"""
        self.get_logger().info('Got a guess...')
        self.last_guess = time.time() # Restart the time mark

        result = Guessing.Result()
        if self.is_winner(goal_handle.request.guess):
            result.win = True
            self._set_winner()
        else:
            result.win = False
            self._add_guessed(goal_handle.request.guess)

        self.publish_topic(goal_handle.request.guess, result.win)
        goal_handle.succeed()

        return result

    def _set_winner(self):
        # Reset values for a new round
        self.winner = randrange(1,100)
        self.guessed = [0]

    def _add_guessed(self, number):
        insort(self.guessed, number)


def main(args=None):
    """Starts the server"""
    rclpy.init(args=args)

    guessing_dealer = GuessingDealer()

    try:
        rclpy.spin(guessing_dealer)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        guessing_dealer.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
