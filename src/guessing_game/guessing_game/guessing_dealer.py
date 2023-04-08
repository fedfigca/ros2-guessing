from bisect import insort
from random import randrange

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from guessing_interfaces.action import Guessing
from guessing_interfaces.msg import Guess

class GuessingDealer(Node):
    def __init__(self):
        super().__init__('guessing_dealer')
        self._action_server = ActionServer(
            self,
            Guessing,
            'guessing',
            self.action_callback)

        self.pub = self.create_publisher(Guess, 'guessing_round', 10)
        self.winner = randrange(1, 100)
        self.guessed = [0]

    def is_winner(self, number):
        return number == self.winner

    def publish_topic(self, guess, is_winner):
        topic = Guess()
        self.get_logger().info('Posting guess {}'.format(guess))
        if is_winner:
            self.get_logger().info('Posting winner')
            topic.msg = 'A player has won, the number was {}'.format(guess)
            topic.win = True
            self.get_logger().info('A player won: "{}"'.format(guess))
        else:
            topic.msg = 'A player guessed {}, close but no cigar'.format(guess)
            topic.win = False

        topic.guessed = self.guessed
        self.get_logger().info('A player guessed: "{}"'.format(guess))
        self.pub.publish(topic)

    def action_callback(self, goal_handle):
        self.get_logger().info('Got a guess...')

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
        self.winner = randrange(1,100);

    def _add_guessed(self, number):
        insort(self.guessed, number)


def main(args=None):
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
