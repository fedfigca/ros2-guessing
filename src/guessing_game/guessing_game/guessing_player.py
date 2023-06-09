"""Creates a ROS Node that will work as a player for the Guessing Game"""
from random import randrange
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from guessing_interfaces.action import Guessing
from guessing_interfaces.msg import Guess

class GuessingActionClient(Node):
    """Defines the Node"""
    def __init__(self):
        super().__init__('guessing_action_client')
        self._action_client = ActionClient(self, Guessing, 'guessing')
        self.sub = self.create_subscription(Guess, 'guessing_round', self.topic_callback, 10)
        self.guessed = [0]

    def topic_callback(self, topic):
        """Handles recieved topic messages"""
        self.get_logger().info(f'Handling a topic: {topic.msg}')
        if topic.msg == 'No guesses in a while':
            self.send_goal()
            self.guessed = topic.guessed
        else:
            self.get_logger().info(f'{topic.msg}')
            self.get_logger().info(f'Did they win? : {topic.win}')

        if not topic.win:
            self.guessed = topic.guessed

    def send_goal(self):
        """Sends a Guess to the Action Server Node"""
        self.get_logger().info('Sending guess')
        goal_msg = Guessing.Goal()
        goal_msg.guess = 0
        while goal_msg.guess == 0:
            random_numer = randrange(1, 100)
            if random_numer not in self.guessed:
                goal_msg.guess = random_numer

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handles the response from the Action Server"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Guess accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handles the async result from the Action response"""
        result = future.result().result

        if result.win:
            self.get_logger().info('We Won!!')
        else:
            self.get_logger().info('We guessed wrong, trying again')
            time.sleep(0.25)
            self.send_goal()


def main(args=None):
    """Starts the Node"""
    rclpy.init(args=args)

    action_client = GuessingActionClient()

    try:
        rclpy.spin(action_client)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        action_client.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
