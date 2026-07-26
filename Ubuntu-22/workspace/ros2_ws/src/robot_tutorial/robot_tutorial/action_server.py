import time

import rclpy

from rclpy.action import ActionServer
from rclpy.node import Node

from example_interfaces.action import Fibonacci


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')

        self.action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )

        self.get_logger().info(
            'Fibonacci Action Server started'
        )

    def execute_callback(self, goal_handle):

        self.get_logger().info(
            'Executing goal...'
        )

        feedback_msg = Fibonacci.Feedback()

        feedback_msg.sequence = [0, 1]

        for i in range(
            1,
            goal_handle.request.order
        ):

            feedback_msg.sequence.append(
                feedback_msg.sequence[i]
                +
                feedback_msg.sequence[i - 1]
            )

            self.get_logger().info(
                f'Feedback: {feedback_msg.sequence}'
            )

            goal_handle.publish_feedback(
                feedback_msg
            )

            time.sleep(1.0)

        goal_handle.succeed()

        result = Fibonacci.Result()

        result.sequence = feedback_msg.sequence

        return result


def main(args=None):

    rclpy.init(args=args)

    node = FibonacciActionServer()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
