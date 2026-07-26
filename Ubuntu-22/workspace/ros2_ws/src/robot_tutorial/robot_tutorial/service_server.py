import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger


class ServiceServer(Node):

    def __init__(self):
        super().__init__('service_server')

        self.counter = 0

        self.timer = self.create_timer(
            1.0,
            self.timer_callback
        )

        self.service = self.create_service(
            Trigger,
            '/reset_counter',
            self.reset_callback
        )

        self.get_logger().info(
            'Service Server started'
        )

    def timer_callback(self):

        self.counter += 1

        self.get_logger().info(
            f'Counter = {self.counter}'
        )

    def reset_callback(self, request, response):

        self.counter = 0

        response.success = True
        response.message = 'Counter reset successfully'

        self.get_logger().info(
            'Reset service called'
        )

        return response


def main(args=None):

    rclpy.init(args=args)

    node = ServiceServer()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
