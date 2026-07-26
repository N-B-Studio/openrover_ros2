import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger


class ServiceClient(Node):

    def __init__(self):
        super().__init__('service_client')

        self.client = self.create_client(
            Trigger,
            '/reset_counter'
        )

        self.get_logger().info(
            'Waiting for /reset_counter service...'
        )

        while not self.client.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().info(
                'Service not available, waiting...'
            )

        self.get_logger().info(
            'Service available'
        )

    def send_request(self):

        request = Trigger.Request()

        future = self.client.call_async(
            request
        )

        return future


def main(args=None):

    rclpy.init(args=args)

    node = ServiceClient()

    future = node.send_request()

    rclpy.spin_until_future_complete(
        node,
        future
    )

    response = future.result()

    if response is not None:

        node.get_logger().info(
            f'Success: {response.success}'
        )

        node.get_logger().info(
            f'Message: {response.message}'
        )

    else:

        node.get_logger().error(
            'Service call failed'
        )

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
