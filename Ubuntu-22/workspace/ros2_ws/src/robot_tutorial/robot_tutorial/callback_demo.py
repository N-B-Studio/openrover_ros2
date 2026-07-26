import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose


class CallbackDemo(Node):

    def __init__(self):
        super().__init__('callback_demo')

        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )

        self.timer = self.create_timer(
            1.0,
            self.timer_callback
        )

        self.pose_count = 0

        self.get_logger().info(
            'Callback Demo started'
        )

    def pose_callback(self, msg):
        self.pose_count += 1

        self.get_logger().info(
            f'[POSE] count={self.pose_count}, '
            f'x={msg.x:.2f}, y={msg.y:.2f}'
        )

    def timer_callback(self):
        self.get_logger().info(
            f'[TIMER] Pose messages received so far: '
            f'{self.pose_count}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = CallbackDemo()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
