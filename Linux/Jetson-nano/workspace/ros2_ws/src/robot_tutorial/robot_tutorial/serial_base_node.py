import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist


class SerialBaseNode(Node):

    def __init__(self):

        super().__init__('serial_base_node')

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info(
            'Serial Base Node started'
        )


    def cmd_vel_callback(self, msg):

        linear = msg.linear.x
        angular = msg.angular.z

        self.get_logger().info(
            f'cmd_vel: '
            f'linear={linear:.2f} m/s, '
            f'angular={angular:.2f} rad/s'
        )


def main(args=None):

    rclpy.init(args=args)

    node = SerialBaseNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
