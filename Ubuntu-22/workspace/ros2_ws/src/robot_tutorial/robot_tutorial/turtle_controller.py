import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class TurtleController(Node):

    def __init__(self):
        super().__init__('turtle_controller')

        # Declare ROS parameters
        self.declare_parameter('target_x', 8.0)
        self.declare_parameter('linear_speed', 1.0)
        self.declare_parameter('angular_speed', 1.0)

        # Read parameter values
        self.target_x = (
            self.get_parameter('target_x')
            .get_parameter_value()
            .double_value
        )

        self.linear_speed = (
            self.get_parameter('linear_speed')
            .get_parameter_value()
            .double_value
        )

        self.angular_speed = (
            self.get_parameter('angular_speed')
            .get_parameter_value()
            .double_value
        )

        # Publisher
        self.publisher_ = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

        # Subscriber
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )

        self.get_logger().info(
            f'Turtle Controller started | '
            f'target_x={self.target_x}, '
            f'linear_speed={self.linear_speed}, '
            f'angular_speed={self.angular_speed}'
        )

    def pose_callback(self, msg):

        command = Twist()

        if msg.x < self.target_x:
            command.linear.x = self.linear_speed
            command.angular.z = 0.0
        else:
            command.linear.x = 0.0
            command.angular.z = self.angular_speed

        self.publisher_.publish(command)


def main(args=None):
    rclpy.init(args=args)

    node = TurtleController()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()