#!/usr/bin/env python3

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class OdomCalibrator(Node):
    def __init__(self) -> None:
        super().__init__('odom_calibrator')

        self.start_x = None
        self.start_yaw = None

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
        )

        print()
        print('ODOM CALIBRATION')
        print('Each run starts from zero.')
        print('Ctrl+C to stop.')
        print()

    @staticmethod
    def quaternion_to_yaw(
        x: float,
        y: float,
        z: float,
        w: float,
    ) -> float:
        siny_cosp = 2.0 * (
            w * z + x * y
        )

        cosy_cosp = 1.0 - 2.0 * (
            y * y + z * z
        )

        return math.atan2(
            siny_cosp,
            cosy_cosp,
        )

    @staticmethod
    def normalize_angle(
        angle: float,
    ) -> float:
        return math.atan2(
            math.sin(angle),
            math.cos(angle),
        )

    def odom_callback(
        self,
        msg: Odometry,
    ) -> None:
        x = msg.pose.pose.position.x

        q = msg.pose.pose.orientation

        yaw = self.quaternion_to_yaw(
            q.x,
            q.y,
            q.z,
            q.w,
        )

        if self.start_x is None:
            self.start_x = x
            self.start_yaw = yaw

            print('Baseline captured.')
            return

        delta_x_mm = (
            x - self.start_x
        ) * 1000.0

        delta_yaw = self.normalize_angle(
            yaw - self.start_yaw
        )

        delta_yaw_deg = math.degrees(
            delta_yaw
        )

        print(
            '\r'
            f'X travel: {delta_x_mm:+8.1f} mm | '
            f'Yaw: {delta_yaw_deg:+7.2f} deg',
            end='',
            flush=True,
        )


def main(
    args=None,
) -> None:
    rclpy.init(args=args)

    node = OdomCalibrator()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        print()

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()