#!/usr/bin/env python3
import math
import os
import threading
import time

import serial

import rclpy

from geometry_msgs.msg import (
    TransformStamped,
    Twist,
)

from nav_msgs.msg import Odometry

from sensor_msgs.msg import JointState

from rclpy.node import Node

from tf2_ros import TransformBroadcaster

class BaseDriver(Node):
    def __init__(self) -> None:
        super().__init__('base_driver')

        # ============================================================
        # ROS parameters
        # ============================================================

        self.declare_parameter('serial_port', '/dev/ttyS4')
        self.declare_parameter('baud_rate', 115200)
        
        # Wheel geometry.
        # These are temporary values for Lesson 2.
        # We will measure/calibrate them properly in Lesson 3.
        self.declare_parameter('wheel_diameter', 0.06242)
        self.declare_parameter('wheel_separation', 0.12385)

        self.declare_parameter('command_rate_hz', 20.0)
        self.declare_parameter('ip_period_s', 5.0)

        self.declare_parameter('max_wheel_rpm', 150.0)
        self.declare_parameter('cmd_vel_timeout_s', 0.5, )
        
        self.cmd_vel_timeout_s = float(
            self.get_parameter('cmd_vel_timeout_s').value
            )

        self.serial_port = str(
            self.get_parameter('serial_port').value
        )

        self.baud_rate = int(
            self.get_parameter('baud_rate').value
        )

        self.wheel_diameter = float(
            self.get_parameter('wheel_diameter').value
        )

        self.wheel_separation = float(
            self.get_parameter('wheel_separation').value
        )

        self.command_rate_hz = float(
            self.get_parameter('command_rate_hz').value
        )

        self.ip_period_s = float(
            self.get_parameter('ip_period_s').value
        )

        self.max_wheel_rpm = float(
            self.get_parameter('max_wheel_rpm').value
        )

        # ============================================================
        # Internal state
        # ============================================================

        
        self.target_left_rpm = 0.0
        self.target_right_rpm = 0.0

        self.running = True

        self.serial_lock = threading.Lock()

        # Feedback state.
        self.mcu_ms = 0

        self.left_pos_deg = 0.0
        self.left_vel_rpm = 0.0

        self.right_pos_deg = 0.0
        self.right_vel_rpm = 0.0

        self.status = 0

        self.last_cmd_vel_time = (
            time.monotonic()
        )

        self.cmd_vel_received = False

        # ============================================================
        # Odometry state
        # ============================================================

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        self.odom_linear_velocity = 0.0
        self.odom_angular_velocity = 0.0

        self.previous_left_pos_deg = None
        self.previous_right_pos_deg = None

        self.previous_odom_time = None
        # ============================================================
        # Serial
        # ============================================================

        try:
            self.serial = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=0.05,
            )

        except serial.SerialException as exc:
            self.get_logger().fatal(
                f'Failed to open {self.serial_port}: {exc}'
            )
            raise

        self.get_logger().info(
            f'Serial opened: '
            f'{self.serial_port} @ {self.baud_rate}'
        )

        # ============================================================
        # ROS interface
        # ============================================================

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10,
        )

        self.odom_pub = self.create_publisher(
                Odometry,
                '/odom',
                20,
        )

        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            20,
        )

        self.tf_broadcaster = (
                TransformBroadcaster(self)
        )

        # 20 Hz UART command timer.
        self.command_timer = self.create_timer(
            1.0 / self.command_rate_hz,
            self.command_timer_callback,
        )

        # Rock IP heartbeat.
        self.ip_timer = self.create_timer(
            self.ip_period_s,
            self.ip_timer_callback,
        )

        # Debug output.
        self.debug_timer = self.create_timer(
            1.0,
            self.debug_timer_callback,
        )

        # ============================================================
        # RX thread
        # ============================================================

        self.rx_thread = threading.Thread(
            target=self.serial_rx_loop,
            daemon=True,
        )

        self.rx_thread.start()

        # Send IP immediately.
        self.send_ip()

        # Safe initial stop.
        self.send_velocity_command(
            0.0,
            0.0,
        )

        self.get_logger().info(
            'ros_car_base ready'
        )

        # ================================================================
        # Differential drive conversion
        # ================================================================

    def cmd_vel_callback(
        self,
        msg: Twist,
    ) -> None:

        self.last_cmd_vel_time = (
            time.monotonic()
        )

        self.cmd_vel_received = True

        linear_x = float(
            msg.linear.x
        )

        angular_z = float(
            msg.angular.z
        )

        # Standard differential-drive kinematics:
        #
        # v_left  = v - w * L/2
        # v_right = v + w * L/2
        #
        # where:
        # v = robot linear velocity [m/s]
        # w = robot angular velocity [rad/s]
        # L = wheel separation [m]

        left_mps = (
            linear_x
            - angular_z
            * self.wheel_separation
            / 2.0
        )

        right_mps = (
            linear_x
            + angular_z
            * self.wheel_separation
            / 2.0
        )

        wheel_circumference = (
            math.pi
            * self.wheel_diameter
        )

        left_rpm = (
            left_mps
            / wheel_circumference
            * 60.0
        )

        right_rpm_robot = (
            right_mps
            / wheel_circumference
            * 60.0
        )

        # Physical motor mounting:
        #
        # Left forward  = positive motor RPM
        # Right forward = negative motor RPM

        right_rpm = (
            -right_rpm_robot
        )

        self.target_left_rpm = (
            self.clamp_rpm(left_rpm)
        )

        self.target_right_rpm = (
            self.clamp_rpm(right_rpm)
        )

    def clamp_rpm(
        self,
        value: float,
    ) -> float:
        return max(
            -self.max_wheel_rpm,
            min(
                self.max_wheel_rpm,
                value,
            ),
        )

    # ================================================================
    # UART TX
    # ================================================================

    def command_timer_callback(
        self,
    ) -> None:

        if self.cmd_vel_received:
            age = (
                time.monotonic()
                - self.last_cmd_vel_time
            )

            if age > self.cmd_vel_timeout_s:
                self.target_left_rpm = 0.0
                self.target_right_rpm = 0.0

        self.send_velocity_command(
            self.target_left_rpm,
            self.target_right_rpm,
        )

    def send_velocity_command(
        self,
        left_rpm: float,
        right_rpm: float,
    ) -> None:
        left_raw = round(
            left_rpm * 100.0
        )

        right_raw = round(
            right_rpm * 100.0
        )

        frame = (
            f'V,{left_raw},{right_raw}\n'
        )

        self.serial_write(
            frame
        )

    def send_ip(
        self,
    ) -> None:
        ip = self.get_ipv4()

        frame = (
            f'IP,{ip}\n'
        )

        self.serial_write(
            frame
        )

        self.get_logger().info(
            f'Rock IP: {ip}'
        )

    def ip_timer_callback(
        self,
    ) -> None:
        self.send_ip()

    def serial_write(
        self,
        frame: str,
    ) -> None:
        try:
            with self.serial_lock:
                self.serial.write(
                    frame.encode('ascii')
                )

        except serial.SerialException as exc:
            self.get_logger().error(
                f'UART TX failed: {exc}'
            )

    @staticmethod
    def get_ipv4() -> str:
        output = os.popen(
            'hostname -I'
        ).read().strip()

        for token in output.split():
            if (
                token.count('.') == 3
                and not token.startswith('127.')
            ):
                return token

        return '192.168.xxx.xxx'

    # ================================================================
    # UART RX
    # ================================================================

    def serial_rx_loop(
        self,
    ) -> None:
        while self.running:
            try:
                raw = self.serial.readline()

            except serial.SerialException as exc:
                self.get_logger().error(
                    f'UART RX failed: {exc}'
                )
                time.sleep(0.1)
                continue

            if not raw:
                continue

            line = raw.decode(
                'ascii',
                errors='replace',
            ).strip()

            if not line:
                continue

            if line.startswith('FB,'):
                self.parse_feedback(
                    line
                )

            elif line.startswith('DBG,'):
                self.get_logger().debug(
                    line
                )

            else:
                self.get_logger().info(
                    f'MCU: {line}'
                )

    def parse_feedback(
        self,
        line: str,
    ) -> None:
        parts = line.split(',')

        if len(parts) != 7:
            self.get_logger().warning(
                f'Invalid FB frame: {line}'
            )
            return

        try:
            self.mcu_ms = int(
                parts[1]
            )

            self.left_pos_deg = (
                int(parts[2])
                / 100.0
            )

            self.left_vel_rpm = (
                int(parts[3])
                / 100.0
            )

            self.right_pos_deg = (
                int(parts[4])
                / 100.0
            )

            self.right_vel_rpm = (
                int(parts[5])
                / 100.0
            )

            self.status = int(
                parts[6]
            )
            
            self.update_odometry()
            self.publish_joint_states()
        except ValueError:
            self.get_logger().warning(
                f'Invalid FB values: {line}'
            )

    # ================================================================
    # Debug
    # ================================================================

    def debug_timer_callback(
        self,
    ) -> None:
        self.get_logger().info(
            'TARGET '
            f'L={self.target_left_rpm:+.1f} '
            f'R={self.target_right_rpm:+.1f} rpm | '
            'ACT '
            f'L={self.left_vel_rpm:+.1f} '
            f'R={self.right_vel_rpm:+.1f} rpm | '
            'ODOM '
            f'x={self.odom_x:+.3f} '
            f'y={self.odom_y:+.3f} '
            f'yaw={math.degrees(self.odom_yaw):+.1f}deg | '
            f'status=0x{self.status:02X}'
        )

    # ================================================================
    # Shutdown
    # ================================================================

    def stop_robot(
        self,
    ) -> None:
        self.target_left_rpm = 0.0
        self.target_right_rpm = 0.0

        # Send several explicit stops.
        for _ in range(5):
            self.send_velocity_command(
                0.0,
                0.0,
            )

            time.sleep(0.02)

    def destroy_node(
        self,
    ) -> bool:
        self.get_logger().info(
            'Stopping base...'
        )

        self.stop_robot()

        self.running = False

        if self.serial.is_open:
            self.serial.close()

        return super().destroy_node()

    @staticmethod
    def yaw_to_quaternion(
        yaw: float,
    ) -> tuple[float, float, float, float]:

        half_yaw = yaw * 0.5

        qx = 0.0
        qy = 0.0

        qz = math.sin(
            half_yaw
        )

        qw = math.cos(
            half_yaw
        )

        return (
            qx,
            qy,
            qz,
            qw,
        )

    def update_odometry(
        self,
    ) -> None:

        # ------------------------------------------------------------
        # Convert raw motor coordinates to robot wheel coordinates.
        #
        # Robot convention:
        # forward rotation is positive for BOTH wheels.
        # ------------------------------------------------------------

        left_pos_deg = (
            self.left_pos_deg
        )

        right_pos_deg = (
            -self.right_pos_deg
        )

        # ------------------------------------------------------------
        # First feedback frame only establishes baseline.
        # ------------------------------------------------------------

        if (
            self.previous_left_pos_deg is None
            or self.previous_right_pos_deg is None
        ):
            self.previous_left_pos_deg = (
                left_pos_deg
            )

            self.previous_right_pos_deg = (
                right_pos_deg
            )

            self.previous_odom_time = (
                time.monotonic()
            )

            return

        # ------------------------------------------------------------
        # Encoder angular increments [degree].
        # ------------------------------------------------------------

        delta_left_deg = (
            left_pos_deg
            - self.previous_left_pos_deg
        )

        delta_right_deg = (
            right_pos_deg
            - self.previous_right_pos_deg
        )

        self.previous_left_pos_deg = (
            left_pos_deg
        )

        self.previous_right_pos_deg = (
            right_pos_deg
        )

        # ------------------------------------------------------------
        # Degrees -> radians.
        # ------------------------------------------------------------

        delta_left_rad = math.radians(
            delta_left_deg
        )

        delta_right_rad = math.radians(
            delta_right_deg
        )

        wheel_radius = (
            self.wheel_diameter
            / 2.0
        )

        # ------------------------------------------------------------
        # Wheel angular displacement -> linear travel [m].
        # ------------------------------------------------------------

        delta_left_m = (
            delta_left_rad
            * wheel_radius
        )

        delta_right_m = (
            delta_right_rad
            * wheel_radius
        )

        # ------------------------------------------------------------
        # Differential-drive displacement.
        #
        # ds:
        # distance travelled by robot centre.
        #
        # dtheta:
        # robot heading change.
        # ------------------------------------------------------------

        delta_distance = (
            delta_right_m
            + delta_left_m
        ) / 2.0

        delta_yaw = (
            delta_right_m
            - delta_left_m
        ) / self.wheel_separation

        # ------------------------------------------------------------
        # Midpoint integration.
        #
        # Use yaw at the middle of this small movement interval.
        # More accurate than simply using old yaw.
        # ------------------------------------------------------------

        heading_mid = (
            self.odom_yaw
            + delta_yaw / 2.0
        )

        self.odom_x += (
            delta_distance
            * math.cos(heading_mid)
        )

        self.odom_y += (
            delta_distance
            * math.sin(heading_mid)
        )

        self.odom_yaw += (
            delta_yaw
        )

        # Keep yaw bounded.
        self.odom_yaw = math.atan2(
            math.sin(self.odom_yaw),
            math.cos(self.odom_yaw),
        )

        # ------------------------------------------------------------
        # Calculate measured chassis velocities.
        # ------------------------------------------------------------

        now = time.monotonic()

        if self.previous_odom_time is not None:
            dt = (
                now
                - self.previous_odom_time
            )

            if dt > 0.0001:
                self.odom_linear_velocity = (
                    delta_distance
                    / dt
                )

                self.odom_angular_velocity = (
                    delta_yaw
                    / dt
                )

        self.previous_odom_time = now

        # ------------------------------------------------------------
        # Publish ROS odometry + TF.
        # ------------------------------------------------------------

        self.publish_odometry()
        
        # ================================================================
        # Odometry Publisher
        # ================================================================

    def publish_odometry(
        self,
    ) -> None:
        stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        (
            qx,
            qy,
            qz,
            qw,
        ) = self.yaw_to_quaternion(
            self.odom_yaw
        )

        # ============================================================
        # nav_msgs/Odometry
        # ============================================================

        odom_msg = Odometry()

        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = 'odom'

        odom_msg.child_frame_id = (
            'base_link'
        )

        # Robot position in odom frame.
        odom_msg.pose.pose.position.x = (
            self.odom_x
        )

        odom_msg.pose.pose.position.y = (
            self.odom_y
        )

        odom_msg.pose.pose.position.z = 0.0

        # Robot orientation.
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        # Measured robot velocity.
        odom_msg.twist.twist.linear.x = (
            self.odom_linear_velocity
        )

        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0

        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0

        odom_msg.twist.twist.angular.z = (
            self.odom_angular_velocity
        )

        self.odom_pub.publish(
            odom_msg
        )

        # ============================================================
        # TF: odom -> base_link
        # ============================================================

        transform = TransformStamped()

        transform.header.stamp = stamp

        transform.header.frame_id = (
            'odom'
        )

        transform.child_frame_id = (
            'base_link'
        )

        transform.transform.translation.x = (
            self.odom_x
        )

        transform.transform.translation.y = (
            self.odom_y
        )

        transform.transform.translation.z = 0.0

        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(
            transform
        )


        # ================================================================
    
    # Joint State Publisher
    # ================================================================

    def publish_joint_states(
        self,
    ) -> None:

        msg = JointState()

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.name = [
            'left_wheel_joint',
            'right_wheel_joint',
        ]

        # Motor coordinate -> robot wheel coordinate
        #
        # Left forward  = +
        # Right forward = +
        #
        # Raw right motor encoder is mirrored,
        # therefore the right side is negated.

        msg.position = [
            math.radians(
                self.left_pos_deg
            ),

            math.radians(
                -self.right_pos_deg
            ),
        ]

        # rpm -> rad/s

        msg.velocity = [
            (
                self.left_vel_rpm
                * 2.0
                * math.pi
                / 60.0
            ),

            (
                -self.right_vel_rpm
                * 2.0
                * math.pi
                / 60.0
            ),
        ]

        self.joint_state_pub.publish(
            msg
        )
def main(
    args=None,
) -> None:
    rclpy.init(
        args=args
    )

    node = BaseDriver()

    try:
        rclpy.spin(
            node
        )

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
