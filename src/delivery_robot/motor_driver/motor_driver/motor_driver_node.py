"""
ROS2 node for driving 4 mecanum wheels via 2x L298N motor driver boards.

Subscribes to /cmd_vel (geometry_msgs/Twist) and converts omnidirectional
velocity commands into individual wheel PWM signals using mecanum inverse
kinematics.  Publishes /motor_status with per-wheel duty cycles.

Uses lgpio for Pi 5 GPIO control.
"""

import lgpio
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from delivery_robot_msgs.msg import MotorStatus


class Motor:
    """Single DC motor controlled by an L298N H-bridge channel."""

    def __init__(self, chip, en_pin: int, in1_pin: int, in2_pin: int,
                 pwm_freq: int = 1000):
        self.chip = chip
        self.en = en_pin
        self.in1 = in1_pin
        self.in2 = in2_pin
        self.duty = 0.0
        self.pwm_freq = pwm_freq

        for pin in (in1_pin, in2_pin, en_pin):
            try:
                lgpio.gpio_claim_output(chip, pin)
            except lgpio.error:
                lgpio.gpio_free(chip, pin)
                lgpio.gpio_claim_output(chip, pin)

    def set_speed(self, speed: float, min_duty: float = 0.0):
        """Set motor speed from -1.0 (full reverse) to 1.0 (full forward).

        If min_duty > 0, any non-zero command is scaled into the
        [min_duty, 1.0] range so the motor always gets enough voltage
        to overcome static friction.
        """
        speed = max(-1.0, min(1.0, speed))
        self.duty = speed

        if abs(speed) < 0.01:
            lgpio.gpio_write(self.chip, self.in1, 0)
            lgpio.gpio_write(self.chip, self.in2, 0)
            lgpio.tx_pwm(self.chip, self.en, self.pwm_freq, 0)
        else:
            pwm = min_duty + abs(speed) * (1.0 - min_duty)
            if speed > 0:
                lgpio.gpio_write(self.chip, self.in1, 1)
                lgpio.gpio_write(self.chip, self.in2, 0)
            else:
                lgpio.gpio_write(self.chip, self.in1, 0)
                lgpio.gpio_write(self.chip, self.in2, 1)
            lgpio.tx_pwm(self.chip, self.en, self.pwm_freq, pwm * 100.0)

    def stop(self):
        self.set_speed(0.0)

    def cleanup(self):
        self.stop()
        lgpio.gpio_free(self.chip, self.en)
        lgpio.gpio_free(self.chip, self.in1)
        lgpio.gpio_free(self.chip, self.in2)


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver')

        self.declare_parameter('pwm_frequency', 1000)
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('min_duty', 0.3)
        self.declare_parameter('watchdog_timeout', 0.5)

        self.declare_parameter('front_left.en', 12)
        self.declare_parameter('front_left.in1', 5)
        self.declare_parameter('front_left.in2', 6)
        self.declare_parameter('front_right.en', 13)
        self.declare_parameter('front_right.in1', 16)
        self.declare_parameter('front_right.in2', 19)
        self.declare_parameter('rear_left.en', 18)
        self.declare_parameter('rear_left.in1', 20)
        self.declare_parameter('rear_left.in2', 21)
        self.declare_parameter('rear_right.en', 23)
        self.declare_parameter('rear_right.in1', 24)
        self.declare_parameter('rear_right.in2', 25)

        # Pi 5 uses gpiochip4 (RP1) for the 40-pin header
        self.declare_parameter('gpio_chip', 4)
        self.gpio_chip_num = self.get_parameter('gpio_chip').value

        self.pwm_freq = self.get_parameter('pwm_frequency').value
        self.max_speed = self.get_parameter('max_speed').value
        self.min_duty = self.get_parameter('min_duty').value
        self.watchdog_timeout = self.get_parameter('watchdog_timeout').value

        try:
            self.chip = lgpio.gpiochip_open(self.gpio_chip_num)
        except Exception as e:
            self.get_logger().error(f'Failed to open GPIO chip: {e}')
            self.chip = None

        self.motors = {}
        if self.chip is not None:
            for name in ('front_left', 'front_right', 'rear_left', 'rear_right'):
                en = self.get_parameter(f'{name}.en').value
                in1 = self.get_parameter(f'{name}.in1').value
                in2 = self.get_parameter(f'{name}.in2').value
                self.motors[name] = Motor(self.chip, en, in1, in2, self.pwm_freq)
                self.get_logger().info(f'{name}: en={en} in1={in1} in2={in2}')

        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self._cmd_vel_callback, 10
        )
        self.status_pub = self.create_publisher(MotorStatus, 'motor_status', 10)

        self._last_cmd_time = self.get_clock().now()
        self.create_timer(0.1, self._watchdog_check)
        self.create_timer(0.2, self._publish_status)

        self.get_logger().info('Motor driver ready')

    def _cmd_vel_callback(self, msg: Twist):
        self._last_cmd_time = self.get_clock().now()

        vx = msg.linear.x * self.max_speed
        vy = msg.linear.y * self.max_speed
        omega = msg.angular.z * self.max_speed

        # Mecanum inverse kinematics
        fl = vx - vy - omega
        fr = vx + vy + omega
        rl = vx + vy - omega
        rr = vx - vy + omega

        # Normalize so no wheel exceeds 1.0
        max_val = max(abs(fl), abs(fr), abs(rl), abs(rr), 1.0)
        fl /= max_val
        fr /= max_val
        rl /= max_val
        rr /= max_val

        if self.motors:
            self.motors['front_left'].set_speed(fl, self.min_duty)
            self.motors['front_right'].set_speed(fr, self.min_duty)
            self.motors['rear_left'].set_speed(rl, self.min_duty)
            self.motors['rear_right'].set_speed(rr, self.min_duty)

    def _watchdog_check(self):
        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        if elapsed > self.watchdog_timeout:
            for m in self.motors.values():
                m.stop()

    def _publish_status(self):
        msg = MotorStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        duties = [0.0] * 4
        if self.motors:
            duties = [
                self.motors['front_left'].duty,
                self.motors['front_right'].duty,
                self.motors['rear_left'].duty,
                self.motors['rear_right'].duty,
            ]
        for i in range(4):
            msg.duty_cycles[i] = float(duties[i])
            msg.active[i] = bool(abs(duties[i]) > 0.01)
        msg.mode = 'manual'
        self.status_pub.publish(msg)

    def destroy_node(self):
        for m in self.motors.values():
            m.cleanup()
        if self.chip is not None:
            lgpio.gpiochip_close(self.chip)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
