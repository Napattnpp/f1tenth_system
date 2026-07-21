# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64

class ThrottleInterpolator(Node):
    def __init__(self):
        super().__init__('throttle_interpolator')

        self.declare_parameter('rpm_input_topic', 'commands/motor/unsmoothed_speed')
        self.declare_parameter('rpm_output_topic', 'commands/motor/speed')
        self.declare_parameter('servo_input_topic', 'commands/servo/unsmoothed_position')
        self.declare_parameter('servo_output_topic', 'commands/servo/position')
        self.declare_parameter('max_acceleration', 2.5)
        self.declare_parameter('speed_max', 23250.0)
        self.declare_parameter('speed_min', -23250.0)
        self.declare_parameter('throttle_smoother_rate', 75.0)
        self.declare_parameter('speed_to_erpm_gain', 4614.0)
        self.declare_parameter('max_servo_speed', 3.2)
        self.declare_parameter('steering_angle_to_servo_gain', -1.2135)
        self.declare_parameter('servo_smoother_rate', 75.0)
        self.declare_parameter('servo_max', 0.85)
        self.declare_parameter('servo_min', 0.15)
        self.declare_parameter('steering_angle_to_servo_offset', 0.5304)
        self.declare_parameter('watchdog_timeout', 0.5)

        self.rpm_input_topic = self.get_parameter('rpm_input_topic').value
        self.rpm_output_topic = self.get_parameter('rpm_output_topic').value
        self.servo_input_topic = self.get_parameter('servo_input_topic').value
        self.servo_output_topic = self.get_parameter('servo_output_topic').value
        self.max_acceleration = self.get_parameter('max_acceleration').value
        self.max_rpm = self.get_parameter('speed_max').value
        self.min_rpm = self.get_parameter('speed_min').value
        self.throttle_smoother_rate = self.get_parameter('throttle_smoother_rate').value
        self.speed_to_erpm_gain = self.get_parameter('speed_to_erpm_gain').value
        self.max_servo_speed = self.get_parameter('max_servo_speed').value
        self.steering_angle_to_servo_gain = self.get_parameter('steering_angle_to_servo_gain').value
        self.servo_smoother_rate = self.get_parameter('servo_smoother_rate').value
        self.max_servo = self.get_parameter('servo_max').value
        self.min_servo = self.get_parameter('servo_min').value
        self.steering_angle_to_servo_offset = self.get_parameter('steering_angle_to_servo_offset').value
        self.watchdog_timeout = self.get_parameter('watchdog_timeout').value

        # Calculate a single combined timer rate
        self.timer_rate = max(self.throttle_smoother_rate, self.servo_smoother_rate)

        self.last_rpm = 0.0
        self.desired_rpm = self.last_rpm
        self.last_servo = self.steering_angle_to_servo_offset
        self.desired_servo_position = self.last_servo

        # Track the last time a command was received for the watchdog
        self.last_rpm_time = self.get_clock().now()
        self.last_servo_time = self.get_clock().now()

        self.rpm_output = self.create_publisher(Float64, self.rpm_output_topic, 1)
        self.servo_output = self.create_publisher(Float64, self.servo_output_topic, 1)

        self.rpm_sub = self.create_subscription(
            Float64,
            self.rpm_input_topic,
            self._process_throttle_command,
            1)
        self.servo_sub = self.create_subscription(
            Float64,
            self.servo_input_topic,
            self._process_servo_command,
            1)

        self.max_delta_servo = abs(self.steering_angle_to_servo_gain * self.max_servo_speed / self.timer_rate)
        self.max_delta_rpm = abs(self.speed_to_erpm_gain * self.max_acceleration / self.timer_rate)

        # Combined timer to sync outputs and optimize scheduling
        self.timer = self.create_timer(1.0/self.timer_rate, self._timer_callback)

    def _timer_callback(self):
        now = self.get_clock().now()

        # 1. Watchdog timeout checks
        time_since_last_rpm = (now - self.last_rpm_time).nanoseconds / 1e9
        if time_since_last_rpm > self.watchdog_timeout:
            self.desired_rpm = 0.0

        time_since_last_servo = (now - self.last_servo_time).nanoseconds / 1e9
        if time_since_last_servo > self.watchdog_timeout:
            self.desired_servo_position = self.steering_angle_to_servo_offset

        # 2. Smooth/Interpolate RPM
        desired_delta_rpm = self.desired_rpm - self.last_rpm
        clipped_delta_rpm = max(min(desired_delta_rpm, self.max_delta_rpm), -self.max_delta_rpm)
        smoothed_rpm = self.last_rpm + clipped_delta_rpm
        self.last_rpm = smoothed_rpm

        # 3. Smooth/Interpolate Servo
        desired_delta_servo = self.desired_servo_position - self.last_servo
        clipped_delta_servo = max(min(desired_delta_servo, self.max_delta_servo), -self.max_delta_servo)
        smoothed_servo = self.last_servo + clipped_delta_servo
        self.last_servo = smoothed_servo

        # 4. Publish commands continuously to prevent VESC watchdog timeout
        rpm_msg = Float64()
        rpm_msg.data = float(smoothed_rpm)
        self.rpm_output.publish(rpm_msg)

        servo_msg = Float64()
        servo_msg.data = float(smoothed_servo)
        self.servo_output.publish(servo_msg)

    def _process_throttle_command(self, msg):
        input_rpm = msg.data
        # Do some sanity clipping
        input_rpm = min(max(input_rpm, self.min_rpm), self.max_rpm)
        self.desired_rpm = input_rpm
        self.last_rpm_time = self.get_clock().now()

    def _process_servo_command(self, msg):
        input_servo = msg.data
        # Do some sanity clipping
        input_servo = min(max(input_servo, self.min_servo), self.max_servo)
        # set the target servo position
        self.desired_servo_position = input_servo
        self.last_servo_time = self.get_clock().now()

def main(args=None):
    rclpy.init(args=args)
    p = ThrottleInterpolator()
    rclpy.spin(p)

if __name__ == '__main__':
    main()
