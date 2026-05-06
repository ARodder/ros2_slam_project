#!/usr/bin/env python3

"""
calibrate_linear.py

ROS2 port of calibrate_linear.py:
Move the robot a specified distance to check the PID parameters of the base controller.

Created by Aleksander Røder using perplexity.ai with the Gemini 3.1 pro model.
Using the ROS 1 calibrate_linear.py as a source from https://github.com/waveshare/jetracer_ros/blob/main/scripts/calibrate_linear.py
"""

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Point
from tf2_ros import Buffer, TransformListener, TransformException

from rcl_interfaces.msg import SetParametersResult


class CalibrateLinear(Node):
    def __init__(self):
        # Initialize node
        super().__init__('calibrate_linear')

        # Declare parameters (ROS2 style)
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('test_distance', 1.0)
        self.declare_parameter('speed', 0.3)
        self.declare_parameter('tolerance', 0.03)
        self.declare_parameter('odom_linear_scale_correction', 1.0)
        self.declare_parameter('start_test', True)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')

        # Get initial parameter values
        self.rate = self.get_parameter('rate').get_parameter_value().double_value
        self.test_distance = self.get_parameter('test_distance').get_parameter_value().double_value
        self.speed = self.get_parameter('speed').get_parameter_value().double_value
        self.tolerance = self.get_parameter('tolerance').get_parameter_value().double_value
        self.odom_linear_scale_correction = (
            self.get_parameter('odom_linear_scale_correction').get_parameter_value().double_value
        )
        self.start_test = self.get_parameter('start_test').get_parameter_value().bool_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value

        # Parameter callback to emulate dynamic_reconfigure
        self.add_on_set_parameters_callback(self.parameters_callback)

        # Publisher
        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', 5)

        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State
        self.position = Point()
        self.x_start = 0.0
        self.y_start = 0.0

        # Give tf some time to buffer
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=2.0))

        self.get_logger().info('Waiting for TF transform between %s and %s' %
                               (self.odom_frame, self.base_frame))
        # No direct equivalent of waitForTransform in tf2_ros Python, so just try in the timer loop.

        self.get_logger().info('Use rqt (Plugins -> Configuration -> Parameters) to tune parameters.')

        # Initialize starting position
        initial_pos = self.get_position()
        if initial_pos is not None:
            self.x_start = initial_pos.x
            self.y_start = initial_pos.y

        # Timer for control loop
        period = 1.0 / self.rate if self.rate > 0.0 else 0.05
        self.timer = self.create_timer(period, self.update)

    def parameters_callback(self, params):
        # Update internal variables when parameters are changed at runtime
        for p in params:
            if p.name == 'test_distance':
                self.test_distance = p.value
            elif p.name == 'speed':
                self.speed = p.value
            elif p.name == 'tolerance':
                self.tolerance = p.value
            elif p.name == 'odom_linear_scale_correction':
                self.odom_linear_scale_correction = p.value
            elif p.name == 'start_test':
                self.start_test = p.value
        return SetParametersResult(successful=True)

    def get_position(self):
        try:
            # Lookup transform from odom_frame to base_frame
            # Time 0 means "latest" in ROS2 as well
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().warn(f'TF Exception: {ex}')
            return None

        # transform.transform.translation is geometry_msgs/Vector3
        trans = transform.transform.translation
        return Point(x=trans.x, y=trans.y, z=trans.z)

    def update(self):
        # Main loop executed by timer
        move_cmd = Twist()

        if self.start_test:
            pos = self.get_position()
            if pos is None:
                # Cannot compute; keep robot stopped
                self.cmd_vel.publish(move_cmd)
                return

            # Compute distance from start
            distance = math.sqrt((pos.x - self.x_start)**2 + (pos.y - self.y_start)**2)
            distance *= self.odom_linear_scale_correction

            error = distance - self.test_distance

            if not self.start_test or abs(error) < self.tolerance:
                # Stop test
                self.start_test = False
                # Update parameter so GUI reflects it
                self.set_parameters([rclpy.parameter.Parameter(
                    'start_test',
                    rclpy.parameter.Parameter.Type.BOOL,
                    False
                )])
                self.get_logger().info('Finished test, error = %.4f m' % error)
            else:
                move_cmd.linear.x = math.copysign(self.speed, -1.0 * error)
        else:
            # Not running test: reset starting point to current pose
            pos = self.get_position()
            if pos is not None:
                self.x_start = pos.x
                self.y_start = pos.y

        self.cmd_vel.publish(move_cmd)

    def destroy_node(self):
        # Stop robot when shutting down
        self.get_logger().info('Stopping the robot...')
        self.cmd_vel.publish(Twist())
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CalibrateLinear()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
