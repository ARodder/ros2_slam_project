#!/usr/bin/env python3
"""
teleop_joy.py

ROS2 port of teleop_joy.py:
Translates joystick input to velocity commands for the JetRacer.

Created by Aleksander Røder using perplexity.ai with the Gemini 3.1 pro model.
Using the ROS 1 teleop_joy.py as a source. The script was retrieved from the original ROS 1 jetson Nano Image.
"""
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class TeleopJoy(Node):
    def __init__(self):
        super().__init__('teleop_joy')

        # Parameters (ROS1 private params "~x_speed" -> ROS2 parameters on this node)
        self.declare_parameter('x_speed', 1)
        self.declare_parameter('y_speed', 0.0)  # kept for compatibility; not used by default mapping
        self.declare_parameter('w_speed', 1.0)
        self.declare_parameter('hz', 20.0)

        self.declare_parameter('joy_topic', 'joy')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        # Mapping (matches your original indices)
        self.declare_parameter('enable_button', 6)
        self.declare_parameter('linear_x_axis', 1)
        self.declare_parameter('angular_z_axis', 2)

        self.x_speed = float(self.get_parameter('x_speed').value)
        self.y_speed = float(self.get_parameter('y_speed').value)
        self.w_speed = float(self.get_parameter('w_speed').value)
        hz = float(self.get_parameter('hz').value)

        self.joy_topic = str(self.get_parameter('joy_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)

        self.enable_button = int(self.get_parameter('enable_button').value)
        self.linear_x_axis = int(self.get_parameter('linear_x_axis').value)
        self.angular_z_axis = int(self.get_parameter('angular_z_axis').value)

        self.active = False
        self.cmd = Twist()

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.joy_sub = self.create_subscription(Joy, self.joy_topic, self.joy_callback, 10)

        period = 1.0 / hz if hz > 0.0 else 0.05
        self.timer = self.create_timer(period, self.timer_callback)

    def timer_callback(self):
        if self.active:
            self.cmd_pub.publish(self.cmd)

    def joy_callback(self, msg: Joy):
        # Defensive checks (avoid index errors if a controller has fewer buttons/axes)
        axes_ok = (self.linear_x_axis < len(msg.axes)) and (self.angular_z_axis < len(msg.axes))

        if axes_ok:
            self.cmd.linear.x = self.x_speed * msg.axes[self.linear_x_axis]
            # Original script didn't use y; left here if you later want to map it:
            # self.cmd.linear.y = self.y_speed * msg.axes[...]
            self.cmd.angular.z = self.w_speed * msg.axes[self.angular_z_axis]
            self.active = True
            self.cmd_pub.publish(self.cmd)  # immediate publish on input, like your ROS1 version
        else:
            self.cmd = Twist()  # zeros
            self.active = False
            self.cmd_pub.publish(self.cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopJoy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
