import rclpy
from rclpy.node import node
import os
import subprocess

from mavros_msgs.msg import State
from rclpy.qos import qos_profile_sensor_data

class BagControl(Node):
    def __init__(self):
        super().__init__('bag_control')
        self.sub = self.create_subscription(
            State, '~/state', self.state_callback,
            qos_profile_sensor_data
        )
        self.declare_params()
        self.logging = False
        self.log_timeout_active = False

    def declare_params(self):
        self.declare_parameter('bag_start_command', '')
        self.declare_parameter('bag_stop_command', '')
        self.declare_parameter('bag_stop_delay', '0')

    def state_callback(self, msg):
        if not self.logging:
            if msg.armed:
                os.system(self.get_parameter('bag_start_command').get_parameter_value().string_value)
                self.logging = True
        elif self.logging and not self.log_timeout_active:
            if not msg.armed:
                self.log_timeout_active = True
                self.log_stop_timer = self.create_timer(
                    self.get_parameter('bag_stop_delay').get_parameter_value().float_value,
                    timer_callback)
        elif self.logging and self.log_timeout_active:
            if msg.armed:
                self.destroy_timer(self.log_stop_timer)
                self.log_timeout_active = False

    def timer_callback(self):
        os.system(self.get_parameter('bag_stop_command').get_parameter_value().string_value)
        self.logging = False
        self.log_timeout_active = False
        self.destroy_timer(self.log_stop_timer)

def main(args=None):
    rclpy.init(args=args)
    control = BagControl()
    rclpy.spin(control)
    control.destroy_node()
    rclpy.shutdown()