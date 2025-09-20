#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from serial_msgs.msg import MotorCurrents


class SampleNode(Node):

    def __init__(self):
        super().__init__('test_node')
        self.motor_publisher_ = self.create_publisher(
            msg_type=MotorCurrents,
            topic='motor_currents',
        )
        self.timer_ = self.create_timer(0.5, self.send_motor_currents)
    
    def send_motor_currents(self):
        message = MotorCurrents()
        message.left_wheels = 100
        message.right_wheels = 154
        self.motor_publisher_.publish(message)

    
def main(args=None):
    rclpy.init(args=args)

    node = SampleNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()