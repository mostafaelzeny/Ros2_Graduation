#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        self.subscriber_ = self.create_subscription(Float32, 'diameter', self.listener_callback, 10)
        self.last_time = time.time()

    def listener_callback(self, msg):
        diameter = msg.data
        self.get_logger().info(f"Received diameter: {diameter:.2f}")
        now = time.time()

        if now - self.last_time >= 10:
            if 5 < diameter < 13:
                t = (-0.136 * diameter) + 2.161
                self.get_logger().info("Orange detected! Rotating clockwise...")
                time.sleep(t)
                self.get_logger().info("Stop.")
                time.sleep(2)
                self.get_logger().info("Rotating counterclockwise...")
                time.sleep(t)
                self.get_logger().info("Stop.")
                time.sleep(2)
            else:
                self.get_logger().info("No valid orange. Stop.")
            self.last_time = now

def main(args=None):
    rclpy.init(args=args)
    node = ServoController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
