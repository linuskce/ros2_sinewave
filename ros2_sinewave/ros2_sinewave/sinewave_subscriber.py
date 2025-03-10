#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class SineWaveSubscriber(Node):
    def __init__(self):
        super().__init__('sinewave_subscriber')

        # Create subscription to the "sine_wave" topic
        self.subscription = self.create_subscription(
            Float32,
            'sine_wave',
            self.listener_callback,
            10 
        )
        self.subscription 

        self.get_logger().info("SineWaveSubscriber started...")

    def listener_callback(self, msg):
        # Log the received sine wave value
        self.get_logger().info(f"Received: {msg.data:.3f}")


def main(args=None):
    rclpy.init(args=args)
    node = SineWaveSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
