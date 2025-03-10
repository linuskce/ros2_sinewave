#!/usr/bin/env python3

# imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math

# import of auto-generated parameter module
from .sinewave_parameters import sinewave_parameters

class SineWavePublisher(Node):
    def __init__(self):
        super().__init__('sinewave_publisher')
        
        # Loading of Parameters
        self.param_listener = sinewave_parameters.ParamListener(self)
        self.params = self.param_listener.get_params()

        self.amplitude = self.params.amplitude
        self.frequency = self.params.publisher_frequency
        self.phase = self.params.phase
        
        # Publisher setup
        self.publisher_ = self.create_publisher(Float32, 'sine_wave', 10)
        
        timer_period = 0.01  
        self.timer = self.create_timer(timer_period, self.publish_sine_wave)

        self.time = 0.0
        self.get_logger().info(f'SineWavePublisher started with A={self.amplitude}, f={self.frequency}, φ={self.phase}')
        
    def publish_sine_wave(self):
        value = self.amplitude * math.sin(2.0 * math.pi * self.frequency * self.time + self.phase)
        msg = Float32()
        msg.data = value
        self.publisher_.publish(msg)

        self.get_logger().info(f'Publishing: {value:.3f}')
        
        dt = 0.001
        self.time += dt
        
def main(args=None):
    rclpy.init(args=args)
    node = SineWavePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



