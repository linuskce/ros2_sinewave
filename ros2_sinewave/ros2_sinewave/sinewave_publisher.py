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

        # Retrieve parameters (fallback to default values if parameter not found)
        self.amplitude = getattr(self.params, 'amplitude', 1.0)
        if not hasattr(self.params, 'amplitude'):
            self.get_logger().warning("Parameter 'amplitude' not found. Defaulting to 1.0")
            
        self.frequency = getattr(self.params, 'publisher_frequency', 10.0)
        if not hasattr(self.params, 'publisher_frequency'):
            self.get_logger().warning("Parameter 'publisher_frequency' not found. Defaulting to 10.0 Hz")
            
        self.phase = getattr(self.params, 'phase', 0.0)
        if not hasattr(self.params, 'phase'):
            self.get_logger().warning("Parameter 'phase' not found. Defaulting to 0.0")
            
        # Publisher setup
        self.publisher_ = self.create_publisher(Float32, 'sine_wave', 10)
        
        timer_period = 0.01  
        self.timer = self.create_timer(timer_period, self.publish_sine_wave)

        self.time = 0.0
        self.get_logger().info(f'SineWavePublisher started with A={self.amplitude}, f={self.frequency}, phi={self.phase}')
        
    def publish_sine_wave(self):
        # Compute sinewave value
        try:
            value = self.amplitude * math.sin(2.0 * math.pi * self.frequency * self.time + self.phase)
        except Exception as e:
            self.get_logger().error(f"Error computing sine value: {e}")
            return 
    
        msg = Float32()
        msg.data = value
        
        try:
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing: {value:.4f}')
        except Exception as e:
            self.get_logger().error(f"Error publishing message: {e}")
        
        dt = 0.001
        self.time += dt
        
def main(args=None):
    rclpy.init(args=args)
    node = SineWavePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



