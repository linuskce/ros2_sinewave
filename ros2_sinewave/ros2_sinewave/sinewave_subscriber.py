#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import os 
import cv2

# Import the custom service definition
from ros2_sinewave_interfaces.srv import ConvertImage


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
        
        # Create a service server for ConvertImage
        self.srv = self.create_service(
            ConvertImage,
            'convert_image', 
            self.call_image_converting_service
        )
        
        
    def listener_callback(self, msg):
        # Log the received sine wave value
        self.get_logger().info(f"Received: {msg.data:.3f}")
        
    def call_image_converting_service(self, request, response):
        """
        Service callback for converting an image to grayscale.
        request: contains request.image_path
        response: to fill response.grayscale_image_path
        """
        input_path = request.image_path
        if not os.path.exists(input_path):
            self.get_logger().error(f"File not found: {input_path}")
            response.grayscale_image_path = ""
            return response
        
        try:
            img = cv2.imread(input_path)
            if img is None:
                raise ValueError("Image could not be loaded (imread returned None)")
        except Exception as e:
            error_msg = f"Error loading image: {e}"
            self.get_logger().error(error_msg)
            response.grayscale_image_path = error_msg
            return response

        # Convert image to grayscale
        try: 
            gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        except Exception as e: 
            error_msg = f"Error converting image to grayscale: {e}"
            self.get_logger().error(error_msg)
            response.grayscale_image_path = error_msg
            return response
        
        # Define the new filepath
        base, ext = os.path.splitext(input_path)
        gray_path = base + "_grayscale" + ext
        
        # Save grayscale image
        try:
            cv2.imwrite(gray_path, gray_img)
        except Exception as e:
            error_msg = f"Error writing grayscale image: {e}"
            self.get_logger().error(error_msg)
            response.grayscale_image_path = error_msg
            return response
            
        self.get_logger().info(f"Saved grayscale image to: {gray_path}")
        
        # Fill the response
        response.grayscale_image_path = gray_path
        return response
  
    

def main(args=None):
    rclpy.init(args=args)
    node = SineWaveSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
