import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import time

class CartpoleOscillator(Node):
    def __init__(self):
        super().__init__('cartpole_oscillator')
        
        # Control parameters
        self.amplitude = 7.0     # Hareket genliği (kuvvet)
        self.frequency = 0.5     # Hareket frekansı (Hz)
        
        # Effort controller publisher
        self.effort_pub = self.create_publisher(
            Float64MultiArray,
            'effort_controllers/commands',
            10)
        
        # Timer for periodic control
        self.timer = self.create_timer(0.02, self.control_callback)  # 50Hz
        self.start_time = time.time()
        
        self.get_logger().info('Cartpole oscillator started - cart will move back and forth')

    def control_callback(self):
        # Get elapsed time
        elapsed_time = time.time() - self.start_time
        
        # Generate sinusoidal control signal
        effort = self.amplitude * math.sin(2 * math.pi * self.frequency * elapsed_time)
        
        # Publish effort command
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [float(effort)]
        self.effort_pub.publish(cmd_msg)
        
        # Log
        self.get_logger().info(f'Sending effort command: {effort:6.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = CartpoleOscillator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()