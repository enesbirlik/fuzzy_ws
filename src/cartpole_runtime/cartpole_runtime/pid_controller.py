#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class DualPIDController(Node):
    def __init__(self):
        super().__init__('dual_pid_controller')
        # Subscribe to joint states to get both pole and cart information.
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        # Publisher for the cart effort command.
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/cart_effort_controller/commands',
            10
        )
        
        # --- PID gains for the pole (angle control) ---
        self.kp_pole = 19.062033096720352
        self.ki_pole = 5.484786430685422
        self.kd_pole = 13.801040593233569
        self.desired_pole_angle = 0.0  # Target: pole upright (0 rad)
        
        # --- PID gains for the cart (position control) ---
        self.kp_cart = 7.770961990449907
        self.ki_cart = 18.398947695652165
        self.kd_cart = 19.90339301440841
        self.desired_cart_position = 0.0  # Target cart position
        
        # State variables for pole measurements.
        self.current_pole_angle = None
        self.current_pole_velocity = None
        
        # State variables for cart measurements.
        self.current_cart_position = None
        self.current_cart_velocity = None
        
        # Integral and last error terms for each controller.
        self.pole_integral = 0.0
        self.pole_last_error = 0.0
        
        self.cart_integral = 0.0
        self.cart_last_error = 0.0
        
        self.last_time = self.get_clock().now().nanoseconds / 1e9
        
        # Run control loop at 100 Hz.
        self.timer = self.create_timer(0.005, self.control_loop)
    
    def joint_state_callback(self, msg: JointState):
        # Get the pole state.
        if "pole_joint" in msg.name:
            idx = msg.name.index("pole_joint")
            self.current_pole_angle = msg.position[idx]
            self.current_pole_velocity = msg.velocity[idx]
        
        # Get the cart state.
        if "cart_joint" in msg.name:
            idx = msg.name.index("cart_joint")
            self.current_cart_position = msg.position[idx]
            self.current_cart_velocity = msg.velocity[idx]
    
    def control_loop(self):
        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - self.last_time
        if dt <= 0:
            return
        # Ensure we have received both measurements.
        if self.current_pole_angle is None or self.current_cart_position is None:
            return

        # --- Pole PID (angle control) ---
        error_pole = self.desired_pole_angle - self.current_pole_angle
        self.pole_integral += error_pole * dt
        derivative_pole = (error_pole - self.pole_last_error) / dt
        output_pole = self.kp_pole * error_pole + self.ki_pole * self.pole_integral + self.kd_pole * derivative_pole
        self.pole_last_error = error_pole
        
        # --- Cart PID (position control) ---
        error_cart = self.desired_cart_position - self.current_cart_position
        self.cart_integral += error_cart * dt
        derivative_cart = (error_cart - self.cart_last_error) / dt
        output_cart = self.kp_cart * error_cart + self.ki_cart * self.cart_integral + self.kd_cart * derivative_cart
        self.cart_last_error = error_cart
        
        # Combine the outputs (using a negative sign as in your original code).
        effort = -(output_pole + output_cart)
        
        # Publish the command.
        msg = Float64MultiArray()
        msg.data = [effort]
        self.publisher.publish(msg)
        
        self.get_logger().info(
            f"Pole Angle: {self.current_pole_angle:.3f}, Pole Err: {error_pole:.3f}, "
            f"Cart Pos: {self.current_cart_position:.3f}, Cart Err: {error_cart:.3f}, Effort: {effort:.3f}"
        )
        self.last_time = now

def main(args=None):
    rclpy.init(args=args)
    node = DualPIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
