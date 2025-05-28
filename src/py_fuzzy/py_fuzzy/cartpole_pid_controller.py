import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Bool
import threading

class DualPIDController(Node):
    def __init__(self):
        super().__init__('dual_pid_controller')

        # Subscribe to joint states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Publisher for effort commands (single interface)
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/effort_controllers/commands',
            10
        )

        # PID gains for pole
        self.kp_pole = 1500.0
        self.ki_pole = 50.0
        self.kd_pole = 150.0
        self.desired_pole_angle = -math.pi  # Target angle

        # PID gains for cart (unused if single joint)
        self.kp_cart = 100.0
        self.ki_cart = 10.0
        self.kd_cart = 20.0
        self.desired_cart_position = 0.0

        # State storage
        self.current_pole_angle = None
        self.current_cart_position = None
        self.current_cart_velocity = None
        self.pole_integral = 0.0
        self.pole_last_error = 0.0
        self.cart_integral = 0.0
        self.cart_last_error = 0.0

        # Safety parameters
        self.safety_active = False
        self.max_angle_error = math.radians(45)  # 45 degrees in radians
        self.max_cart_position = 0.18  # Cart position limits
        self.min_cart_position = -0.18
        
        # Cart return PID parameters - Increased gains for better performance
        self.cart_return_kp = 200.0  # Increased proportional gain
        self.cart_return_ki = 50.0   # Added integral term
        self.cart_return_kd = 30.0   # Increased derivative gain
        self.cart_return_integral = 0.0  # Cart return integral term
        self.cart_return_last_error = 0.0

        # Initialize timing
        self.last_time = self.get_clock().now().nanoseconds / 1e9
        self.timer = self.create_timer(0.005, self.control_loop)

        # Send initial zero command
        init_cmd = Float64MultiArray()
        init_cmd.data = [0.0]
        self.publisher.publish(init_cmd)

        # Start input thread for reset functionality
        self.input_thread = threading.Thread(target=self.input_handler, daemon=True)
        self.input_thread.start()

        self.get_logger().info('Dual PID Controller started')
        self.get_logger().info('Safety limits: Pole angle ±45°, Cart position ±0.18')
        self.get_logger().info('Press ENTER to reset system if safety mode is active')

    def joint_state_callback(self, msg: JointState):
        try:
            # Use 'slider_to_cart' or correct name
            idx = msg.name.index('slider_to_cart')
            self.current_cart_position = msg.position[idx]
            self.current_cart_velocity = msg.velocity[idx]
            # Use 'cart_to_pole' if needed for angle
            idx2 = msg.name.index('cart_to_pole')
            self.current_pole_angle = msg.position[idx2]
        except ValueError:
            self.get_logger().warn('JointState missing expected names')

    def input_handler(self):
        """Handle user input for system reset"""
        while rclpy.ok():
            try:
                input()  # Wait for ENTER key
                if self.safety_active:
                    self.reset_system()
            except:
                break

    def reset_system(self):
        """Reset system to normal operation"""
        # Only reset if cart is close to zero position (within 0.05m)
        if self.current_cart_position is not None and abs(self.current_cart_position) < 0.05:
            self.safety_active = False
            self.pole_integral = 0.0
            self.pole_last_error = 0.0
            self.cart_integral = 0.0
            self.cart_last_error = 0.0
            self.cart_return_integral = 0.0
            self.cart_return_last_error = 0.0
            self.get_logger().info('System reset - Normal operation resumed')
        else:
            self.get_logger().warn(f'Cannot reset: Cart not at zero position (current: {self.current_cart_position:.3f})')
            self.get_logger().info('Wait for cart to reach zero position before resetting')

    def check_safety_conditions(self):
        """Check if safety conditions are violated"""
        if self.current_pole_angle is None or self.current_cart_position is None:
            return False
        
        angle_error = abs(self.desired_pole_angle - self.current_pole_angle)
        cart_position = self.current_cart_position
        
        safety_triggered = False
        trigger_reason = ""
        
        # Check if angle error exceeds 45 degrees
        if angle_error > self.max_angle_error:
            safety_triggered = True
            trigger_reason = f'Pole angle error {math.degrees(angle_error):.1f}° > 45°'
        
        # Check if cart position exceeds limits
        if cart_position > self.max_cart_position:
            safety_triggered = True
            trigger_reason = f'Cart position {cart_position:.3f} > {self.max_cart_position}'
        elif cart_position < self.min_cart_position:
            safety_triggered = True
            trigger_reason = f'Cart position {cart_position:.3f} < {self.min_cart_position}'
        
        if safety_triggered and not self.safety_active:
            self.safety_active = True
            # Reset cart return controller when entering safety mode
            self.cart_return_integral = 0.0
            self.cart_return_last_error = 0.0
            self.get_logger().warn(f'SAFETY ACTIVATED: {trigger_reason}')
            self.get_logger().info('Motors stopped. Cart will return to zero. Press ENTER to reset when cart reaches zero.')
            return True
        
        return self.safety_active

    def cart_return_control(self, dt):
        """Return cart to zero position using PID control"""
        if self.current_cart_position is None or dt <= 0:
            return 0.0
        
        # PID control for cart return
        cart_error = 0.0 - self.current_cart_position
        self.cart_return_integral += cart_error * dt
        
        # Anti-windup for integral term
        max_integral = 5.0
        self.cart_return_integral = max(min(self.cart_return_integral, max_integral), -max_integral)
        
        cart_derivative = (cart_error - self.cart_return_last_error) / dt
        
        # PID calculation
        effort = (self.cart_return_kp * cart_error + 
                 self.cart_return_ki * self.cart_return_integral + 
                 self.cart_return_kd * cart_derivative)
        
        self.cart_return_last_error = cart_error
        
        # Limit effort for safety but allow sufficient force
        max_return_effort = 300.0  # Increased max effort
        effort = max(min(effort, max_return_effort), -max_return_effort)
        
        return effort

    def control_loop(self):
        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - self.last_time
        if dt <= 0 or self.current_pole_angle is None:
            # Publish zero to avoid controller errors
            zero_cmd = Float64MultiArray()
            zero_cmd.data = [0.0]
            self.publisher.publish(zero_cmd)
            self.last_time = now
            return

        # Check safety conditions
        safety_violation = self.check_safety_conditions()
        
        if safety_violation or self.safety_active:
            # Safety mode: return cart to zero and stop pole control
            effort = self.cart_return_control(dt)
            
            # Log safety status with more detail
            if self.current_cart_position is not None:
                distance_to_zero = abs(self.current_cart_position)
                self.get_logger().info(f"SAFETY MODE - Cart Pos: {self.current_cart_position:.3f}, "
                                     f"Distance to zero: {distance_to_zero:.3f}, Effort: {effort:.3f}")
                
                # Notify when cart is close to zero
                if distance_to_zero < 0.05:
                    self.get_logger().info("Cart near zero position - Ready for reset!")
        else:
            # Normal operation: PID control on pole angle
            error = self.desired_pole_angle - self.current_pole_angle
            self.pole_integral += error * dt
            derivative = (error - self.pole_last_error) / dt
            effort = -(self.kp_pole * error + self.ki_pole * self.pole_integral + self.kd_pole * derivative)
            self.pole_last_error = error

            # Log normal operation
            self.get_logger().info(f"Normal Mode - Pole Angle: {self.current_pole_angle:.3f}, Effort: {effort:.3f}")

        # Publish command
        cmd = Float64MultiArray()
        cmd.data = [effort]
        self.publisher.publish(cmd)

        self.last_time = now


def main(args=None):
    rclpy.init(args=args)
    node = DualPIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()