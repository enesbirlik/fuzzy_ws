import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np

class CartPolePIDController(Node):
    def __init__(self):
        super().__init__('cartpole_pid_controller')

        # PID parameters
        self.Kp = 30.0  # Proportional gain
        self.Ki = 10.0   # Integral gains
        self.Kd = 10.0  # Derivative gain

        # PID calculation variables
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = None

        # target angle radian
        self.target_angle = 0.0

        # Joint states subscriber
        self.joint_states_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)
        
        # Effort controller publisher
        self.effort_pub = self.create_publisher(
            Float64MultiArray,
            'effort_controllers/commands',
            10)

    def rad_to_deg(self, rad):
        return rad * 180.0 / np.pi

    def deg_to_rad(self, deg):
        return deg * np.pi / 180.0

    def joint_state_callback(self, msg):
        try:
            pole_index = msg.name.index('cart_to_pole')
            
            current_angle = msg.position[pole_index]
            current_velocity = msg.velocity[pole_index]
            
            # take current time
            current_time = self.get_clock().now()
            if self.last_time is None:
                self.last_time = current_time
                return

            # dt calculation
            dt = (current_time - self.last_time).nanoseconds / 1e9  # saniyeye çevir
            if dt <= 0:
                return

            # PID control
            control_output = self.pid_control(current_angle, current_velocity, dt)
            
            # efoort limited to ±10 N
            control_output = np.clip(control_output, -1000.0, 1000.0)
            
            # publish effort
            cmd_msg = Float64MultiArray()
            cmd_msg.data = [float(-control_output)]  # İşareti ters çevir
            self.effort_pub.publish(cmd_msg)
            
            # update last time
            self.last_time = current_time
            
            # Log
            self.get_logger().info(
                f'Angle: {self.rad_to_deg(current_angle):6.17f}°, '
                f'Angular_Vel: {self.rad_to_deg(current_velocity):7.17f}°/s, '
                f'Effort: {control_output:5.17f}'
            )
            
        except ValueError:
            self.get_logger().error('Joint state message does not contain expected joint names.')

    def pid_control(self, current_angle, current_velocity, dt):
        try:
            # angel error
            error = self.target_angle - current_angle

            # Integral part
            self.integral += error * dt
            
            # Anti-windup: integral clipping
            self.integral = np.clip(self.integral, -1.0, 1.0)

            # derivative part
            derivative = -current_velocity  # negative because of the definition of error

            # PID output
            output = (self.Kp * error + 
                     self.Ki * self.integral + 
                     self.Kd * derivative)

            # Update last error
            self.last_error = error

            return output
        
        except Exception as e:
            self.get_logger().error(f'PID hesaplama hatası: {str(e)}')
            return 0.0

def main(args=None):
    rclpy.init(args=args)
    node = CartPolePIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()