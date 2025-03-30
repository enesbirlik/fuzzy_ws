#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class PolePIDController(Node):
    def __init__(self):
        super().__init__('pole_pid_controller')
        # /joint_states konusundan pole verilerini almak için abone olunuyor.
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        # /cart_velocity_controller/commands konusuna komut yayınlanıyor.
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/cart_effort_controller/commands',
            10
        )
        
        # PID parametreleri (sistem dinamiklerinize göre ayarlayın)
        self.kp = 100.0
        self.ki = 0.0
        self.kd = 20.0
        self.desired_angle = 0.0  # Hedef açı: 0 rad (pole dik)
        
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = self.get_clock().now().nanoseconds / 1e9
        
        self.current_angle = None
        self.current_velocity = None
        
        # Kontrol döngüsünü 100 Hz'de çalıştırmak için timer (0.01 saniye aralık)
        self.timer = self.create_timer(0.01, self.control_loop)

    def joint_state_callback(self, msg: JointState):
        # Gelen joint isimleri arasında "cart_to_pole" varsa, ilgili açıyı ve hızı al.
        if "cart_to_pole" in msg.name:
            idx = msg.name.index("cart_to_pole")
            self.current_angle = msg.position[idx]
            self.current_velocity = msg.velocity[idx]

    def wrap_to_range(self, x, min_val=-3.14, max_val=3.14):
        range_width = max_val - min_val
        wrapped = math.fmod(x - min_val, range_width)
        if (wrapped < 0):
            wrapped += range_width
        return wrapped + min_val

    def control_loop(self):
        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - self.last_time
        if dt <= 0 or self.current_angle is None:
            return
        
        # Önce current_angle'ı normalize edelim
        current_angle = self.wrap_to_range(self.current_angle)
        # Hata: hedef açı ile normalize edilmiş current_angle arasındaki farkı normalize ediyoruz.
        error = self.desired_angle - current_angle
        
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        
        # Hesaplanan PID output'unu komut olarak yayınla.
        msg = Float64MultiArray()
        msg.data = [output]
        self.publisher.publish(msg)
        
        self.get_logger().info(f"Norm Current Angle: {current_angle:.3f}, Error: {error:.3f}, Output: {output:.3f}")
        
        self.last_error = error
        self.last_time = now

def main(args=None):
    rclpy.init(args=args)
    node = PolePIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
