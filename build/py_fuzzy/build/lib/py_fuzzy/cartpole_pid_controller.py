#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import time

class CartPolePIDController(Node):
    def __init__(self):
        super().__init__('cartpole_pid_controller')
        
        # PID parametreleri
        self.Kp = 1.5  # Oransal kazanç
        self.Ki = 0.0  # İntegral kazanç
        self.Kd = 2.0  # Türevsel kazanç
        
        # PID değişkenleri
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        
        # Hedef açı
        self.target_angle = 0.0
        
        # Joint states subscriber
        self.joint_states_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)
            
        # Velocity controller publisher
        self.vel_pub = self.create_publisher(
            Float64MultiArray,
            'velocity_controller/commands',
            10)
        
        self.get_logger().info('CartPole PID Controller başlatıldı')

    def joint_state_callback(self, msg):
        try:
            # Çubuğun açısını ve hızını al
            pole_index = msg.name.index('cart_to_pole')
            current_angle = msg.position[pole_index]
            current_velocity = msg.velocity[pole_index]
            
            # PID kontrol çıktısını hesapla
            control_output = self.pid_control(current_angle)
            
            # Kontrol çıktısını sınırla
            control_output = max(min(control_output, 25.0), -25.0)
            
            # Kontrol komutunu gönder
            cmd_msg = Float64MultiArray()
            cmd_msg.data = [control_output]
            self.vel_pub.publish(cmd_msg)
            
            # Debug bilgisi
            self.get_logger().info(f'Angle: {current_angle:.2f}, Velocity: {current_velocity:.2f}, Control: {control_output:.2f}')
            
        except ValueError:
            self.get_logger().error('cart_to_pole joint bulunamadı!')

    def pid_control(self, current_angle):
        # Zaman farkını hesapla
        current_time = time.time()
        dt = current_time - self.last_time
        
        # Hata hesapla
        error = self.target_angle - current_angle
        
        # İntegral hesapla
        self.integral += error * dt
        
        # Türev hesapla
        derivative = (error - self.last_error) / dt if dt > 0 else 0
        
        # PID çıktısını hesapla
        output = (self.Kp * error + 
                 self.Ki * self.integral + 
                 self.Kd * derivative)
        
        # Değerleri güncelle
        self.last_error = error
        self.last_time = current_time
        
        return output

def main(args=None):
    rclpy.init(args=args)
    controller = CartPolePIDController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()