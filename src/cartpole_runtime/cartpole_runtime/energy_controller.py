#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import math

class PoleBalancer(Node):
    def __init__(self):
        super().__init__('pole_balancer')
        # Effort kontrolü üzerinden çalışacağımızı varsayıyoruz.
        self.cmd_pub = self.create_publisher(Float64MultiArray, '/cart_effort_controller/commands', 10)
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        
        # Pole ve cart durumları (joint "cart_to_pole" ve "slider_to_cart")
        self.cart_pos = 0.0
        self.cart_vel = 0.0
        self.cart_effort = 0.0
        
        self.pole_angle = 0.0     # Pole açısı (0: dik konum, rad cinsinden)
        self.pole_ang_vel = 0.0   # Pole açısal hızı
        self.pole_effort = 0.0

        # Kontrol döngüsü için timer (örneğin 50 Hz)
        self.timer = self.create_timer(0.02, self.control_loop)
        
        # PID kontrol parametreleri (balans bölgesi için)
        self.pid_kp = 50.0
        self.pid_ki = 0.0
        self.pid_kd = 5.0
        self.integrator = 0.0
        self.prev_error = 0.0
        
        # Enerji kontrol parametreleri (swing-up bölgesi)
        self.energy_gain = 8.0  # Enerji kontrol kazancı

        # Balans bölgesi eşik değeri (radyan cinsinden)
        self.balance_threshold = 0.1

        # Parametreler: pole kütlesi, uzunluğu, yerçekimi
        self.m = 0.5    # [kg]
        self.l = 0.12    # [m]
        self.g = 9.81   # [m/s^2]

    def joint_state_callback(self, msg: JointState):
        # Cart: "slider_to_cart"
        if "slider_to_cart" in msg.name:
            idx = msg.name.index("slider_to_cart")
            self.cart_pos = msg.position[idx]
            self.cart_vel = msg.velocity[idx]
            if len(msg.effort) > idx:
                self.cart_effort = msg.effort[idx]
        
        # Pole: "cart_to_pole"
        if "cart_to_pole" in msg.name:
            idx = msg.name.index("cart_to_pole")
            self.pole_angle = msg.position[idx]
            self.pole_ang_vel = msg.velocity[idx]
            if len(msg.effort) > idx:
                self.pole_effort = msg.effort[idx]
        
    def control_loop(self):
        # Eğer pole açı |θ| küçükse, PID kontrolü uygulayalım; büyükse enerji kontrolü.
        if abs(self.pole_angle) < self.balance_threshold:
            # PID kontrol
            error = self.pole_angle  # Hedef 0
            self.integrator += error * 0.02
            derivative = (error - self.prev_error) / 0.02
            u = self.pid_kp * error + self.pid_ki * self.integrator + self.pid_kd * derivative
            self.prev_error = error
            control_mode = "PID"
        else:
            # Enerji kontrolü (swing-up)
            # Hesaplanan enerji: E = 0.5*m*(l*θ_dot)^2 + m*g*l*(1 - cos(θ))
            kinetic = 0.5 * self.m * (self.l * self.pole_ang_vel)**2
            potential = self.m * self.g * self.l * (1 - math.cos(self.pole_angle))
            current_energy = kinetic + potential
            # Hedef enerji: yukarı dik konumda (θ = 0) enerji farkı sıfır kabul ediliyor
            energy_error = current_energy  # (Hedef E = 0)
            # Enerji kontrol yasası: u = k * (E) * sign(θ_dot*cos(θ))
            sign_term = math.copysign(1.0, self.pole_ang_vel * math.cos(self.pole_angle)) if self.pole_ang_vel != 0 else 1.0
            u = self.energy_gain * energy_error * sign_term
            control_mode = "Energy"

        # Yayınlanacak komut: Float64MultiArray şeklinde
        msg = Float64MultiArray()
        msg.data = [u]
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"[{control_mode} CTRL] u = {u:.3f} | θ = {self.pole_angle:.3f} rad")
        
def main(args=None):
    rclpy.init(args=args)
    node = PoleBalancer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
