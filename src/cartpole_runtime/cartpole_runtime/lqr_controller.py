#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import scipy.linalg
import math

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class LQRSwingUpController(Node):
    def __init__(self):
        super().__init__('lqr_swingup_controller')
        # Parametreler: Bu değerleri sisteminize göre ayarlayın
        self.declare_parameter('M', 1.0)       # Cart kütlesi (kg)
        self.declare_parameter('m', 0.1)       # Pendulum kütlesi (kg)
        self.declare_parameter('l', 0.5)       # Pendulum yarı uzunluğu (m)
        self.declare_parameter('g', 9.81)      # Yerçekimi (m/s^2)
        self.declare_parameter('u_max', 1.0)   # Maksimum hız komutu (m/s)
        self.declare_parameter('swing_threshold', 0.2)  # LQR'ye geçiş için pole açısı (rad)
        self.declare_parameter('swing_k', 1.0)          # Swing-up kontrol kazancı

        self.M = self.get_parameter('M').value
        self.m = self.get_parameter('m').value
        self.l = self.get_parameter('l').value
        self.g = self.get_parameter('g').value
        self.u_max = self.get_parameter('u_max').value
        self.swing_threshold = self.get_parameter('swing_threshold').value
        self.swing_k = self.get_parameter('swing_k').value

        # Durum: x = [cart_position, cart_velocity, pole_angle, pole_ang_velocity]
        self.x = np.zeros(4)

        # LQR tasarımı: lineerleştirilmiş sistem (örnek matrisler; sisteminize göre ayarlayın)
        A = np.array([
            [0, 1, 0, 0],
            [0, 0, -(self.m*self.g)/self.M, 0],
            [0, 0, 0, 1],
            [0, 0, ((self.M+self.m)*self.g)/(self.M*self.l), 0]
        ])
        B = np.array([
            [0],
            [1/self.M],
            [0],
            [-1/(self.M*self.l)]
        ])
        Q = np.diag([10, 1, 100, 1])
        R = np.array([[1]])
        P = scipy.linalg.solve_continuous_are(A, B, Q, R)
        self.K = np.linalg.inv(R) @ B.T @ P
        self.get_logger().info(f"LQR gain: {self.K}")

        # Abonelik ve yayıncı
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.cmd_pub = self.create_publisher(Float64MultiArray, '/cart_velocity_controller/commands', 10)

        # Kontrol döngüsü: 50 Hz
        self.timer = self.create_timer(0.02, self.control_loop)

    def joint_state_callback(self, msg: JointState):
        # Cart için: "slider_to_cart"
        if "slider_to_cart" in msg.name:
            idx = msg.name.index("slider_to_cart")
            self.x[0] = msg.position[idx]
            self.x[1] = msg.velocity[idx]
        # Pole için: "cart_to_pole"
        if "cart_to_pole" in msg.name:
            idx = msg.name.index("cart_to_pole")
            self.x[2] = msg.position[idx]
            self.x[3] = msg.velocity[idx]

    def control_loop(self):
        # Hedef: Dimdik pozisyon, LQR stabilizasyonu için x_des = [0, 0, 0, 0]
        # Ancak, eğer pole açısı |θ| > swing_threshold ise swing-up kontrolü uygulanır.
        theta = self.x[2]
        theta_dot = self.x[3]

        if abs(theta) > self.swing_threshold:
            # Swing-up kontrolü (enerji şekillendirme)
            # Sistemin toplam enerjisi: E = 0.5*m*(l*theta_dot)^2 + m*g*l*(1 - cos(theta))
            E = 0.5 * self.m * (self.l * theta_dot)**2 + self.m * self.g * self.l * (1 - math.cos(theta))
            E_des = self.m * self.g * self.l  # Dimdik konumda (maksimum potansiyel enerji)
            # Enerji farkı üzerinden basit kontrol: u = k * (E_des - E) * sign(theta_dot * cos(theta))
            u = self.swing_k * (E_des - E) * np.sign(theta_dot * math.cos(theta))
            mode = "SWING-UP"
        else:
            # LQR stabilizasyonu
            x_des = np.zeros(4)
            error = self.x - x_des
            u = -self.K @ error
            u = float(u[0])
            mode = "LQR"

        # Sınırlama
        u = max(min(u, self.u_max), -self.u_max)

        # Komut yayınla
        msg = Float64MultiArray()
        msg.data = [u]
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"Mode: {mode}, State: {self.x}, Control (velocity): {u:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = LQRSwingUpController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
