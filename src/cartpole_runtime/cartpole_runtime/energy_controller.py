#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import scipy.linalg
import math
import time

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class LQRSwingUpPIController(Node):
    def __init__(self):
        super().__init__('lqr_swingup_pi_controller')
        # Parametreler (sisteminizin özelliklerine göre ayarlayın)
        self.declare_parameter('M', 1.0)       # Cart kütlesi (kg)
        self.declare_parameter('m', 0.1)       # Pendulum kütlesi (kg)
        self.declare_parameter('l', 0.5)       # Pendulum yarı uzunluğu (m)
        self.declare_parameter('g', 9.81)      # Yerçekimi (m/s^2)
        self.declare_parameter('u_max', 10.0)   # Maksimum hız komutu (m/s)
        self.declare_parameter('swing_threshold', 0.2)  # LQR'ye geçiş için açı farkı (rad)
        self.declare_parameter('swing_k', 1.0)          # Swing-up kontrol kazancı

        self.M = self.get_parameter('M').value
        self.m = self.get_parameter('m').value
        self.l = self.get_parameter('l').value
        self.g = self.get_parameter('g').value
        self.u_max = self.get_parameter('u_max').value
        self.swing_threshold = self.get_parameter('swing_threshold').value
        self.swing_k = self.get_parameter('swing_k').value

        # Hedef pole açısı: π (ters konum)
        self.desired_theta = np.pi

        # Durum: x = [cart_position, cart_velocity, pole_angle, pole_ang_velocity]
        self.x = np.zeros(4)

        # LQR tasarımı için lineerleştirilmiş sistem (linearizasyon, hedefin π olduğu varsayımına göre yapılmalı)
        # Not: Bu A, B matrisleri yaklaşık olarak, hedefin 0 etrafında linearize edildiği varsayımındaydı.
        # Gerçek uygulamada linearizasyonu π etrafında yapmanız gerekir.
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
        # Hız kontrolü için: motorlar sadece hız kontrolü sağladığından, velocity command kullanıyoruz
        self.cmd_pub = self.create_publisher(Float64MultiArray, '/cart_velocity_controller/commands', 10)

        self.start_time = time.time()
        # Kontrol döngüsü: 50 Hz
        self.timer = self.create_timer(0.02, self.control_loop)

    def joint_state_callback(self, msg: JointState):
        # Cart: "slider_to_cart"
        if "slider_to_cart" in msg.name:
            idx = msg.name.index("slider_to_cart")
            self.x[0] = msg.position[idx]
            self.x[1] = msg.velocity[idx]
        # Pole: "cart_to_pole"
        if "cart_to_pole" in msg.name:
            idx = msg.name.index("cart_to_pole")
            self.x[2] = msg.position[idx]
            self.x[3] = msg.velocity[idx]

    def control_loop(self):
        # Mevcut pole açısını alın
        theta = self.x[2]
        theta_dot = self.x[3]

        # Hedef durum: [0, 0, π, 0]
        x_des = np.array([0.0, 0.0, self.desired_theta, 0.0])
        # Açısal hata: |theta - π|
        angle_error = abs(theta - self.desired_theta)

        if angle_error < self.swing_threshold:
            # Hedefe yakınsa LQR stabilizasyonu devreye girsin
            error = self.x - x_des
            u = -self.K @ error
            u = float(u[0])
            mode = "LQR"
        else:
            # Hedeften uzaktaysa enerji şekillendirme (swing-up) kontrolü uygulansın
            # Potansiyel enerji: PE = m*g*l*(1 - cos(theta))
            # Hedef potansiyel enerji: PE_des = m*g*l*(1 - cos(π)) = 2*m*g*l
            E = 0.5 * self.m * (self.l * theta_dot)**2 + self.m * self.g * self.l * (1 - math.cos(theta))
            E_des = self.m * self.g * self.l * (1 - math.cos(self.desired_theta))
            u = self.swing_k * (E_des - E) * np.sign(theta_dot * math.sin(theta - self.desired_theta))
            mode = "SWING-UP"

        # Sınırlama: Komut ± u_max
        u = max(min(u, self.u_max), -self.u_max)

        msg = Float64MultiArray()
        msg.data = [u]
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"Mode: {mode}, State: {self.x}, Control (velocity): {u:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = LQRSwingUpPIController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
