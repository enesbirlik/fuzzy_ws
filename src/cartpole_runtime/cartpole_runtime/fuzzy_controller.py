import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import time
import numpy as np
import skfuzzy as fuzz
import skfuzzy.control as ctrl
import argparse

class FuzzyController(Node):
    def __init__(self, controller_type='position'):
        super().__init__('fuzzy_controller')
        self.controller_type = controller_type
        
        # Seçilen kontrol tipine göre komut konusunu belirleyelim
        if self.controller_type == 'position':
            self.cmd_topic = '/cart_position_controller/commands'
        else:
            self.cmd_topic = '/cart_velocity_controller/commands'
        
        self.cmd_pub = self.create_publisher(Float64MultiArray, self.cmd_topic, 10)
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        
        # Pole açısı ve açısal hızı (swing-up kontrolü için esas)
        self.pole_angle = 0.0
        self.pole_angular_velocity = 0.0
        
        # Fuzzy kontrol sistemini kur
        self.setup_fuzzy_controller()
        
        # Kontrol döngüsünü timer ile çalıştır (20 Hz)
        self.timer = self.create_timer(0.05, self.control_callback)
        self.get_logger().info(f"Fuzzy controller started in {self.controller_type} mode on topic {self.cmd_topic}")

    def setup_fuzzy_controller(self):
        # Girişlerin evrenleri: 
        # error: fark = (0 - pole_angle) [-0.5, 0.5] (radyan cinsinden, örnek ölçek)
        # error_dot: pole angular velocity farkı [-2, 2]
        self.error = ctrl.Antecedent(np.arange(-0.5, 0.51, 0.01), 'error')
        self.error_dot = ctrl.Antecedent(np.arange(-2, 2.01, 0.01), 'error_dot')
        # Çıkış evreni: command [-1, 1]
        self.command = ctrl.Consequent(np.arange(-1, 1.01, 0.01), 'command')
        
        # 7 adet üyelik fonksiyonu etiketleri
        labels = ['NB', 'NM', 'NS', 'ZE', 'PS', 'PM', 'PB']
        
        # Error için üyelik fonksiyonları: merkezler [-0.5, -0.3333, -0.1667, 0, 0.1667, 0.3333, 0.5]
        e_centers = [-0.5, -0.3333, -0.1667, 0.0, 0.1667, 0.3333, 0.5]
        for i, label in enumerate(labels):
            if i == 0:
                self.error[label] = fuzz.trimf(self.error.universe, [e_centers[i], e_centers[i], e_centers[i+1]])
            elif i == len(labels)-1:
                self.error[label] = fuzz.trimf(self.error.universe, [e_centers[i-1], e_centers[i], e_centers[i]])
            else:
                self.error[label] = fuzz.trimf(self.error.universe, [e_centers[i-1], e_centers[i], e_centers[i+1]])
        
        # error_dot için üyelik fonksiyonları: merkezler [-2, -1.333, -0.667, 0, 0.667, 1.333, 2]
        ed_centers = [-2.0, -1.333, -0.667, 0.0, 0.667, 1.333, 2.0]
        for i, label in enumerate(labels):
            if i == 0:
                self.error_dot[label] = fuzz.trimf(self.error_dot.universe, [ed_centers[i], ed_centers[i], ed_centers[i+1]])
            elif i == len(labels)-1:
                self.error_dot[label] = fuzz.trimf(self.error_dot.universe, [ed_centers[i-1], ed_centers[i], ed_centers[i]])
            else:
                self.error_dot[label] = fuzz.trimf(self.error_dot.universe, [ed_centers[i-1], ed_centers[i], ed_centers[i+1]])
        
        # Command için üyelik fonksiyonları: merkezler [-1, -0.667, -0.333, 0, 0.333, 0.667, 1]
        c_centers = [-1.0, -0.667, -0.333, 0.0, 0.333, 0.667, 1.0]
        for i, label in enumerate(labels):
            if i == 0:
                self.command[label] = fuzz.trimf(self.command.universe, [c_centers[i], c_centers[i], c_centers[i+1]])
            elif i == len(labels)-1:
                self.command[label] = fuzz.trimf(self.command.universe, [c_centers[i-1], c_centers[i], c_centers[i]])
            else:
                self.command[label] = fuzz.trimf(self.command.universe, [c_centers[i-1], c_centers[i], c_centers[i+1]])
        
        # 49 kuralı (7x7) kural tablosunu oluşturuyoruz.
        # Örneğin; çıkış indeksi: output_idx = 6 - round((i+j)/2)
        self.rules = []
        for i, e_label in enumerate(labels):
            for j, ed_label in enumerate(labels):
                output_idx = 6 - int(round((i + j) / 2))
                output_label = labels[output_idx]
                rule = ctrl.Rule(self.error[e_label] & self.error_dot[ed_label], self.command[output_label])
                self.rules.append(rule)
        
        # Fuzzy kontrol sistemi ve simülasyonu
        self.fuzzy_ctrl = ctrl.ControlSystem(self.rules)
        self.fuzzy_sim = ctrl.ControlSystemSimulation(self.fuzzy_ctrl)
        self.get_logger().info("Fuzzy control system set up with 49 rules.")

    def joint_state_callback(self, msg: JointState):
        # Pole verilerini "cart_to_pole" joint'inden alıyoruz
        if "cart_to_pole" in msg.name:
            idx = msg.name.index("cart_to_pole")
            self.pole_angle = msg.position[idx]
            self.pole_angular_velocity = msg.velocity[idx]

    def control_callback(self):
        # Hedef pole açısı: 0 (upright); hata = 0 - pole_angle
        error_val = 0 - self.pole_angle
        error_dot_val = 0 - self.pole_angular_velocity
        
        # Giriş değerlerini evren sınırlarına göre sınırlıyoruz
        error_val = np.clip(error_val, -0.5, 0.5)
        error_dot_val = np.clip(error_dot_val, -2.0, 2.0)
        
        self.fuzzy_sim.input['error'] = error_val
        self.fuzzy_sim.input['error_dot'] = error_dot_val
        
        try:
            self.fuzzy_sim.compute()
            control_output = self.fuzzy_sim.output['command']
        except Exception as e:
            self.get_logger().error(f"Fuzzy computation error: {e}")
            control_output = 0.0
        
        # Swing-up durumu: pole çok eğikse komut çıkışını ölçeklendir
        if abs(self.pole_angle) > 0.5:
            control_output *= 1.5
        
        msg = Float64MultiArray()
        msg.data = [control_output]
        self.cmd_pub.publish(msg)
        self.get_logger().info(
            f"Fuzzy Control: error={error_val:.3f}, error_dot={error_dot_val:.3f}, output={control_output:.3f}"
        )


def main(args=None):
    parser = argparse.ArgumentParser(description='Fuzzy Controller for Cartpole Swing-Up')
    parser.add_argument(
        'controller_type',
        nargs='?',
        default='position',
        choices=['position', 'velocity'],
        help='Select controller type (position or velocity)'
    )
    parsed_args, unknown = parser.parse_known_args()
    
    rclpy.init(args=args)
    node = FuzzyController(controller_type=parsed_args.controller_type)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
