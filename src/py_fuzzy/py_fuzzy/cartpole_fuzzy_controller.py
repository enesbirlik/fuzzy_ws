import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

class CartPoleFuzzyController(Node):
    def __init__(self):
        super().__init__('cartpole_fuzzy_controller')

        # Fuzzy kontrolörün kurulumu
        self.setup_fuzzy_controller()

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

    def setup_fuzzy_controller(self):
        # Evren tanımları
        angle = ctrl.Antecedent(np.linspace(-15, 15, 300), 'angle')
        angle_vel = ctrl.Antecedent(np.linspace(-150, 150, 300), 'angle_vel')
        effort = ctrl.Consequent(np.linspace(-10, 10, 300), 'effort')

        # Açı için üyelik fonksiyonları (±15 derece)
        angle['NVVB'] = fuzz.trimf(angle.universe, [-15, -15, -12])    # Negatif Çok Çok Büyük
        angle['NVB'] = fuzz.trimf(angle.universe, [-13, -10, -7])      # Negatif Çok Büyük
        angle['NB'] = fuzz.trimf(angle.universe, [-8, -6, -4])         # Negatif Büyük
        angle['NM'] = fuzz.trimf(angle.universe, [-5, -3.5, -2])       # Negatif Orta
        angle['NS'] = fuzz.trimf(angle.universe, [-3, -1.5, -0.5])     # Negatif Küçük
        angle['Z'] = fuzz.trimf(angle.universe, [-1, 0, 1])            # Sıfır
        angle['PS'] = fuzz.trimf(angle.universe, [0.5, 1.5, 3])        # Pozitif Küçük
        angle['PM'] = fuzz.trimf(angle.universe, [2, 3.5, 5])          # Pozitif Orta
        angle['PB'] = fuzz.trimf(angle.universe, [4, 6, 8])            # Pozitif Büyük
        angle['PVB'] = fuzz.trimf(angle.universe, [7, 10, 13])         # Pozitif Çok Büyük
        angle['PVVB'] = fuzz.trimf(angle.universe, [12, 15, 15])       # Pozitif Çok Çok Büyük

        # Açısal hız için üyelik fonksiyonları (±150 deg/s)
        angle_vel['NVVB'] = fuzz.trimf(angle_vel.universe, [-150, -150, -120])  # Negatif Çok Çok Büyük
        angle_vel['NVB'] = fuzz.trimf(angle_vel.universe, [-130, -100, -70])    # Negatif Çok Büyük
        angle_vel['NB'] = fuzz.trimf(angle_vel.universe, [-80, -60, -40])       # Negatif Büyük
        angle_vel['NM'] = fuzz.trimf(angle_vel.universe, [-50, -30, -15])       # Negatif Orta
        angle_vel['NS'] = fuzz.trimf(angle_vel.universe, [-20, -10, -3])        # Negatif Küçük
        angle_vel['Z'] = fuzz.trimf(angle_vel.universe, [-5, 0, 5])             # Sıfır
        angle_vel['PS'] = fuzz.trimf(angle_vel.universe, [3, 10, 20])           # Pozitif Küçük
        angle_vel['PM'] = fuzz.trimf(angle_vel.universe, [15, 30, 50])          # Pozitif Orta
        angle_vel['PB'] = fuzz.trimf(angle_vel.universe, [40, 60, 80])          # Pozitif Büyük
        angle_vel['PVB'] = fuzz.trimf(angle_vel.universe, [70, 100, 130])       # Pozitif Çok Büyük
        angle_vel['PVVB'] = fuzz.trimf(angle_vel.universe, [120, 150, 150])     # Pozitif Çok Çok Büyük

        # Effort için üyelik fonksiyonları (±10 N)
        effort['NVVB'] = fuzz.trimf(effort.universe, [-10, -10, -8])     # Negatif Çok Çok Büyük
        effort['NVB'] = fuzz.trimf(effort.universe, [-9, -7, -5])        # Negatif Çok Büyük
        effort['NB'] = fuzz.trimf(effort.universe, [-6, -4, -2.5])       # Negatif Büyük
        effort['NM'] = fuzz.trimf(effort.universe, [-3, -2, -1])         # Negatif Orta
        effort['NS'] = fuzz.trimf(effort.universe, [-1.5, -0.75, -0.25]) # Negatif Küçük
        effort['Z'] = fuzz.trimf(effort.universe, [-0.5, 0, 0.5])        # Sıfır
        effort['PS'] = fuzz.trimf(effort.universe, [0.25, 0.75, 1.5])    # Pozitif Küçük
        effort['PM'] = fuzz.trimf(effort.universe, [1, 2, 3])            # Pozitif Orta
        effort['PB'] = fuzz.trimf(effort.universe, [2.5, 4, 6])          # Pozitif Büyük
        effort['PVB'] = fuzz.trimf(effort.universe, [5, 7, 9])           # Pozitif Çok Büyük
        effort['PVVB'] = fuzz.trimf(effort.universe, [8, 10, 10])        # Pozitif Çok Çok Büyük

        # Kural tabanı
        rules = [
            # Merkez bölge kuralları
            ctrl.Rule(angle['Z'] & angle_vel['Z'], effort['Z']),
            ctrl.Rule(angle['Z'] & angle_vel['PS'], effort['NS']),
            ctrl.Rule(angle['Z'] & angle_vel['NS'], effort['PS']),
            
            # Küçük sapmalar için kurallar
            ctrl.Rule(angle['PS'] & angle_vel['Z'], effort['NS']),
            ctrl.Rule(angle['NS'] & angle_vel['Z'], effort['PS']),
            ctrl.Rule(angle['PS'] & angle_vel['PS'], effort['NM']),
            ctrl.Rule(angle['NS'] & angle_vel['NS'], effort['PM']),
            
            # Orta sapmalar için kurallar
            ctrl.Rule(angle['PM'] & angle_vel['Z'], effort['NB']),
            ctrl.Rule(angle['NM'] & angle_vel['Z'], effort['PB']),
            ctrl.Rule(angle['PM'] & angle_vel['PS'], effort['NVB']),
            ctrl.Rule(angle['NM'] & angle_vel['NS'], effort['PVB']),
            
            # Büyük sapmalar için kurallar
            ctrl.Rule(angle['PB'] & angle_vel['Z'], effort['NVB']),
            ctrl.Rule(angle['NB'] & angle_vel['Z'], effort['PVB']),
            ctrl.Rule(angle['PB'] & angle_vel['PS'], effort['NVVB']),
            ctrl.Rule(angle['NB'] & angle_vel['NS'], effort['PVVB']),
            
            # Çok büyük sapmalar için kurallar
            ctrl.Rule(angle['PVB'], effort['NVVB']),
            ctrl.Rule(angle['NVB'], effort['PVVB']),
            
            # Çok çok büyük sapmalar için kurallar
            ctrl.Rule(angle['PVVB'], effort['NVVB']),
            ctrl.Rule(angle['NVVB'], effort['PVVB']),
            
            # Yüksek hız durumları için kurallar
            ctrl.Rule(angle_vel['PVVB'], effort['NVVB']),
            ctrl.Rule(angle_vel['NVVB'], effort['PVVB']),
            ctrl.Rule(angle_vel['PVB'], effort['NVB']),
            ctrl.Rule(angle_vel['NVB'], effort['PVB']),
            
            # Kombinasyon kuralları
            ctrl.Rule(angle['PS'] & angle_vel['PB'], effort['NVB']),
            ctrl.Rule(angle['NS'] & angle_vel['NB'], effort['PVB']),
        ]

        self.fuzzy_sim = ctrl.ControlSystem(rules)
        self.fuzzy_simulation = ctrl.ControlSystemSimulation(self.fuzzy_sim)

    def rad_to_deg(self, rad):
        return rad * 180.0 / np.pi

    def joint_state_callback(self, msg):
        try:
            pole_index = msg.name.index('cart_to_pole')
            
            current_angle_deg = self.rad_to_deg(msg.position[pole_index])
            current_velocity_deg = self.rad_to_deg(msg.velocity[pole_index])
            
            control_output = self.fuzzy_control(current_angle_deg, current_velocity_deg)
            control_output = np.clip(control_output, -10.0, 10.0)  # Effort'u ±10 N ile sınırla
            
            cmd_msg = Float64MultiArray()
            cmd_msg.data = [float(-control_output)]  # Çıkışın işaretini ters çevir
            self.effort_pub.publish(cmd_msg)
            
            # Detaylı log
            self.get_logger().info(
                f'Angle: {current_angle_deg:6.2f}°, '
                f'Angular_Vel: {current_velocity_deg:7.2f}°/s, '
                f'Effort: {control_output:5.2f}'
            )
            
        except ValueError:
            self.get_logger().error('Joint state message does not contain expected joint names.')

    def fuzzy_control(self, angle_deg, angle_vel_deg):
        try:
            # Giriş değerlerini sınırla
            angle_deg = np.clip(angle_deg, -15, 15)         # Açıyı ±15 derece ile sınırla
            angle_vel_deg = np.clip(angle_vel_deg, -150, 150)  # Açısal hızı ±150 deg/s ile sınırla
            
            self.fuzzy_simulation.input['angle'] = angle_deg
            self.fuzzy_simulation.input['angle_vel'] = angle_vel_deg
            
            self.fuzzy_simulation.compute()
            
            return self.fuzzy_simulation.output['effort']
        
        except Exception as e:
            self.get_logger().error(f'Fuzzy hesaplama hatası: {str(e)}')
            return 0.0

def main(args=None):
    rclpy.init(args=args)
    node = CartPoleFuzzyController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()