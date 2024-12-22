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
        angle = ctrl.Antecedent(np.linspace(-30, 30, 300), 'angle')
        angle_vel = ctrl.Antecedent(np.linspace(-300, 300, 300), 'angle_vel')
        effort = ctrl.Consequent(np.linspace(-30, 30, 300), 'effort')

        # Açı için üyelik fonksiyonları (±30 derece aralığında)
        angle['NVVB'] = fuzz.trimf(angle.universe, [-30, -30, -25])    # Negatif Çok Çok Büyük
        angle['NVB'] = fuzz.trimf(angle.universe, [-25, -20, -15])     # Negatif Çok Büyük
        angle['NB'] = fuzz.trimf(angle.universe, [-20, -15, -10])      # Negatif Büyük
        angle['NM'] = fuzz.trimf(angle.universe, [-15, -10, -5])       # Negatif Orta
        angle['NS'] = fuzz.trimf(angle.universe, [-10, -5, -2])        # Negatif Küçük
        angle['NSS'] = fuzz.trimf(angle.universe, [-5, -2.5, 0])       # Negatif Çok Küçük
        angle['Z'] = fuzz.trimf(angle.universe, [-2.5, 0, 2.5])        # Sıfır
        angle['PSS'] = fuzz.trimf(angle.universe, [0, 2.5, 5])         # Pozitif Çok Küçük
        angle['PS'] = fuzz.trimf(angle.universe, [2, 5, 10])           # Pozitif Küçük
        angle['PM'] = fuzz.trimf(angle.universe, [5, 10, 15])          # Pozitif Orta
        angle['PB'] = fuzz.trimf(angle.universe, [10, 15, 20])         # Pozitif Büyük
        angle['PVB'] = fuzz.trimf(angle.universe, [15, 20, 25])        # Pozitif Çok Büyük
        angle['PVVB'] = fuzz.trimf(angle.universe, [25, 30, 30])       # Pozitif Çok Çok Büyük

        # Açısal hız için üyelik fonksiyonları
        angle_vel['NVVB'] = fuzz.trimf(angle_vel.universe, [-300, -300, -240])  # Negatif Çok Çok Büyük
        angle_vel['NVB'] = fuzz.trimf(angle_vel.universe, [-270, -210, -150])   # Negatif Çok Büyük
        angle_vel['NB'] = fuzz.trimf(angle_vel.universe, [-180, -140, -100])    # Negatif Büyük
        angle_vel['NM'] = fuzz.trimf(angle_vel.universe, [-120, -80, -40])      # Negatif Orta
        angle_vel['NS'] = fuzz.trimf(angle_vel.universe, [-60, -30, -10])       # Negatif Küçük
        angle_vel['Z'] = fuzz.trimf(angle_vel.universe, [-20, 0, 20])           # Sıfır
        angle_vel['PS'] = fuzz.trimf(angle_vel.universe, [10, 30, 60])          # Pozitif Küçük
        angle_vel['PM'] = fuzz.trimf(angle_vel.universe, [40, 80, 120])         # Pozitif Orta
        angle_vel['PB'] = fuzz.trimf(angle_vel.universe, [100, 140, 180])       # Pozitif Büyük
        angle_vel['PVB'] = fuzz.trimf(angle_vel.universe, [150, 210, 270])      # Pozitif Çok Büyük
        angle_vel['PVVB'] = fuzz.trimf(angle_vel.universe, [240, 300, 300])     # Pozitif Çok Çok Büyük

        # Effort için üyelik fonksiyonları (±30 N aralığında)
        effort['NVVB'] = fuzz.trimf(effort.universe, [-30, -30, -24])    # Negatif Çok Çok Büyük
        effort['NVB'] = fuzz.trimf(effort.universe, [-27, -21, -15])     # Negatif Çok Büyük
        effort['NB'] = fuzz.trimf(effort.universe, [-18, -12, -6])       # Negatif Büyük
        effort['NM'] = fuzz.trimf(effort.universe, [-9, -6, -3])         # Negatif Orta
        effort['NS'] = fuzz.trimf(effort.universe, [-4.5, -2.25, -0.75]) # Negatif Küçük
        effort['Z'] = fuzz.trimf(effort.universe, [-1.5, 0, 1.5])        # Sıfır
        effort['PS'] = fuzz.trimf(effort.universe, [0.75, 2.25, 4.5])    # Pozitif Küçük
        effort['PM'] = fuzz.trimf(effort.universe, [3, 6, 9])            # Pozitif Orta
        effort['PB'] = fuzz.trimf(effort.universe, [6, 12, 18])          # Pozitif Büyük
        effort['PVB'] = fuzz.trimf(effort.universe, [15, 21, 27])        # Pozitif Çok Büyük
        effort['PVVB'] = fuzz.trimf(effort.universe, [24, 30, 30])       # Pozitif Çok Çok Büyük

        # Kural tabanı
        rules = [
            # Merkez bölge kuralları
            ctrl.Rule(angle['Z'] & angle_vel['Z'], effort['Z']),
            ctrl.Rule(angle['Z'] & angle_vel['PS'], effort['NS']),
            ctrl.Rule(angle['Z'] & angle_vel['NS'], effort['PS']),
            
            # Çok küçük sapmalar için kurallar
            ctrl.Rule(angle['PSS'] & angle_vel['Z'], effort['NS']),
            ctrl.Rule(angle['NSS'] & angle_vel['Z'], effort['PS']),
            ctrl.Rule(angle['PSS'] & angle_vel['PS'], effort['NM']),
            ctrl.Rule(angle['NSS'] & angle_vel['NS'], effort['PM']),
            
            # Küçük sapmalar için kurallar
            ctrl.Rule(angle['PS'] & angle_vel['Z'], effort['NM']),
            ctrl.Rule(angle['NS'] & angle_vel['Z'], effort['PM']),
            ctrl.Rule(angle['PS'] & angle_vel['PS'], effort['NB']),
            ctrl.Rule(angle['NS'] & angle_vel['NS'], effort['PB']),
            
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
            control_output = np.clip(control_output, -30.0, 30.0)  # Effort'u ±30 ile sınırla
            
            cmd_msg = Float64MultiArray()
            cmd_msg.data = [float(control_output)]
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
            angle_deg = np.clip(angle_deg, -90, 90)
            angle_vel_deg = np.clip(angle_vel_deg, -300, 300)
            
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