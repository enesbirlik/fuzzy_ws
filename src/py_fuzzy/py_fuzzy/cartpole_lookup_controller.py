import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np

class CartPoleLookupController(Node):
    def __init__(self):
        super().__init__('cartpole_lookup_controller')
        
        # Lookup table aralıklarını tanımla
        self.angle_bins = np.linspace(-15, 15, 31)  # 1 derece hassasiyet
        self.velocity_bins = np.linspace(-150, 150, 31)  # 10 derece/s hassasiyet
        
        # Lookup table'ı oluştur
        self.create_lookup_table()
        
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

    def create_lookup_table(self):
        # Boş lookup table matrisi oluştur
        self.lookup_table = np.zeros((len(self.angle_bins)-1, len(self.velocity_bins)-1))
        
        # Lookup table'ı doldur
        for i in range(len(self.angle_bins)-1):
            for j in range(len(self.velocity_bins)-1):
                angle_mid = (self.angle_bins[i] + self.angle_bins[i+1]) / 2
                vel_mid = (self.velocity_bins[j] + self.velocity_bins[j+1]) / 2
                
                effort = self.calculate_control_effort(angle_mid, vel_mid)
                self.lookup_table[i,j] = effort

    def calculate_control_effort(self, angle, velocity):
        # Basit kontrol 
        angle_term = -2.0 * angle  # Açıya oransal tepki
        velocity_term = -1.0 * velocity  # Hıza oransal tepki
        
        # Farklı bölgeler için 
        if abs(angle) < 2.0:  
            effort = 0.5 * angle_term + 0.3 * velocity_term
        elif abs(angle) < 5.0: 
            effort = 1.0 * angle_term + 0.5 * velocity_term
        elif abs(angle) < 10.0: 
            effort = 2.0 * angle_term + 0.7 * velocity_term
        else: 
            effort = 3.0 * angle_term + 1.0 * velocity_term
        
        return np.clip(effort, -10.0, 10.0) 

    def find_lookup_index(self, value, bins):
        index = np.digitize(value, bins) - 1
        return min(max(0, index), len(bins)-2)

    def get_control_effort(self, angle, velocity):
        # Açı ve hız değerleri için indeksleri bul
        angle_index = self.find_lookup_index(angle, self.angle_bins)
        velocity_index = self.find_lookup_index(velocity, self.velocity_bins)
        
        # Lookup table'dan değeri al
        return self.lookup_table[angle_index, velocity_index]

    def rad_to_deg(self, rad):
        return rad * 180.0 / np.pi

    def joint_state_callback(self, msg):
        try:
            pole_index = msg.name.index('cart_to_pole')
            
            current_angle_deg = self.rad_to_deg(msg.position[pole_index])
            current_velocity_deg = self.rad_to_deg(msg.velocity[pole_index])
            
            # Lookup table'dan kontrol çıkışını al
            control_output = self.get_control_effort(current_angle_deg, current_velocity_deg)
            
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

def main(args=None):
    rclpy.init(args=args)
    node = CartPoleLookupController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()