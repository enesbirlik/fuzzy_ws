import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
import numpy as np
from collections import deque

class CartPoleDataPlotter(Node):
    def __init__(self):
        super().__init__('cartpole_data_plotter')
        
        # Veri bufferları
        self.max_points = 500
        self.times = deque(maxlen=self.max_points)
        self.angles = deque(maxlen=self.max_points)
        self.velocities = deque(maxlen=self.max_points)
        self.efforts = deque(maxlen=self.max_points)
        
        # Başlangıç zamanı
        self.start_time = None
        
        # Effort değerini takip etmek için
        self.current_effort = 0.0
        
        # Joint states subscriber
        self.joint_states_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)
            
        # Effort subscriber
        self.effort_sub = self.create_subscription(
            Float64MultiArray,
            'effort_controllers/commands',
            self.effort_callback,
            10)
        
        # Plot kurulumu
        plt.ion()
        plt.style.use('dark_background')
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, 1, figsize=(12, 8))
        self.fig.suptitle('Cart-Pole Control System Data', fontsize=12)
        
        # Plot çizgileri
        self.line_angle, = self.ax1.plot([], [], 'c-', label='Angle (degrees)', linewidth=2)
        self.line_velocity, = self.ax2.plot([], [], 'y-', label='Angular Velocity (degrees/s)', linewidth=2)
        self.line_effort, = self.ax3.plot([], [], 'm-', label='Control Effort (N)', linewidth=2)
        
        # Eksenlerin ayarlanması
        self.setup_axes()
        
        # Plot güncelleme zamanı
        self.last_update_time = 0
        self.update_interval = 0.05  # 50ms
        
    def setup_axes(self):
        # Açı grafiği (±4 derece)
        self.ax1.set_ylabel('Angle (°)')
        self.ax1.set_ylim(-4, 4)
        self.ax1.grid(True, linestyle='--', alpha=0.7)
        self.ax1.legend(loc='upper right')
        # Yatay referans çizgisi
        self.ax1.axhline(y=0, color='w', linestyle='--', alpha=0.3)
        
        # Açısal hız grafiği (±8 derece/s)
        self.ax2.set_ylabel('Angular\nVelocity (°/s)')
        self.ax2.set_ylim(-8, 8)
        self.ax2.grid(True, linestyle='--', alpha=0.7)
        self.ax2.legend(loc='upper right')
        # Yatay referans çizgisi
        self.ax2.axhline(y=0, color='w', linestyle='--', alpha=0.3)
        
        # Effort grafiği (±2 N)
        self.ax3.set_xlabel('Time (s)')
        self.ax3.set_ylabel('Effort (N)')
        self.ax3.set_ylim(-2, 2)
        self.ax3.grid(True, linestyle='--', alpha=0.7)
        self.ax3.legend(loc='upper right')
        # Yatay referans çizgisi
        self.ax3.axhline(y=0, color='w', linestyle='--', alpha=0.3)
        
        # Her eksen için minor ticks ekle
        self.ax1.minorticks_on()
        self.ax2.minorticks_on()
        self.ax3.minorticks_on()
        
        # Grid'leri özelleştir
        self.ax1.grid(True, which='minor', linestyle=':', alpha=0.4)
        self.ax2.grid(True, which='minor', linestyle=':', alpha=0.4)
        self.ax3.grid(True, which='minor', linestyle=':', alpha=0.4)
        
        plt.tight_layout()
    
    def rad_to_deg(self, rad):
        return rad * 180.0 / np.pi
        
    def joint_state_callback(self, msg):
        try:
            # Başlangıç zamanını ayarla
            if self.start_time is None:
                self.start_time = self.get_clock().now()
                
            pole_index = msg.name.index('cart_to_pole')
            current_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            
            # Verileri ekle
            self.times.append(current_time)
            self.angles.append(self.rad_to_deg(msg.position[pole_index]))
            self.velocities.append(self.rad_to_deg(msg.velocity[pole_index]))
            self.efforts.append(self.current_effort)
            
            # Plot güncelleme kontrolü
            if current_time - self.last_update_time >= self.update_interval:
                self.update_plot()
                self.last_update_time = current_time
                
        except ValueError as e:
            self.get_logger().error(f'Joint state error: {str(e)}')
            
    def effort_callback(self, msg):
        if len(msg.data) > 0:
            self.current_effort = -msg.data[0]  # İşareti ters çevrilmiş effort
            
    def update_plot(self):
        if len(self.times) < 2:  # En az 2 nokta gerekli
            return
            
        # List'e çevir
        times_array = np.array(self.times)
        angles_array = np.array(self.angles)
        velocities_array = np.array(self.velocities)
        efforts_array = np.array(self.efforts)
        
        # Son 10 saniyelik veriyi göster
        if len(times_array) > 0:
            current_time = times_array[-1]
            mask = times_array >= (current_time - 10)
            
            times_array = times_array[mask]
            angles_array = angles_array[mask]
            velocities_array = velocities_array[mask]
            efforts_array = efforts_array[mask]
        
        # Verileri güncelle
        self.line_angle.set_data(times_array, angles_array)
        self.line_velocity.set_data(times_array, velocities_array)
        self.line_effort.set_data(times_array, efforts_array)
        
        # X ekseni limitlerini güncelle
        for ax in [self.ax1, self.ax2, self.ax3]:
            ax.set_xlim(max(0, times_array[-1] - 10), times_array[-1] + 0.5)
        
        # Grafiği yenile
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = CartPoleDataPlotter()
    
    try:
        plt.show(block=False)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.close('all')

if __name__ == '__main__':
    main()