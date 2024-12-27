import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
import numpy as np
from collections import deque

class SimpleCartPoleDataPlotter(Node):
    def __init__(self):
        super().__init__('simple_cartpole_data_plotter')
        
        # Veri bufferları
        self.buffer_size = 500
        self.times = deque(maxlen=self.buffer_size)
        self.angles = deque(maxlen=self.buffer_size)
        self.velocities = deque(maxlen=self.buffer_size)
        self.efforts = deque(maxlen=self.buffer_size)
        
        self.start_time = None
        self.current_effort = 0.0
        
        # Subscribers
        self.joint_states_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)
            
        self.effort_sub = self.create_subscription(
            Float64MultiArray,
            'effort_controllers/commands',
            self.effort_callback,
            10)
        
        # Plot kurulumu
        plt.ion()  # Interactive mode açık
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, 1, figsize=(10, 8))
        self.setup_plots()
        
        # Plot güncelleme zamanı
        self.last_plot_time = 0
        self.plot_interval = 0.1  # 100ms
        
    def setup_plots(self):
        self.fig.suptitle('Cart-Pole System Response')
        
        # Açı grafiği
        self.line_angle, = self.ax1.plot([], [], 'b-', label='Angle')
        self.ax1.set_ylabel('Angle (°)')
        self.ax1.set_ylim(-6, 6)
        self.ax1.grid(True)
        self.ax1.legend()
        
        # Açısal hız grafiği
        self.line_velocity, = self.ax2.plot([], [], 'g-', label='Angular Velocity')
        self.ax2.set_ylabel('Angular Velocity (°/s)')
        self.ax2.set_ylim(-20, 20)
        self.ax2.grid(True)
        self.ax2.legend()
        
        # Kontrol sinyali grafiği
        self.line_effort, = self.ax3.plot([], [], 'r-', label='Control Effort')
        self.ax3.set_xlabel('Time (s)')
        self.ax3.set_ylabel('Control Effort (N)')
        self.ax3.set_ylim(-3, 3)
        self.ax3.grid(True)
        self.ax3.legend()
        
        plt.tight_layout()
        
    def rad_to_deg(self, rad):
        return rad * 180.0 / np.pi
        
    def joint_state_callback(self, msg):
        try:
            if self.start_time is None:
                self.start_time = self.get_clock().now()
                
            current_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            
            pole_index = msg.name.index('cart_to_pole')
            angle = self.rad_to_deg(msg.position[pole_index])
            velocity = self.rad_to_deg(msg.velocity[pole_index])
            
            self.times.append(current_time)
            self.angles.append(angle)
            self.velocities.append(velocity)
            self.efforts.append(self.current_effort)
            
            # Plot güncelleme kontrolü
            if current_time - self.last_plot_time >= self.plot_interval:
                self.update_plot()
                self.last_plot_time = current_time
                
        except ValueError as e:
            self.get_logger().error(f'Joint state error: {str(e)}')
            
    def effort_callback(self, msg):
        if len(msg.data) > 0:
            self.current_effort = -msg.data[0]
            
    def update_plot(self):
        if len(self.times) < 2:
            return
            
        times_array = np.array(self.times)
        current_time = times_array[-1]
        
        # Son 10 saniyelik veriyi göster
        mask = times_array >= (current_time - 10)
        visible_times = times_array[mask]
        visible_angles = np.array(self.angles)[mask]
        visible_velocities = np.array(self.velocities)[mask]
        visible_efforts = np.array(self.efforts)[mask]
        
        # Verileri güncelle
        self.line_angle.set_data(visible_times, visible_angles)
        self.line_velocity.set_data(visible_times, visible_velocities)
        self.line_effort.set_data(visible_times, visible_efforts)
        
        # X ekseni limitlerini güncelle
        for ax in [self.ax1, self.ax2, self.ax3]:
            ax.set_xlim(max(0, current_time - 10), current_time + 0.5)
        
        # Grafiği yenile
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = SimpleCartPoleDataPlotter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.close('all')

if __name__ == '__main__':
    main()