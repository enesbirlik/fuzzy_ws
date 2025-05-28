import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import threading
import numpy as np

class AdaptiveSwingUpController(Node):
    def __init__(self):
        super().__init__('adaptive_swing_up_controller')

        # Subscribe to joint states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Publisher for effort commands
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/effort_controllers/commands',
            10
        )

        # === SYSTEM PARAMETERS ===
        self.pole_length = 0.24  # 24 cm
        self.pole_mass = 0.05    # 50 gram
        self.gravity = 9.81
        
        # === CONTROL MODES ===
        self.SWING_UP = 0
        self.BALANCE = 1
        self.SAFETY = 2
        self.current_mode = self.SWING_UP
        
        # === STARTUP PROTECTION ===
        self.startup_samples = 50
        self.sample_count = 0
        self.system_ready = False
        
        # === SWING-UP PARAMETERS ===
        self.swing_up_active = True
        self.balance_threshold = math.radians(20)  # 20° dengeye geçiş eşiği (biraz arttırdım)
        self.swing_energy_gain = 400.0  # Enerji kazancı (arttırdım sürtünme için)
        self.swing_damping = 1.2  # Sönümleme (arttırdım)
        self.max_swing_effort = 250.0  # Biraz azalttım
        
        # Energy-based swing-up
        self.desired_energy = self.pole_mass * self.gravity * self.pole_length  # Üst nokta enerjisi
        
        # === BALANCE PARAMETERS ===
        self.balance_kp = 1800.0  # Biraz azalttım
        self.balance_ki = 80.0
        self.balance_kd = 220.0
        self.desired_pole_angle = math.pi  # Üst nokta
        
        # Cart position control (balance sırasında)
        self.cart_kp = 120.0  # Biraz azalttım
        self.cart_kd = 35.0
        self.desired_cart_position = 0.0
        
        # === SAFETY PARAMETERS ===
        self.max_cart_position = 0.18
        self.min_cart_position = -0.18
        self.cart_velocity_limit = 2.5
        self.pole_velocity_limit = 25.0
        
        # Safety controller
        self.safety_kp = 200.0
        self.safety_kd = 50.0
        
        # === STATE VARIABLES ===
        self.current_pole_angle = None
        self.current_cart_position = None
        self.current_cart_velocity = None
        self.current_pole_velocity = None
        
        # === PID STATES ===
        self.balance_integral = 0.0
        self.balance_last_error = 0.0
        
        # === ADAPTIVE PARAMETERS ===
        self.energy_error_history = []
        self.max_history = 20
        self.adaptive_gain_multiplier = 1.0
        self.gain_adaptation_rate = 0.02
        
        # === SAFETY STATES ===
        self.safety_active = False
        self.emergency_stop = False
        
        # === LIMITS ===
        self.max_balance_integral = 15.0
        self.max_effort = 400.0
        
        # === TIMING ===
        self.last_time = self.get_clock().now().nanoseconds / 1e9
        self.timer = self.create_timer(0.01, self.control_loop)  # 100Hz
        
        # === LOGGING ===
        self.log_counter = 0
        self.mode_switch_count = 0
        
        # Send initial zero command
        init_cmd = Float64MultiArray()
        init_cmd.data = [0.0]
        self.publisher.publish(init_cmd)

        # Input thread
        self.input_thread = threading.Thread(target=self.input_handler, daemon=True)
        self.input_thread.start()

        self.get_logger().info('🚀 ADAPTIVE SWING-UP CONTROLLER')
        self.get_logger().info('📊 Collecting initial readings...')
        self.get_logger().info('ENTER: Toggle swing-up | r: Reset to center | q: Quit')

    def joint_state_callback(self, msg: JointState):
        try:
            cart_idx = msg.name.index('slider_to_cart')
            self.current_cart_position = msg.position[cart_idx]
            self.current_cart_velocity = msg.velocity[cart_idx] if len(msg.velocity) > cart_idx else 0.0
            
            pole_idx = msg.name.index('cart_to_pole')
            self.current_pole_angle = msg.position[pole_idx]
            self.current_pole_velocity = msg.velocity[pole_idx] if len(msg.velocity) > pole_idx else 0.0
            
        except ValueError:
            pass

    def input_handler(self):
        while rclpy.ok():
            try:
                command = input().lower()
                
                if command == '':  # ENTER - Toggle swing-up
                    self.swing_up_active = not self.swing_up_active
                    if self.swing_up_active:
                        self.current_mode = self.SWING_UP
                        self.balance_integral = 0.0
                        self.balance_last_error = 0.0
                        self.get_logger().info('🚀 SWING-UP ACTIVATED')
                    else:
                        self.current_mode = self.BALANCE
                        self.get_logger().info('⚖️ BALANCE MODE ONLY')
                
                elif command == 'r':  # Reset to center
                    self.reset_to_center()
                
                elif command == 'q':  # Quit
                    self.emergency_stop = True
                    break
                    
            except:
                break

    def reset_to_center(self):
        """Cart'ı merkeze döndür"""
        self.swing_up_active = False
        self.current_mode = self.SAFETY
        self.desired_cart_position = 0.0
        self.get_logger().info('🔄 RESETTING TO CENTER...')

    def startup_calibration(self):
        """İlk okumalar için bekle"""
        if self.current_pole_angle is None or self.current_cart_position is None:
            return False

        if self.sample_count < self.startup_samples:
            self.sample_count += 1
            if self.sample_count % 10 == 0:
                progress = (self.sample_count / self.startup_samples) * 100
                self.get_logger().info(f'📊 Initializing... {progress:.0f}%')
            return False
        
        if not self.system_ready:
            self.system_ready = True
            angle_deg = math.degrees(self.current_pole_angle)
            cart_pos = self.current_cart_position
            self.get_logger().info(f'✅ SYSTEM READY!')
            self.get_logger().info(f'📍 Initial: Pole={angle_deg:.1f}°, Cart={cart_pos:.3f}m')
            
        return True

    def calculate_pole_energy(self):
        """Pole'un toplam enerjisini hesapla"""
        if self.current_pole_angle is None or self.current_pole_velocity is None:
            return 0.0
        
        # Kinetik enerji: 0.5 * I * ω²
        # I = m * L² (nokta kütle yaklaşımı)
        I = self.pole_mass * (self.pole_length ** 2)
        kinetic_energy = 0.5 * I * (self.current_pole_velocity ** 2)
        
        # Potansiyel enerji: m * g * h
        # h = L * (1 - cos(θ)) (alt noktadan yükseklik)
        potential_energy = self.pole_mass * self.gravity * self.pole_length * (1 - np.cos(self.current_pole_angle))
        
        return kinetic_energy + potential_energy

    def swing_up_control(self):
        """Energy-based swing-up control with STRICT cart limits"""
        if (self.current_pole_angle is None or self.current_cart_position is None or 
            self.current_pole_velocity is None or self.current_cart_velocity is None):
            return 0.0
        
        # === CART LİMİT KONTROLÜ - ÖNCELİKLİ ===
        cart_pos = abs(self.current_cart_position)
        
        # Eğer çok yakınsa swing-up'ı durdur
        if cart_pos > 0.14:  # 14cm - tehlikeli bölge
            # Acil merkez dönüşü
            center_force = -np.sign(self.current_cart_position) * 200.0
            velocity_brake = -self.current_cart_velocity * 50.0
            emergency_effort = center_force + velocity_brake
            self.get_logger().warn(f'🚨 SWING-UP EMERGENCY BRAKE! Cart at {cart_pos:.3f}m')
            return np.clip(emergency_effort, -150.0, 150.0)
        
        # Yumuşak sınırlama bölgesi
        cart_safety_factor = 1.0
        if cart_pos > 0.10:  # 10cm'den sonra azalt
            cart_safety_factor = max(0.2, (0.14 - cart_pos) / 0.04)
            
        # === ENERJİ HESAPLAMA ===
        current_energy = self.calculate_pole_energy()
        energy_error = self.desired_energy - current_energy
        
        # Adaptive gain adjustment
        self.energy_error_history.append(abs(energy_error))
        if len(self.energy_error_history) > self.max_history:
            self.energy_error_history.pop(0)
        
        if len(self.energy_error_history) >= 10:
            recent_avg = np.mean(self.energy_error_history[-10:])
            older_avg = np.mean(self.energy_error_history[-20:-10]) if len(self.energy_error_history) >= 20 else recent_avg
            
            if recent_avg > older_avg * 1.1:
                self.adaptive_gain_multiplier = min(1.5, self.adaptive_gain_multiplier + self.gain_adaptation_rate)
            elif recent_avg < older_avg * 0.9:
                self.adaptive_gain_multiplier = max(0.5, self.adaptive_gain_multiplier - self.gain_adaptation_rate)
        
        # === SWING EFFORT HESAPLAMA ===
        pole_direction = np.sign(self.current_pole_velocity * np.cos(self.current_pole_angle))
        
        # Temel swing effort
        base_swing_effort = (self.swing_energy_gain * self.adaptive_gain_multiplier * 
                           energy_error * pole_direction)
        
        # === CART POZİSYON GÜVENLİK KONTROLÜ ===
        # Eğer cart yanlış yöne gidiyorsa effort'u kes
        cart_vel_direction = np.sign(self.current_cart_velocity)
        cart_pos_direction = np.sign(self.current_cart_position)
        effort_direction = np.sign(base_swing_effort)
        
        # Eğer cart limitlerde ve aynı yöne gidiyorsa effort'u ters çevir veya durdur
        if cart_pos > 0.08:  # 8cm'den sonra dikkatli ol
            if (cart_pos_direction > 0 and effort_direction > 0) or \
               (cart_pos_direction < 0 and effort_direction < 0):
                # Yanlış yön - effort'u azalt veya ters çevir
                if cart_vel_direction == cart_pos_direction:  # Aynı yöne gidiyor
                    base_swing_effort *= -0.3  # Ters yönde küçük effort
                else:
                    base_swing_effort *= 0.1   # Çok küçük effort
        
        # Cart safety factor uygula
        swing_effort = base_swing_effort * cart_safety_factor
        
        # === MERKEZ ÇEKİMİ EKLE ===
        # Cart pozisyonu için merkez çekici kuvvet
        center_attraction = -self.current_cart_position * 100.0 * (cart_pos / 0.15)
        
        # === HIZ SÖNÜMLEMESİ ===
        # Cart hızı için sönümleme - özellikle limitlerde
        velocity_damping_factor = 1.0
        if cart_pos > 0.08:
            velocity_damping_factor = min(5.0, 1.0 + (cart_pos - 0.08) / 0.02 * 4.0)
        
        velocity_damping = -self.swing_damping * velocity_damping_factor * self.current_cart_velocity
        
        # === TOPLAM EFFORT ===
        total_effort = swing_effort + center_attraction + velocity_damping
        
        # === EFFORT LİMİTLEME - CART POZİSYONUNA GÖRE ===
        max_allowed_effort = self.max_swing_effort
        if cart_pos > 0.10:
            max_allowed_effort = max(50.0, self.max_swing_effort * (0.14 - cart_pos) / 0.04)
        
        # Final clipping
        final_effort = np.clip(total_effort, -max_allowed_effort, max_allowed_effort)
        
        # === GÜVENLİK LOGu ===
        if cart_pos > 0.08 and abs(final_effort) > 50:
            if not hasattr(self, 'swing_safety_log_count'):
                self.swing_safety_log_count = 0
            self.swing_safety_log_count += 1
            
            if self.swing_safety_log_count % 20 == 0:  # Her 200ms'de log
                self.get_logger().warn(f'⚠️ SWING SAFETY: Cart={cart_pos:.3f}m, '
                                     f'Factor={cart_safety_factor:.2f}, Effort={final_effort:.1f}')
        
        return final_effort

    def balance_control(self, dt):
        """LQR-style balance control"""
        if dt <= 0:
            return 0.0
        
        # Pole angle error (π = upright)
        pole_error = self.desired_pole_angle - self.current_pole_angle
        
        # Normalize angle error to [-π, π]
        while pole_error > math.pi:
            pole_error -= 2 * math.pi
        while pole_error < -math.pi:
            pole_error += 2 * math.pi
        
        # PID for pole
        self.balance_integral += pole_error * dt
        self.balance_integral = np.clip(self.balance_integral, 
                                       -self.max_balance_integral, 
                                       self.max_balance_integral)
        
        pole_derivative = (pole_error - self.balance_last_error) / dt
        
        # Pole control effort
        pole_effort = -(self.balance_kp * pole_error + 
                       self.balance_ki * self.balance_integral + 
                       self.balance_kd * pole_derivative)
        
        # Cart position control
        cart_error = self.desired_cart_position - self.current_cart_position
        cart_effort = (self.cart_kp * cart_error - 
                      self.cart_kd * self.current_cart_velocity)
        
        # Combine efforts
        total_effort = pole_effort + cart_effort
        
        self.balance_last_error = pole_error
        
        return np.clip(total_effort, -self.max_effort, self.max_effort)

    def safety_control(self):
        """Safety controller - cart'ı merkeze döndür"""
        if (self.current_cart_position is None or self.current_cart_velocity is None):
            return 0.0
        
        # Simple PD to center
        cart_error = -self.current_cart_position  # Target = 0
        effort = (self.safety_kp * cart_error - 
                 self.safety_kd * self.current_cart_velocity)
        
        return np.clip(effort, -200.0, 200.0)

    def check_safety(self):
        """Safety conditions check - daha sıkı cart limitleri"""
        if not self.system_ready:
            return False
        
        # Cart position limits - daha sıkı
        if (self.current_cart_position > 0.16 or  # 16cm limit (eskiden 18cm)
            self.current_cart_position < -0.16):
            return True
        
        # Velocity limits
        if (abs(self.current_cart_velocity) > self.cart_velocity_limit or
            abs(self.current_pole_velocity) > self.pole_velocity_limit):
            return True
        
        return False

    def update_control_mode(self):
        """Control mode state machine"""
        if self.emergency_stop:
            return
        
        # Safety check
        if self.check_safety():
            if not self.safety_active:
                self.safety_active = True
                self.get_logger().warn('🚨 SAFETY MODE ACTIVATED')
            self.current_mode = self.SAFETY
            return
        else:
            if self.safety_active:
                self.safety_active = False
                self.get_logger().info('✅ Safety cleared')
        
        # Normal operation
        if not self.swing_up_active:
            self.current_mode = self.BALANCE
            return
        
        # Check if close to upright for balance mode
        angle_from_upright = abs(self.current_pole_angle - math.pi)
        if angle_from_upright > math.pi:
            angle_from_upright = 2 * math.pi - angle_from_upright
        
        if angle_from_upright < self.balance_threshold:
            if self.current_mode != self.BALANCE:
                self.mode_switch_count += 1
                self.balance_integral = 0.0
                self.balance_last_error = 0.0
                self.get_logger().info(f'⚖️ SWITCHED TO BALANCE MODE (#{self.mode_switch_count})')
            self.current_mode = self.BALANCE
        else:
            if self.current_mode != self.SWING_UP:
                self.get_logger().info('🚀 SWITCHED TO SWING-UP MODE')
            self.current_mode = self.SWING_UP

    def control_loop(self):
        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - self.last_time
        
        if self.emergency_stop:
            zero_cmd = Float64MultiArray()
            zero_cmd.data = [0.0]
            self.publisher.publish(zero_cmd)
            self.last_time = now
            return
        
        if (dt <= 0 or self.current_pole_angle is None or 
            self.current_cart_position is None):
            zero_cmd = Float64MultiArray()
            zero_cmd.data = [0.0]
            self.publisher.publish(zero_cmd)
            self.last_time = now
            return

        # Startup calibration
        if not self.startup_calibration():
            zero_cmd = Float64MultiArray()
            zero_cmd.data = [0.0]
            self.publisher.publish(zero_cmd)
            self.last_time = now
            return

        # Update control mode
        self.update_control_mode()
        
        # Generate control effort based on mode
        if self.current_mode == self.SWING_UP:
            effort = self.swing_up_control()
        elif self.current_mode == self.BALANCE:
            effort = self.balance_control(dt)
        elif self.current_mode == self.SAFETY:
            effort = self.safety_control()
        else:
            effort = 0.0
        
        # Publish command
        cmd = Float64MultiArray()
        cmd.data = [effort]
        self.publisher.publish(cmd)
        
        # Logging
        self.log_counter += 1
        if self.log_counter % 50 == 0:  # Her 0.5 saniyede
            self.log_status(effort)
        
        self.last_time = now

    def log_status(self, effort):
        """Status logging"""
        if not self.system_ready:
            return
        
        angle_deg = math.degrees(self.current_pole_angle)
        angle_from_upright = math.degrees(abs(self.current_pole_angle - math.pi))
        if angle_from_upright > 180:
            angle_from_upright = 360 - angle_from_upright
        
        cart_pos = self.current_cart_position
        cart_vel = self.current_cart_velocity
        
        # Mode string
        mode_str = {
            self.SWING_UP: "🚀 SWING-UP",
            self.BALANCE: "⚖️ BALANCE", 
            self.SAFETY: "🚨 SAFETY"
        }[self.current_mode]
        
        # Energy info for swing-up
        if self.current_mode == self.SWING_UP:
            current_energy = self.calculate_pole_energy()
            energy_pct = (current_energy / self.desired_energy) * 100
            self.get_logger().info(
                f'{mode_str} | Angle: {angle_deg:.1f}° | Energy: {energy_pct:.0f}% | '
                f'Cart: {cart_pos:.3f}m | Effort: {effort:.0f} | Gain: {self.adaptive_gain_multiplier:.2f}'
            )
        else:
            self.get_logger().info(
                f'{mode_str} | From⬆️: {angle_from_upright:.1f}° | '
                f'Cart: {cart_pos:.3f}m ({cart_vel:.2f}m/s) | Effort: {effort:.0f}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveSwingUpController()
    
    print("\n🚀 ADAPTIVE SWING-UP CONTROLLER")
    print("ENTER: Toggle swing-up mode")
    print("r: Reset cart to center")
    print("q: Quit")
    print("=" * 40)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 Controller Stopped")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()