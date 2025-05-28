import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Bool
import threading
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

class CartPoleFuzzyController(Node):
    def __init__(self):
        super().__init__('cartpole_fuzzy_controller')

        # Subscribe to joint states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Publisher for effort commands (single interface)
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/effort_controllers/commands',
            10
        )

        # Fuzzy controller setup
        self.setup_fuzzy_controller()

        # Target values
        self.desired_pole_angle = -math.pi  # Target angle
        self.desired_cart_position = 0.0

        # State storage
        self.current_pole_angle = None
        self.current_cart_position = None
        self.current_cart_velocity = None
        self.current_pole_velocity = None

        # Safety parameters
        self.safety_active = False
        self.max_angle_error = math.radians(45)  # 45 degrees in radians
        self.max_cart_position = 0.18  # Cart position limits
        self.min_cart_position = -0.18
        
        # Cart return PID parameters - PWM aralığına göre ayarlandı
        self.cart_return_kp = 50.0
        self.cart_return_ki = 5.0
        self.cart_return_kd = 8.0
        self.cart_return_integral = 0.0
        self.cart_return_last_error = 0.0

        # Initialize timing
        self.last_time = self.get_clock().now().nanoseconds / 1e9
        self.timer = self.create_timer(0.005, self.control_loop)

        # Send initial zero command
        init_cmd = Float64MultiArray()
        init_cmd.data = [0.0]
        self.publisher.publish(init_cmd)

        # Start input thread for reset functionality
        self.input_thread = threading.Thread(target=self.input_handler, daemon=True)
        self.input_thread.start()

        self.get_logger().info('Cartpole Fuzzy Controller started - Detailed Rule Table')
        self.get_logger().info('Safety limits: Pole angle ±45°, Cart position ±0.18')
        self.get_logger().info('Press ENTER to reset system if safety mode is active')

    def setup_fuzzy_controller(self):
        """Detaylı kural tablosu ile fuzzy controller kurulumu"""
        
        # Error (u) ve derivative error (de) için evren tanımları
        # PID kodunuzdaki error aralığına göre ayarlandı
        angle_error = ctrl.Antecedent(np.linspace(-math.pi/2, math.pi/2, 200), 'angle_error')  # ±90 derece
        angle_velocity = ctrl.Antecedent(np.linspace(-15, 15, 200), 'angle_velocity')  # ±15 rad/s
        pwm_output = ctrl.Consequent(np.linspace(-100, 100, 200), 'pwm_output')

        # Error (u) için 7 seviyeli üyelik fonksiyonları
        angle_error['NL'] = fuzz.trimf(angle_error.universe, [-math.pi/2, -math.pi/2, -math.pi/4])      # Negatif Large
        angle_error['NM'] = fuzz.trimf(angle_error.universe, [-math.pi/3, -math.pi/6, -math.pi/12])     # Negatif Medium
        angle_error['NS'] = fuzz.trimf(angle_error.universe, [-math.pi/8, -math.pi/16, -math.pi/32])    # Negatif Small
        angle_error['Z'] = fuzz.trimf(angle_error.universe, [-math.pi/24, 0, math.pi/24])               # Zero
        angle_error['PS'] = fuzz.trimf(angle_error.universe, [math.pi/32, math.pi/16, math.pi/8])       # Pozitif Small
        angle_error['PM'] = fuzz.trimf(angle_error.universe, [math.pi/12, math.pi/6, math.pi/3])        # Pozitif Medium
        angle_error['PL'] = fuzz.trimf(angle_error.universe, [math.pi/4, math.pi/2, math.pi/2])         # Pozitif Large

        # Derivative error (de) için 7 seviyeli üyelik fonksiyonları
        angle_velocity['NL'] = fuzz.trimf(angle_velocity.universe, [-15, -15, -10])     # Negatif Large
        angle_velocity['NM'] = fuzz.trimf(angle_velocity.universe, [-12, -7, -4])       # Negatif Medium
        angle_velocity['NS'] = fuzz.trimf(angle_velocity.universe, [-5, -2, -0.5])      # Negatif Small
        angle_velocity['Z'] = fuzz.trimf(angle_velocity.universe, [-1, 0, 1])           # Zero
        angle_velocity['PS'] = fuzz.trimf(angle_velocity.universe, [0.5, 2, 5])         # Pozitif Small
        angle_velocity['PM'] = fuzz.trimf(angle_velocity.universe, [4, 7, 12])          # Pozitif Medium
        angle_velocity['PL'] = fuzz.trimf(angle_velocity.universe, [10, 15, 15])        # Pozitif Large

        # PWM çıkışı için 7 seviyeli üyelik fonksiyonları
        pwm_output['NL'] = fuzz.trimf(pwm_output.universe, [-100, -100, -70])    # Negatif Large
        pwm_output['NM'] = fuzz.trimf(pwm_output.universe, [-85, -60, -35])      # Negatif Medium
        pwm_output['NS'] = fuzz.trimf(pwm_output.universe, [-45, -25, -8])       # Negatif Small
        pwm_output['Z'] = fuzz.trimf(pwm_output.universe, [-12, 0, 12])          # Zero
        pwm_output['PS'] = fuzz.trimf(pwm_output.universe, [8, 25, 45])          # Pozitif Small
        pwm_output['PM'] = fuzz.trimf(pwm_output.universe, [35, 60, 85])         # Pozitif Medium
        pwm_output['PL'] = fuzz.trimf(pwm_output.universe, [70, 100, 100])       # Pozitif Large

        # Resimdeki kural tablosuna göre 49 kural (7x7 tablo)
        rules = [
            # Error = NL satırı
            ctrl.Rule(angle_error['NL'] & angle_velocity['NL'], pwm_output['NL']),
            ctrl.Rule(angle_error['NL'] & angle_velocity['NM'], pwm_output['NL']),
            ctrl.Rule(angle_error['NL'] & angle_velocity['NS'], pwm_output['NL']),
            ctrl.Rule(angle_error['NL'] & angle_velocity['Z'], pwm_output['NL']),
            ctrl.Rule(angle_error['NL'] & angle_velocity['PS'], pwm_output['NM']),
            ctrl.Rule(angle_error['NL'] & angle_velocity['PM'], pwm_output['NS']),
            ctrl.Rule(angle_error['NL'] & angle_velocity['PL'], pwm_output['Z']),
            
            # Error = NM satırı
            ctrl.Rule(angle_error['NM'] & angle_velocity['NL'], pwm_output['NL']),
            ctrl.Rule(angle_error['NM'] & angle_velocity['NM'], pwm_output['NL']),
            ctrl.Rule(angle_error['NM'] & angle_velocity['NS'], pwm_output['NL']),
            ctrl.Rule(angle_error['NM'] & angle_velocity['Z'], pwm_output['NM']),
            ctrl.Rule(angle_error['NM'] & angle_velocity['PS'], pwm_output['NS']),
            ctrl.Rule(angle_error['NM'] & angle_velocity['PM'], pwm_output['Z']),
            ctrl.Rule(angle_error['NM'] & angle_velocity['PL'], pwm_output['PS']),
            
            # Error = NS satırı
            ctrl.Rule(angle_error['NS'] & angle_velocity['NL'], pwm_output['NL']),
            ctrl.Rule(angle_error['NS'] & angle_velocity['NM'], pwm_output['NL']),
            ctrl.Rule(angle_error['NS'] & angle_velocity['NS'], pwm_output['NM']),
            ctrl.Rule(angle_error['NS'] & angle_velocity['Z'], pwm_output['NS']),
            ctrl.Rule(angle_error['NS'] & angle_velocity['PS'], pwm_output['Z']),
            ctrl.Rule(angle_error['NS'] & angle_velocity['PM'], pwm_output['PS']),
            ctrl.Rule(angle_error['NS'] & angle_velocity['PL'], pwm_output['PM']),
            
            # Error = Z satırı (merkez satır)
            ctrl.Rule(angle_error['Z'] & angle_velocity['NL'], pwm_output['NL']),
            ctrl.Rule(angle_error['Z'] & angle_velocity['NM'], pwm_output['NM']),
            ctrl.Rule(angle_error['Z'] & angle_velocity['NS'], pwm_output['NS']),
            ctrl.Rule(angle_error['Z'] & angle_velocity['Z'], pwm_output['Z']),
            ctrl.Rule(angle_error['Z'] & angle_velocity['PS'], pwm_output['PS']),
            ctrl.Rule(angle_error['Z'] & angle_velocity['PM'], pwm_output['PM']),
            ctrl.Rule(angle_error['Z'] & angle_velocity['PL'], pwm_output['PL']),
            
            # Error = PS satırı
            ctrl.Rule(angle_error['PS'] & angle_velocity['NL'], pwm_output['NM']),
            ctrl.Rule(angle_error['PS'] & angle_velocity['NM'], pwm_output['NS']),
            ctrl.Rule(angle_error['PS'] & angle_velocity['NS'], pwm_output['Z']),
            ctrl.Rule(angle_error['PS'] & angle_velocity['Z'], pwm_output['PS']),
            ctrl.Rule(angle_error['PS'] & angle_velocity['PS'], pwm_output['PM']),
            ctrl.Rule(angle_error['PS'] & angle_velocity['PM'], pwm_output['PL']),
            ctrl.Rule(angle_error['PS'] & angle_velocity['PL'], pwm_output['PL']),
            
            # Error = PM satırı
            ctrl.Rule(angle_error['PM'] & angle_velocity['NL'], pwm_output['NS']),
            ctrl.Rule(angle_error['PM'] & angle_velocity['NM'], pwm_output['Z']),
            ctrl.Rule(angle_error['PM'] & angle_velocity['NS'], pwm_output['PS']),
            ctrl.Rule(angle_error['PM'] & angle_velocity['Z'], pwm_output['PM']),
            ctrl.Rule(angle_error['PM'] & angle_velocity['PS'], pwm_output['PL']),
            ctrl.Rule(angle_error['PM'] & angle_velocity['PM'], pwm_output['PL']),
            ctrl.Rule(angle_error['PM'] & angle_velocity['PL'], pwm_output['PL']),
            
            # Error = PL satırı
            ctrl.Rule(angle_error['PL'] & angle_velocity['NL'], pwm_output['Z']),
            ctrl.Rule(angle_error['PL'] & angle_velocity['NM'], pwm_output['PS']),
            ctrl.Rule(angle_error['PL'] & angle_velocity['NS'], pwm_output['PM']),
            ctrl.Rule(angle_error['PL'] & angle_velocity['Z'], pwm_output['PL']),
            ctrl.Rule(angle_error['PL'] & angle_velocity['PS'], pwm_output['PL']),
            ctrl.Rule(angle_error['PL'] & angle_velocity['PM'], pwm_output['PL']),
            ctrl.Rule(angle_error['PL'] & angle_velocity['PL'], pwm_output['PL']),
        ]

        self.pole_fuzzy_ctrl = ctrl.ControlSystem(rules)
        self.pole_fuzzy_sim = ctrl.ControlSystemSimulation(self.pole_fuzzy_ctrl)

    def joint_state_callback(self, msg: JointState):
        try:
            idx = msg.name.index('slider_to_cart')
            self.current_cart_position = msg.position[idx]
            self.current_cart_velocity = msg.velocity[idx]
            idx2 = msg.name.index('cart_to_pole')
            self.current_pole_angle = msg.position[idx2]
            self.current_pole_velocity = msg.velocity[idx2]
        except ValueError:
            self.get_logger().warn('JointState missing expected names')

    def input_handler(self):
        """Handle user input for system reset"""
        while rclpy.ok():
            try:
                input()
                if self.safety_active:
                    self.reset_system()
            except:
                break

    def reset_system(self):
        """Reset system to normal operation"""
        if self.current_cart_position is not None and abs(self.current_cart_position) < 0.05:
            self.safety_active = False
            self.cart_return_integral = 0.0
            self.cart_return_last_error = 0.0
            self.get_logger().info('System reset - Normal operation resumed')
        else:
            self.get_logger().warn(f'Cannot reset: Cart not at zero position (current: {self.current_cart_position:.3f})')
            self.get_logger().info('Wait for cart to reach zero position before resetting')

    def check_safety_conditions(self):
        """Check if safety conditions are violated"""
        if self.current_pole_angle is None or self.current_cart_position is None:
            return False
        
        angle_error = abs(self.desired_pole_angle - self.current_pole_angle)
        cart_position = self.current_cart_position
        
        safety_triggered = False
        trigger_reason = ""
        
        if angle_error > self.max_angle_error:
            safety_triggered = True
            trigger_reason = f'Pole angle error {math.degrees(angle_error):.1f}° > 45°'
        
        if cart_position > self.max_cart_position:
            safety_triggered = True
            trigger_reason = f'Cart position {cart_position:.3f} > {self.max_cart_position}'
        elif cart_position < self.min_cart_position:
            safety_triggered = True
            trigger_reason = f'Cart position {cart_position:.3f} < {self.min_cart_position}'
        
        if safety_triggered and not self.safety_active:
            self.safety_active = True
            self.cart_return_integral = 0.0
            self.cart_return_last_error = 0.0
            self.get_logger().warn(f'SAFETY ACTIVATED: {trigger_reason}')
            self.get_logger().info('Motors stopped. Cart will return to zero using PID. Press ENTER to reset when cart reaches zero.')
            return True
        
        return self.safety_active

    def cart_return_control(self, dt):
        """Return cart to zero position using PID control"""
        if self.current_cart_position is None or dt <= 0:
            return 0.0
        
        cart_error = 0.0 - self.current_cart_position
        self.cart_return_integral += cart_error * dt
        
        max_integral = 2.0
        self.cart_return_integral = max(min(self.cart_return_integral, max_integral), -max_integral)
        
        cart_derivative = (cart_error - self.cart_return_last_error) / dt
        
        pwm_output = (self.cart_return_kp * cart_error + 
                     self.cart_return_ki * self.cart_return_integral + 
                     self.cart_return_kd * cart_derivative)
        
        self.cart_return_last_error = cart_error
        
        pwm_output = max(min(pwm_output, 100.0), -100.0)
        
        return pwm_output

    def fuzzy_pole_control(self):
        """Detaylı kural tablosu ile pole balance kontrolü"""
        if self.current_pole_angle is None or self.current_pole_velocity is None:
            return 0.0
        
        try:
            # PID kodunuzdaki gibi error hesaplama
            angle_error = self.desired_pole_angle - self.current_pole_angle
            
            # Normalize angle error to [-pi, pi]
            while angle_error > math.pi:
                angle_error -= 2 * math.pi
            while angle_error < -math.pi:
                angle_error += 2 * math.pi
            
            # Derivative error (angle velocity)
            angle_velocity = self.current_pole_velocity
            
            # Fuzzy evren sınırları içinde tut
            angle_error = np.clip(angle_error, -math.pi/2, math.pi/2)
            angle_velocity = np.clip(angle_velocity, -15, 15)
            
            # Fuzzy inference
            self.pole_fuzzy_sim.input['angle_error'] = angle_error
            self.pole_fuzzy_sim.input['angle_velocity'] = angle_velocity
            
            self.pole_fuzzy_sim.compute()
            
            pwm_output = self.pole_fuzzy_sim.output['pwm_output']
            
            # PWM sınırları
            pwm_output = np.clip(pwm_output, -100, 100)
            
            return pwm_output
            
        except Exception as e:
            self.get_logger().error(f'Fuzzy pole control error: {str(e)}')
            return 0.0

    def control_loop(self):
        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - self.last_time
        if dt <= 0 or self.current_pole_angle is None:
            zero_cmd = Float64MultiArray()
            zero_cmd.data = [0.0]
            self.publisher.publish(zero_cmd)
            self.last_time = now
            return

        safety_violation = self.check_safety_conditions()
        
        if safety_violation or self.safety_active:
            # Safety mode: PID ile cart return
            pwm_output = self.cart_return_control(dt)
            
            if self.current_cart_position is not None:
                distance_to_zero = abs(self.current_cart_position)
                self.get_logger().info(f"SAFETY MODE - Cart Pos: {self.current_cart_position:.3f}, "
                                     f"Distance to zero: {distance_to_zero:.3f}, PWM: {pwm_output:.1f}")
                
                if distance_to_zero < 0.05:
                    self.get_logger().info("Cart near zero position - Ready for reset!")
        else:
            # Normal operation: Detaylı fuzzy PWM control
            pwm_output = self.fuzzy_pole_control()
            
            # PID kodunuzdaki gibi negatif işaret
            pwm_output = -pwm_output

            angle_error = self.desired_pole_angle - self.current_pole_angle
            self.get_logger().info(f"Detailed Fuzzy Mode - Angle: {math.degrees(self.current_pole_angle):.1f}°, "
                                 f"Error: {math.degrees(angle_error):.1f}°, "
                                 f"Velocity: {self.current_pole_velocity:.2f}, PWM: {pwm_output:.1f}")

        # PWM komutunu gönder
        cmd = Float64MultiArray()
        cmd.data = [pwm_output]
        self.publisher.publish(cmd)

        self.last_time = now


def main(args=None):
    rclpy.init(args=args)
    node = CartPoleFuzzyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()