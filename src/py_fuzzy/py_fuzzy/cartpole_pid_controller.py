#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import math
import numpy as np
import threading
import time
import os
import sys
import curses

class CartpoleBalanceController(Node):
    def __init__(self):
        super().__init__('cartpole_balance_controller')
        
        # PID Control Parameters
        self.Kp_angle = 15.0
        self.Ki_angle = 0.1
        self.Kd_angle = 3.0
        
        self.Kp_pos = 4.0
        self.Ki_pos = 0.01
        self.Kd_pos = 1.5
        
        # Adaptive gain parameters
        self.adaptive_enabled = True
        self.adaptive_scale_p = 0.7
        self.adaptive_scale_i = 0.5
        self.adaptive_scale_d = 1.2
        
        # Control Weights
        self.angle_weight = 0.85
        self.pos_weight = 0.15
        
        # Cart Limits
        self.cart_limit = 0.3
        
        # System state
        self.cart_pos = 0.0
        self.cart_vel = 0.0
        self.pole_angle = 0.0
        self.pole_vel = 0.0
        self.angle_deg = 0.0
        
        # Controller variables
        self.angle_integral = 0.0
        self.pos_integral = 0.0
        self.prev_time = self.get_clock().now()
        self.control_started = False
        self.paused = False
        self.centering_cart = False
        self.force_output = 0.0
        
        # Status messages
        self.status_message = "Başlangıç bekleniyor"
        self.last_key_message = ""
        
        # ROS2 communication
        self.state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.state_callback,
            10)
            
        self.command_pub = self.create_publisher(
            Float64MultiArray,
            '/effort_controllers/commands',
            10)
            
        # Control loop - 100Hz
        self.timer = self.create_timer(0.01, self.control_callback)
        
        # UI refresh timer - 5Hz
        self.ui_timer = self.create_timer(0.2, self.update_display)
        
        # Start the UI thread
        self.screen = None
        self.ui_thread = threading.Thread(target=self.ui_loop)
        self.ui_thread.daemon = True
        self.ui_thread.start()
        
        self.get_logger().info('CartPole Denge Kontrolcüsü Başlatıldı')
        
    def ui_loop(self):
        """Terminal UI main loop"""
        try:
            # Initialize curses
            self.screen = curses.initscr()
            curses.noecho()
            curses.cbreak()
            self.screen.keypad(True)
            self.screen.timeout(100)  # Non-blocking input with 100ms timeout
            
            while True:
                # Get keyboard input
                try:
                    key = self.screen.getch()
                    if key != -1:  # -1 means no key pressed
                        self.process_key(key)
                except Exception as e:
                    self.status_message = f"Tuş hatası: {e}"
                
                # Sleep a bit to reduce CPU usage
                time.sleep(0.05)
                
        except Exception as e:
            # Properly clean up curses in case of error
            if self.screen:
                curses.nocbreak()
                self.screen.keypad(False)
                curses.echo()
                curses.endwin()
            print(f"UI hatası: {e}")
            
    def process_key(self, key):
        """Process keyboard input"""
        if key == ord('q'):  # Increase Kp_angle
            self.Kp_angle += 0.5
            self.last_key_message = "Kp_angle arttırıldı"
        elif key == ord('a'):  # Decrease Kp_angle
            self.Kp_angle = max(0, self.Kp_angle - 0.5)
            self.last_key_message = "Kp_angle azaltıldı"
        elif key == ord('w'):  # Increase Ki_angle
            self.Ki_angle += 0.01
            self.last_key_message = "Ki_angle arttırıldı"
        elif key == ord('s'):  # Decrease Ki_angle
            self.Ki_angle = max(0, self.Ki_angle - 0.01)
            self.last_key_message = "Ki_angle azaltıldı"
        elif key == ord('e'):  # Increase Kd_angle
            self.Kd_angle += 0.2
            self.last_key_message = "Kd_angle arttırıldı"
        elif key == ord('d'):  # Decrease Kd_angle
            self.Kd_angle = max(0, self.Kd_angle - 0.2)
            self.last_key_message = "Kd_angle azaltıldı"
        elif key == ord('r'):  # Increase Kp_pos
            self.Kp_pos += 0.2
            self.last_key_message = "Kp_pos arttırıldı"
        elif key == ord('f'):  # Decrease Kp_pos
            self.Kp_pos = max(0, self.Kp_pos - 0.2)
            self.last_key_message = "Kp_pos azaltıldı"
        elif key == ord('1'):  # Toggle adaptive mode
            self.adaptive_enabled = not self.adaptive_enabled
            self.last_key_message = f"Adaptif: {'Açık' if self.adaptive_enabled else 'Kapalı'}"
        elif key == ord('u'):  # Increase angle weight
            self.angle_weight = min(1.0, self.angle_weight + 0.05)
            self.pos_weight = 1.0 - self.angle_weight
            self.last_key_message = "Açı ağırlığı arttırıldı"
        elif key == ord('j'):  # Decrease angle weight
            self.angle_weight = max(0.0, self.angle_weight - 0.05)
            self.pos_weight = 1.0 - self.angle_weight
            self.last_key_message = "Açı ağırlığı azaltıldı"
        elif key == ord(' '):  # Toggle pause/center cart
            self.toggle_pause()
        elif key == ord('x'):  # Exit
            # Clean up curses
            curses.nocbreak()
            self.screen.keypad(False)
            curses.echo()
            curses.endwin()
            # Exit program
            os._exit(0)
            
    def toggle_pause(self):
        """Toggle pause state and center cart"""
        if not self.paused:
            # Pause the controller
            self.paused = True
            self.centering_cart = True
            self.status_message = "DURAKLATİLDI - Araba merkeze çekiliyor"
            self.last_key_message = "Space tuşu: Duraklatıldı"
        else:
            # Resume the controller, but wait for pole to be upright
            self.paused = False
            self.centering_cart = False
            self.control_started = False
            self.angle_integral = 0.0
            self.pos_integral = 0.0
            self.status_message = "Çubuğu yukarı konumlandırın"
            self.last_key_message = "Space tuşu: Devam"
            
    def update_display(self):
        """Update the terminal UI display"""
        if not self.screen:
            return
            
        try:
            self.screen.clear()
            
            # Draw header
            self.screen.addstr(0, 0, "== CartPole PID Kontrolcü ==", curses.A_BOLD)
            
            # Draw PID parameters
            self.screen.addstr(2, 0, "PID Parametreleri:")
            self.screen.addstr(3, 2, f"Kp_angle: {self.Kp_angle:.2f}  [q/a] ")
            self.screen.addstr(4, 2, f"Ki_angle: {self.Ki_angle:.3f}  [w/s] ")
            self.screen.addstr(5, 2, f"Kd_angle: {self.Kd_angle:.2f}  [e/d] ")
            self.screen.addstr(6, 2, f"Kp_pos:   {self.Kp_pos:.2f}  [r/f] ")
            self.screen.addstr(7, 2, f"Adaptif:  {'Açık' if self.adaptive_enabled else 'Kapalı'}  [1] ")
            self.screen.addstr(8, 2, f"Açı Ağr:  {self.angle_weight:.2f}  [u/j] ")
            
            # System status
            self.screen.addstr(10, 0, "Sistem Durumu:")
            self.screen.addstr(11, 2, f"Açı: {self.angle_deg:.1f}°")
            self.screen.addstr(12, 2, f"Poz: {self.cart_pos:.3f} m")
            self.screen.addstr(13, 2, f"Kuvvet: {self.force_output:.2f} N")
            
            # Controller status
            status_y = 15
            self.screen.addstr(status_y, 0, "Kontrol Durumu: ")
            if self.paused:
                self.screen.addstr(status_y, 16, "DURAKLATILDI", curses.A_BOLD)
            elif not self.control_started:
                self.screen.addstr(status_y, 16, "BEKLİYOR", curses.A_BOLD)
            else:
                self.screen.addstr(status_y, 16, "AKTİF", curses.A_BOLD)
            
            # Status message
            self.screen.addstr(16, 0, f"Durum: {self.status_message}")
            
            # Last key message
            if self.last_key_message:
                self.screen.addstr(17, 0, f"Son işlem: {self.last_key_message}")
            
            # Controls reminder
            self.screen.addstr(19, 0, "[Space] Duraklat/Devam   [x] Çıkış")
            
            # Refresh the screen
            self.screen.refresh()
            
        except Exception as e:
            # Catch any drawing errors
            pass
        
    def state_callback(self, msg):
        """Process joint state data from the system"""
        try:
            if len(msg.position) >= 2:
                self.cart_pos = msg.position[0]
                self.pole_angle = msg.position[1]
                
                # Calculate angle in degrees for display
                self.angle_deg = math.degrees(self.normalize_angle(self.pole_angle))
                
                if len(msg.velocity) >= 2:
                    self.cart_vel = msg.velocity[0]
                    self.pole_vel = msg.velocity[1]
                
                # Auto-start control when pole is near upright (if not paused)
                if not self.control_started and not self.paused:
                    normalized_angle = self.normalize_angle(self.pole_angle)
                    if abs(normalized_angle) < math.radians(45):
                        self.control_started = True
                        self.status_message = "Denge kontrolü aktif"
                        self.angle_integral = 0.0
                        self.pos_integral = 0.0
                        self.prev_time = self.get_clock().now()
                
        except Exception as e:
            self.status_message = f"State callback hatası: {str(e)}"
    
    def normalize_angle(self, angle):
        """Normalize angle to -pi to pi range (0 = upright position)"""
        return ((angle + math.pi) % (2 * math.pi)) - math.pi
    
    def calculate_adaptive_gains(self, angle):
        """Calculate adaptive gains based on pole angle"""
        if not self.adaptive_enabled:
            return self.Kp_angle, self.Ki_angle, self.Kd_angle
            
        angle_abs = abs(angle)
        
        # Near upright position (within 5 degrees)
        if angle_abs < math.radians(5.0):
            # Use gentler control parameters when pole is almost upright
            kp = self.Kp_angle * self.adaptive_scale_p  # Default: 70% of normal P gain
            ki = self.Ki_angle * self.adaptive_scale_i  # Default: 50% of normal I gain
            kd = self.Kd_angle * self.adaptive_scale_d  # Default: 120% of normal D gain
        # Transition zone (5-15 degrees)
        elif angle_abs < math.radians(15.0):
            # Linear interpolation between gentle and normal gains
            ratio = (angle_abs - math.radians(5.0)) / math.radians(10.0)
            kp = self.Kp_angle * (self.adaptive_scale_p + ((1.0 - self.adaptive_scale_p) * ratio))
            ki = self.Ki_angle * (self.adaptive_scale_i + ((1.0 - self.adaptive_scale_i) * ratio))
            kd = self.Kd_angle * (self.adaptive_scale_d - ((self.adaptive_scale_d - 1.0) * ratio))
        else:
            # Use normal gain values when far from upright
            kp = self.Kp_angle
            ki = self.Ki_angle
            kd = self.Kd_angle
            
        return kp, ki, kd
    
    def center_cart(self):
        """Simple controller to center the cart"""
        # P controller to center the cart
        kp_center = 5.0
        kd_center = 3.0
        
        # Calculate centering force
        center_force = -kp_center * self.cart_pos - kd_center * self.cart_vel
        
        # Limit force for safety
        max_force = 10.0
        center_force = max(min(center_force, max_force), -max_force)
        
        # Check if cart is centered (position and velocity close to zero)
        is_centered = abs(self.cart_pos) < 0.02 and abs(self.cart_vel) < 0.05
        
        # If cart is centered, stop centering mode
        if is_centered and self.centering_cart:
            self.status_message = "Araba merkeze döndü. Space tuşuna basın."
            self.centering_cart = False
            center_force = 0.0
        
        return center_force
        
    def pid_control(self):
        """PID control algorithm with adaptive gains"""
        # Calculate time difference
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9  # seconds
        
        # Safety check for time step
        if dt <= 0 or dt > 0.1:
            dt = 0.01  # Default time step
        
        # Normalize angle (upright = 0)
        normalized_angle = self.normalize_angle(self.pole_angle)
        
        # Get adaptive gains based on current angle
        kp_angle, ki_angle, kd_angle = self.calculate_adaptive_gains(normalized_angle)
        
        # Calculate errors
        angle_error = normalized_angle  # Target angle = 0 (upright)
        pos_error = self.cart_pos       # Target position = 0 (center)
        
        #----------------------
        # Angle PID Control
        #----------------------
        p_angle = kp_angle * angle_error
        
        self.angle_integral += angle_error * dt
        self.angle_integral = max(min(self.angle_integral, 1.0), -1.0)
        i_angle = ki_angle * self.angle_integral
        
        d_angle = kd_angle * self.pole_vel
        
        angle_control = -(p_angle + i_angle + d_angle)
        
        #----------------------
        # Position PID Control
        #----------------------
        p_pos = self.Kp_pos * pos_error
        
        self.pos_integral += pos_error * dt
        self.pos_integral = max(min(self.pos_integral, 0.5), -0.5)
        i_pos = self.Ki_pos * self.pos_integral
        
        d_pos = self.Kd_pos * self.cart_vel
        
        pos_control = -(p_pos + i_pos + d_pos)
        
        #----------------------
        # Weighted Total Control
        #----------------------
        total_control = (self.angle_weight * angle_control + 
                         self.pos_weight * pos_control)
        
        # Update time
        self.prev_time = current_time
        
        return total_control
    
    def control_callback(self):
        """Main control loop"""
        # Check pause state first
        if self.paused:
            # If in centering mode, run the cart centering controller
            if self.centering_cart:
                control_force = self.center_cart()
            else:
                # Just send zero command while paused
                control_force = 0.0
                
            cmd_msg = Float64MultiArray()
            cmd_msg.data = [float(control_force)]
            self.command_pub.publish(cmd_msg)
            self.force_output = control_force
            return
            
        # Normal control flow
        if not self.control_started:
            cmd_msg = Float64MultiArray()
            cmd_msg.data = [0.0]
            self.command_pub.publish(cmd_msg)
            self.force_output = 0.0
            return
        
        # Calculate control force
        control_force = self.pid_control()
        
        # Apply cart limits - simple version
        if abs(self.cart_pos) > 0.7 * self.cart_limit:
            # Add force to push back toward center
            limit_force = -np.sign(self.cart_pos) * 5.0 * (abs(self.cart_pos) - 0.7 * self.cart_limit) / (0.3 * self.cart_limit)
            control_force += limit_force
        
        # Limit force for safety
        max_force = 15.0
        control_force = max(min(control_force, max_force), -max_force)
        
        # Save for display
        self.force_output = control_force
        
        # Send command
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [float(control_force)]
        self.command_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = CartpoleBalanceController()
        rclpy.spin(controller)
        
    except Exception as e:
        print(f"Hata: {str(e)}")
    finally:
        # Clean up curses in case it's still active
        try:
            curses.nocbreak()
            curses.echo()
            curses.endwin()
        except:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()