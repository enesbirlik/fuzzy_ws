import math
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
from collections import deque
import random
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Bool
import threading
import os
from datetime import datetime

# GPU kullanımı
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

class ReplayBuffer:
    def __init__(self, capacity=100000):
        self.buffer = deque(maxlen=capacity)
    
    def push(self, state, action, reward, next_state, done):
        self.buffer.append((state, action, reward, next_state, done))
    
    def sample(self, batch_size):
        batch = random.sample(self.buffer, batch_size)
        state, action, reward, next_state, done = map(np.stack, zip(*batch))
        return state, action, reward, next_state, done
    
    def __len__(self):
        return len(self.buffer)

class Actor(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dim=256):
        super(Actor, self).__init__()
        self.fc1 = nn.Linear(state_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.fc3 = nn.Linear(hidden_dim, hidden_dim)
        self.mean = nn.Linear(hidden_dim, action_dim)
        self.log_std = nn.Linear(hidden_dim, action_dim)
        
        self.apply(self._init_weights)
        
    def _init_weights(self, module):
        if isinstance(module, nn.Linear):
            torch.nn.init.xavier_uniform_(module.weight)
            torch.nn.init.constant_(module.bias, 0)
    
    def forward(self, state):
        x = F.relu(self.fc1(state))
        x = F.relu(self.fc2(x))
        x = F.relu(self.fc3(x))
        mean = self.mean(x)
        log_std = self.log_std(x)
        log_std = torch.clamp(log_std, -20, 2)
        return mean, log_std
    
    def sample(self, state):
        mean, log_std = self.forward(state)
        std = log_std.exp()
        normal = torch.distributions.Normal(mean, std)
        x_t = normal.rsample()
        action = torch.tanh(x_t)
        log_prob = normal.log_prob(x_t)
        log_prob -= torch.log(1 - action.pow(2) + 1e-6)
        log_prob = log_prob.sum(1, keepdim=True)
        return action, log_prob

class Critic(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dim=256):
        super(Critic, self).__init__()
        # Q1
        self.fc1 = nn.Linear(state_dim + action_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.fc3 = nn.Linear(hidden_dim, hidden_dim)
        self.q1 = nn.Linear(hidden_dim, 1)
        
        # Q2
        self.fc4 = nn.Linear(state_dim + action_dim, hidden_dim)
        self.fc5 = nn.Linear(hidden_dim, hidden_dim)
        self.fc6 = nn.Linear(hidden_dim, hidden_dim)
        self.q2 = nn.Linear(hidden_dim, 1)
        
        self.apply(self._init_weights)
        
    def _init_weights(self, module):
        if isinstance(module, nn.Linear):
            torch.nn.init.xavier_uniform_(module.weight)
            torch.nn.init.constant_(module.bias, 0)
    
    def forward(self, state, action):
        x = torch.cat([state, action], 1)
        
        x1 = F.relu(self.fc1(x))
        x1 = F.relu(self.fc2(x1))
        x1 = F.relu(self.fc3(x1))
        q1 = self.q1(x1)
        
        x2 = F.relu(self.fc4(x))
        x2 = F.relu(self.fc5(x2))
        x2 = F.relu(self.fc6(x2))
        q2 = self.q2(x2)
        
        return q1, q2

class AdaptiveSwingUpController(Node):
    def __init__(self):
        super().__init__('adaptive_swing_up_controller')
        
        # Sistem parametreleri
        self.pole_length = 0.24  # 24 cm
        self.pole_mass = 0.1     # 100 gram
        self.gravity = 9.81
        
        # ULTRA SIKI GÜVENLİK LİMİTLERİ
        self.cart_hard_limit = 0.15      # MUTLAK LİMİT - 15cm
        self.cart_emergency_zone = 0.13   # ACİL DURUM - 13cm
        self.cart_danger_zone = 0.11     # TEHLİKE - 11cm  
        self.cart_soft_limit = 0.09      # YUMUŞAK - 9cm
        self.cart_warning_zone = 0.07    # UYARI - 7cm
        self.max_effort = 100.0
        
        # ENCODER KAYIT SİSTEMİ
        self.position_history = deque(maxlen=50)  # Son 250ms
        self.velocity_history = deque(maxlen=20)  # Son 100ms
        self.effort_history = deque(maxlen=20)    # Son effort'lar
        
        # ULTRA GÜVENLİK DEĞİŞKENLERİ
        self.emergency_override = False
        self.emergency_steps = 0
        self.emergency_brake_active = False
        self.last_safe_position = 0.0
        self.velocity_trend = deque(maxlen=5)
        self.consecutive_wall_approach = 0
        self.safety_violation_count = 0
        
        # MUTLAK DUVAR ÇEKİCİ
        self.wall_repulsion_force = 200.0
        self.emergency_brake_force = 150.0
        
        # State değişkenleri
        self.cart_position = 0.0
        self.cart_velocity = 0.0
        self.pole_angle = 0.0
        self.pole_velocity = 0.0
        
        # Güvenlik durumu
        self.in_danger_zone = False
        self.wall_hit_count = 0
        self.consecutive_danger_steps = 0
        self.episode_danger_time = 0
        
        # Cart return için basit PD kontrol
        self.return_kp = 150.0
        self.return_kd = 30.0
        self.cart_returning = False
        
        # ROS2 setup
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/effort_controllers/commands',
            10
        )
        
        # SAC parametreleri
        self.state_dim = 7  # [cart_pos, cart_vel, cos(theta), sin(theta), theta_dot, prev_action, danger_factor]
        self.action_dim = 1
        self.batch_size = 256
        self.gamma = 0.99
        self.tau = 0.005
        self.alpha = 0.2
        self.lr = 3e-4
        
        # Ağları oluştur
        self.actor = Actor(self.state_dim, self.action_dim).to(device)
        self.critic = Critic(self.state_dim, self.action_dim).to(device)
        self.critic_target = Critic(self.state_dim, self.action_dim).to(device)
        self.critic_target.load_state_dict(self.critic.state_dict())
        
        # Optimizers
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=self.lr)
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=self.lr)
        
        # Replay buffer
        self.replay_buffer = ReplayBuffer()
        
        # Eğitim değişkenleri
        self.training_active = False
        self.episode_count = 0
        self.step_count = 0
        self.episode_reward = 0
        self.episode_steps = 0
        self.best_reward = -float('inf')
        self.prev_action = 0.0
        self.prev_state = None
        
        # Exploration parametreleri
        self.exploration_noise = 0.1
        self.min_exploration = 0.01
        self.exploration_decay = 0.999
        
        # Performans takibi
        self.reward_history = deque(maxlen=100)
        self.successful_episodes = 0
        self.wall_hits_total = 0
        
        # Model kayıt
        self.model_dir = os.path.expanduser('~/cartpole_models')
        os.makedirs(self.model_dir, exist_ok=True)
        
        # Timers
        self.control_timer = self.create_timer(0.01, self.control_loop)  # 100 Hz - STM ile uyumlu
        self.last_control_time = self.get_clock().now().nanoseconds / 1e9
        
        # Logging - güvenlik bilgisi
        self.get_logger().info('ULTRA SAFE Adaptive Swing-Up Controller başlatıldı')
        self.get_logger().info(f'SIKI LİMİTLER: Warning={self.cart_warning_zone}, Soft={self.cart_soft_limit}, Danger={self.cart_danger_zone}, Emergency={self.cart_emergency_zone}, HARD={self.cart_hard_limit}')
        
        # Input thread
        self.input_thread = threading.Thread(target=self.input_handler, daemon=True)
        self.input_thread.start()
        
    def joint_state_callback(self, msg: JointState):
        try:
            cart_idx = msg.name.index('slider_to_cart')
            pole_idx = msg.name.index('cart_to_pole')
            
            # Önceki değerleri kaydet
            prev_position = self.cart_position
            prev_velocity = self.cart_velocity
            
            self.cart_position = msg.position[cart_idx]
            self.cart_velocity = msg.velocity[cart_idx]
            self.pole_angle = msg.position[pole_idx]
            self.pole_velocity = msg.velocity[pole_idx]
            
            # ENCODER KAYIT
            current_time = self.get_clock().now().nanoseconds / 1e9
            self.position_history.append((current_time, self.cart_position))
            self.velocity_history.append((current_time, self.cart_velocity))
            
            # MUTLAK GÜVENLİK KONTROLÜ
            if abs(self.cart_position) >= self.cart_hard_limit:
                self.get_logger().error(f'MUTLAK LİMİT AŞILDI! Pos: {self.cart_position:.4f}')
                self.safety_violation_count += 1
                
                # ACİL DURDUR
                self.emergency_brake_active = True
                self.training_active = False
            
        except ValueError:
            self.get_logger().warn('Joint state isimleri bulunamadı')
    
    def get_velocity_acceleration(self):
        """Hızlanma hesapla"""
        if len(self.velocity_history) < 3:
            return 0.0
        
        recent_velocities = [v[1] for v in list(self.velocity_history)[-3:]]
        recent_times = [v[0] for v in list(self.velocity_history)[-3:]]
        
        if len(recent_velocities) >= 2:
            dt = recent_times[-1] - recent_times[-2]
            if dt > 0:
                acceleration = (recent_velocities[-1] - recent_velocities[-2]) / dt
                return acceleration
        return 0.0
    
    def predict_collision_time(self):
        """Çarpışma zamanını tahmin et"""
        if abs(self.cart_velocity) < 0.01:
            return float('inf')
        
        # Hangi duvara gidiyor?
        if self.cart_velocity > 0:
            distance_to_wall = self.cart_hard_limit - self.cart_position
        else:
            distance_to_wall = self.cart_hard_limit + self.cart_position
        
        if distance_to_wall <= 0:
            return 0.0  # Zaten duvarı geçti
        
        # Hızlanma dahil tahmin
        acceleration = self.get_velocity_acceleration()
        velocity = self.cart_velocity
        
        # Kinematik denklem: d = vt + 0.5*a*t^2
        # 0.5*a*t^2 + v*t - d = 0
        if abs(acceleration) > 0.001:
            discriminant = velocity**2 + 2 * acceleration * distance_to_wall
            if discriminant >= 0:
                t1 = (-velocity + np.sqrt(discriminant)) / acceleration
                t2 = (-velocity - np.sqrt(discriminant)) / acceleration
                # Pozitif ve en küçük zamanı al
                times = [t for t in [t1, t2] if t > 0]
                if times:
                    return min(times)
        
        # Sabit hız tahmini
        return distance_to_wall / abs(velocity)
    
    def predict_future_position(self, steps=10):
        """ÇOK DETAYLI pozisyon tahmini"""
        dt = 0.005  # 5ms timestep
        pos = self.cart_position
        vel = self.cart_velocity
        
        # Hızlanma etkisi
        acceleration = self.get_velocity_acceleration()
        
        # Son effort'un etkisi (basit model)
        if len(self.effort_history) > 0:
            last_effort = self.effort_history[-1]
            # Effort'un hızlanmaya etkisi (basit tahmin)
            effort_acceleration = last_effort * 0.001  # Tuning gerekebilir
            acceleration += effort_acceleration
        
        # Çok adımlı tahmin
        predicted_positions = []
        for step in range(steps):
            t = dt * (step + 1)
            # s = ut + 0.5*a*t^2
            predicted_pos = pos + vel * t + 0.5 * acceleration * t**2
            predicted_positions.append(predicted_pos)
        
        # En kötü durumu döndür
        return predicted_positions[-1]
    
    def get_danger_factor(self):
        """ULTRA agresif tehlike faktörü"""
        cart_abs = abs(self.cart_position)
        
        # Çoklu tahmin
        future_pos_1 = abs(self.predict_future_position(5))   # 25ms
        future_pos_2 = abs(self.predict_future_position(15))  # 75ms
        future_pos_3 = abs(self.predict_future_position(30))  # 150ms
        
        # En kötü senaryo
        worst_pos = max(cart_abs, future_pos_1, future_pos_2, future_pos_3)
        
        # Çarpışma zamanı faktörü
        collision_time = self.predict_collision_time()
        time_factor = 1.0
        if collision_time < 0.1:  # 100ms'den az
            time_factor = 2.0
        elif collision_time < 0.2:  # 200ms'den az
            time_factor = 1.5
        
        # Tehlike hesaplama
        if worst_pos < self.cart_warning_zone:
            danger = 0.0
        elif worst_pos < self.cart_soft_limit:
            danger = (worst_pos - self.cart_warning_zone) / (self.cart_soft_limit - self.cart_warning_zone) * 0.2
        elif worst_pos < self.cart_danger_zone:
            danger = 0.2 + (worst_pos - self.cart_soft_limit) / (self.cart_danger_zone - self.cart_soft_limit) * 0.3
        elif worst_pos < self.cart_emergency_zone:
            danger = 0.5 + (worst_pos - self.cart_danger_zone) / (self.cart_emergency_zone - self.cart_danger_zone) * 0.3
        elif worst_pos < self.cart_hard_limit:
            danger = 0.8 + (worst_pos - self.cart_emergency_zone) / (self.cart_hard_limit - self.cart_emergency_zone) * 0.2
        else:
            danger = 1.0  # MAXIMUM DANGER
        
        return min(1.0, danger * time_factor)
    
    def get_state(self):
        """State vektörü - tehlike faktörü dahil"""
        cos_theta = np.cos(self.pole_angle)
        sin_theta = np.sin(self.pole_angle)
        danger_factor = self.get_danger_factor()
        
        state = np.array([
            self.cart_position / self.cart_hard_limit,
            self.cart_velocity / 2.0,
            cos_theta,
            sin_theta,
            self.pole_velocity / 10.0,
            self.prev_action / self.max_effort,
            danger_factor  # Tehlike seviyesi
        ], dtype=np.float32)
        
        return state
    
    def reset_episode(self):
        """Yeni episode başlat"""
        self.episode_reward = 0
        self.episode_steps = 0
        self.prev_action = 0.0
        self.prev_state = None
        self.consecutive_danger_steps = 0
        self.episode_danger_time = 0
        self.safety_violation_count = 0  # Reset safety violations
        
        # Emergency durumlarını temizle
        self.emergency_override = False
        self.emergency_steps = 0
        self.emergency_brake_active = False
        
        # Cart pozisyonu kontrolü
        if abs(self.cart_position) > 0.10:  # 10cm tolerans
            self.get_logger().info(f'Cart merkezde değil ({self.cart_position:.3f}m), merkeze dönülüyor...')
            self.cart_returning = True
            return False
        
        self.get_logger().info(f'Episode {self.episode_count} başlıyor...')
        return True
    
    def cart_return_control(self):
        """Basit PD kontrol ile cart'ı merkeze döndür"""
        error = -self.cart_position
        control = self.return_kp * error - self.return_kd * self.cart_velocity
        
        # Limit effort
        control = np.clip(control, -80.0, 80.0)
        
        # Merkeze yakınsa dur
        if abs(self.cart_position) < 0.05 and abs(self.cart_velocity) < 0.1:
            self.cart_returning = False
            self.get_logger().info('Cart merkeze döndü, eğitim devam ediyor...')
            self.reset_episode()
            return 0.0
        
        return control
    
    def update_networks(self):
        """SAC update"""
        if len(self.replay_buffer) < self.batch_size:
            return
        
        # Sample batch
        state, action, reward, next_state, done = self.replay_buffer.sample(self.batch_size)
        
        state = torch.FloatTensor(state).to(device)
        action = torch.FloatTensor(action).to(device)
        reward = torch.FloatTensor(reward).unsqueeze(1).to(device)
        next_state = torch.FloatTensor(next_state).to(device)
        done = torch.FloatTensor(done).unsqueeze(1).to(device)
        
        # Critic update
        with torch.no_grad():
            next_action, next_log_prob = self.actor.sample(next_state)
            target_q1, target_q2 = self.critic_target(next_state, next_action)
            target_q = torch.min(target_q1, target_q2) - self.alpha * next_log_prob
            target_q = reward + (1 - done) * self.gamma * target_q
        
        current_q1, current_q2 = self.critic(state, action)
        critic_loss = F.mse_loss(current_q1, target_q) + F.mse_loss(current_q2, target_q)
        
        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        torch.nn.utils.clip_grad_norm_(self.critic.parameters(), 1.0)
        self.critic_optimizer.step()
        
        # Actor update
        new_action, log_prob = self.actor.sample(state)
        q1, q2 = self.critic(state, new_action)
        q = torch.min(q1, q2)
        actor_loss = (self.alpha * log_prob - q).mean()
        
        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        torch.nn.utils.clip_grad_norm_(self.actor.parameters(), 1.0)
        self.actor_optimizer.step()
        
        # Soft update
        for param, target_param in zip(self.critic.parameters(), self.critic_target.parameters()):
            target_param.data.copy_(self.tau * param.data + (1 - self.tau) * target_param.data)
    
    def on_episode_end(self):
        """Episode sonu"""
        self.reward_history.append(self.episode_reward)
        avg_reward = np.mean(self.reward_history) if len(self.reward_history) > 0 else 0
        
        # Success check
        if self.episode_reward > 10000 and self.wall_hit_count == 0:
            self.successful_episodes += 1
        
        success_rate = (self.successful_episodes / (self.episode_count + 1)) * 100 if self.episode_count > 0 else 0
        
        self.get_logger().info(
            f'Episode {self.episode_count} | '
            f'Reward: {self.episode_reward:.1f} | '
            f'Avg: {avg_reward:.1f} | '
            f'Success: {success_rate:.1f}% | '
            f'Wall hits: {self.wall_hit_count} | '
            f'Danger time: {self.episode_danger_time:.1f}s | '
            f'Exploration: {self.exploration_noise:.3f}'
        )
        
        # Save best model
        if self.episode_reward > self.best_reward and self.wall_hit_count == 0:
            self.best_reward = self.episode_reward
            self.save_model('best')
            self.get_logger().info(f'Yeni en iyi model! Reward: {self.best_reward:.1f}')
        
        # Periodic save
        if self.episode_count % 100 == 0:
            self.save_model(f'checkpoint_{self.episode_count}')
        
        # Reset counters
        self.wall_hit_count = 0
        self.episode_count += 1
        
        # Start new episode
        self.reset_episode()
    
    def save_model(self, name):
        """Model kaydet"""
        path = os.path.join(self.model_dir, f'{name}.pth')
        torch.save({
            'actor_state_dict': self.actor.state_dict(),
            'critic_state_dict': self.critic.state_dict(),
            'critic_target_state_dict': self.critic_target.state_dict(),
            'episode_count': self.episode_count,
            'best_reward': self.best_reward,
            'wall_hits_total': self.wall_hits_total,
            'exploration_noise': self.exploration_noise
        }, path)
        self.get_logger().info(f'Model kaydedildi: {path}')
    
    def load_model(self, name):
        """Model yükle"""
        path = os.path.join(self.model_dir, f'{name}.pth')
        if os.path.exists(path):
            checkpoint = torch.load(path, map_location=device)
            self.actor.load_state_dict(checkpoint['actor_state_dict'])
            self.critic.load_state_dict(checkpoint['critic_state_dict'])
            self.critic_target.load_state_dict(checkpoint['critic_target_state_dict'])
            self.episode_count = checkpoint.get('episode_count', 0)
            self.best_reward = checkpoint.get('best_reward', -float('inf'))
            self.wall_hits_total = checkpoint.get('wall_hits_total', 0)
            self.exploration_noise = checkpoint.get('exploration_noise', 0.1)
            self.get_logger().info(f'Model yüklendi: {path}')
            return True
        return False
    
    def input_handler(self):
        """Kullanıcı komutları"""
        while rclpy.ok():
            try:
                command = input().lower()
                
                if command == '':  # ENTER
                    self.training_active = not self.training_active
                    if self.training_active:
                        self.get_logger().info('Eğitim BAŞLADI')
                        if not self.reset_episode():
                            self.cart_returning = True
                    else:
                        self.get_logger().info('Eğitim DURDURULDU')
                
                elif command == 'r':  # Reset
                    self.training_active = False
                    self.cart_returning = True
                    self.get_logger().info('Sistem resetleniyor...')
                
                elif command == 's':  # Save
                    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                    self.save_model(f'manual_{timestamp}')
                
                elif command == 'l':  # Load
                    if self.load_model('best'):
                        self.exploration_noise = self.min_exploration  # Test için exploration kapat
                
                elif command == 'q':  # Quit
                    break
                    
            except:
                break
    
    def calculate_reward(self):
        """ULTRA AĞIR ceza sistemi"""
        # Encoder kayıt
        self.velocity_trend.append(self.cart_velocity)
        
        # Açı hatası
        angle_error = abs(self.pole_angle - math.pi)
        
        # Tehlike faktörleri
        danger = self.get_danger_factor()
        collision_time = self.predict_collision_time()
        
        # Base reward
        if angle_error < 0.2:  # Dengede
            base_reward = 15.0 - angle_error * 30.0
            
            # Merkeze yakınsa büyük bonus
            if abs(self.cart_position) < 0.04:
                base_reward += 10.0
            elif abs(self.cart_position) < 0.06:
                base_reward += 5.0
        else:
            # Swing-up
            height = self.pole_length * (1 + np.cos(self.pole_angle))
            base_reward = height * 2.0
        
        # ULTRA AĞIR DUVAR CEZALARI
        danger_penalty = 0.0
        if danger > 0:
            if danger < 0.2:  # Warning zone
                danger_penalty = danger * 50.0
            elif danger < 0.5:  # Soft/Danger zone
                danger_penalty = 50.0 + (danger - 0.2) * 200.0
            elif danger < 0.8:  # Emergency approach
                danger_penalty = 110.0 + (danger - 0.5) * 400.0
            else:  # Near hard limit
                danger_penalty = 230.0 + (danger - 0.8) * 1000.0
        
        # Çarpışma zamanı cezası
        collision_penalty = 0.0
        if collision_time < 0.5:
            collision_penalty = (0.5 - collision_time) * 300.0
        
        # Velocity penalty - limitlerde hız olmamalı
        velocity_penalty = 0.0
        if abs(self.cart_position) > self.cart_warning_zone:
            vel_factor = min(abs(self.cart_velocity), 3.0)
            pos_factor = (abs(self.cart_position) - self.cart_warning_zone) / (self.cart_emergency_zone - self.cart_warning_zone)
            velocity_penalty = vel_factor * pos_factor * 100.0
        
        # Yanlış yön MEGA ceza
        wrong_direction_penalty = 0.0
        if (self.cart_position > self.cart_soft_limit and self.cart_velocity > 0.2) or \
           (self.cart_position < -self.cart_soft_limit and self.cart_velocity < -0.2):
            wrong_direction_penalty = abs(self.cart_velocity) * 500.0
        
        # ULTIMATE WALL DEATH PENALTY
        wall_penalty = 0.0
        if abs(self.cart_position) >= self.cart_hard_limit:
            wall_penalty = 50000.0  # Episode destroyer
            self.wall_hit_count += 1
            self.wall_hits_total += 1
            self.get_logger().error(f'DUVAR ÇARPMASI! Pos: {self.cart_position:.4f}, Toplam: {self.wall_hits_total}')
        
        # Safety violation penalty
        safety_penalty = self.safety_violation_count * 1000.0
        
        # Toplam reward
        reward = base_reward - danger_penalty - collision_penalty - velocity_penalty - wrong_direction_penalty - wall_penalty - safety_penalty
        
        return reward
    
    def get_safe_action(self, state):
        """MUTLAK GÜVENLİK aksiyon sistemi"""
        # Encoder kayıt
        self.velocity_trend.append(self.cart_velocity)
        
        # ACİL FREN kontrolü
        if self.emergency_brake_active:
            if abs(self.cart_position) > self.cart_emergency_zone:
                # Güçlü fren
                if self.cart_position > 0:
                    effort = -self.emergency_brake_force
                else:
                    effort = self.emergency_brake_force
                self.get_logger().error(f'ACİL FREN AKTİF! Pos: {self.cart_position:.4f}, Effort: {effort}')
                return 0.0, effort
            else:
                # Güvenli bölgeye döndü
                self.emergency_brake_active = False
                self.get_logger().info('Acil fren deaktif - güvenli bölge')
        
        # Neural network action
        with torch.no_grad():
            state_tensor = torch.FloatTensor(state).unsqueeze(0).to(device)
            
            if self.training_active and np.random.random() < self.exploration_noise:
                action, _ = self.actor.sample(state_tensor)
                action = action.cpu().numpy()[0]
            else:
                mean, _ = self.actor(state_tensor)
                action = torch.tanh(mean).cpu().numpy()[0]
        
        effort = float(action[0] * self.max_effort)
        
        # Çoklu pozisyon tahmini (100Hz için düzeltme)
        future_pos_fast = self.predict_future_position(2)    # 20ms - çok hızlı
        future_pos_short = self.predict_future_position(5)   # 50ms - hızlı
        future_pos_medium = self.predict_future_position(10) # 100ms - orta
        future_pos_long = self.predict_future_position(20)   # 200ms - uzun
        
        danger = self.get_danger_factor()
        collision_time = self.predict_collision_time()
        
        # LEVEL 1: MUTLAK ACİL DURUM
        if (abs(self.cart_position) >= self.cart_emergency_zone or 
            abs(future_pos_fast) >= self.cart_emergency_zone or
            collision_time < 0.05):
            
            self.emergency_override = True
            self.emergency_steps += 1
            
            # WALL REPULSION - Duvara göre güçlü itme
            if self.cart_position > 0:
                effort = -self.wall_repulsion_force
            else:
                effort = self.wall_repulsion_force
            
            # Hız değerlendirme - yanlış yöne gidiyorsa daha güçlü
            if (self.cart_position > 0 and self.cart_velocity > 0) or \
               (self.cart_position < 0 and self.cart_velocity < 0):
                effort *= 1.8  # %80 artır
            
            effort = np.clip(effort, -self.max_effort, self.max_effort)
            
            self.get_logger().error(f'LEVEL 1 ACİL! Pos: {self.cart_position:.4f}, CollTime: {collision_time:.3f}, Effort: {effort:.1f}')
        
        # LEVEL 2: YÜKSEK TEHLİKE  
        elif (abs(future_pos_short) >= self.cart_emergency_zone or 
              abs(future_pos_medium) >= self.cart_danger_zone or
              danger > 0.8 or collision_time < 0.1):
            
            # Neural network'ü çok sınırla
            effort *= 0.1
            
            # Güçlü merkez çekimi
            center_force = -self.cart_position * 180.0
            
            # Hız sönümleme
            velocity_damping = -self.cart_velocity * 60.0
            
            # Kombine et
            effort = effort * 0.2 + center_force * 0.5 + velocity_damping * 0.3
            effort = np.clip(effort, -90.0, 90.0)
            
            self.get_logger().warn(f'LEVEL 2 TEHLİKE! Danger: {danger:.2f}, CollTime: {collision_time:.3f}')
        
        # LEVEL 3: ORTA TEHLİKE
        elif (abs(future_pos_long) >= self.cart_danger_zone or 
              danger > 0.5 or collision_time < 0.2):
            
            # Orta seviye müdahale
            effort *= 0.5
            
            # Merkez yönlendirme
            center_force = -self.cart_position * 100.0
            effort = effort * 0.6 + center_force * 0.4
            
            # Yanlış yön kontrolü
            if (self.cart_position > 0 and effort > 0) or (self.cart_position < 0 and effort < 0):
                effort *= 0.2
            
            effort = np.clip(effort, -70.0, 70.0)
        
        # LEVEL 4: DÜŞÜK TEHLİKE
        elif danger > 0.2:
            # Hafif müdahale
            effort *= 0.8
            
            # Hafif merkez çekimi
            center_force = -self.cart_position * 30.0
            effort = effort * 0.9 + center_force * 0.1
        
        # SON KONTROL - Her durumda yapılacak kontroller
        
        # Effort kaydı
        self.effort_history.append(effort)
        
        # Test effort ile son kontrol (100Hz için düzeltme)
        test_future = abs(self.cart_position + self.cart_velocity * 0.02 + effort * 0.0001)
        if test_future > self.cart_emergency_zone:
            if self.cart_position > 0:
                effort = min(effort, -50.0)
            else:
                effort = max(effort, 50.0)
            self.get_logger().warn(f'SON KONTROL MÜDAHALE! TestPos: {test_future:.4f}')
        
        # Emergency reset
        if not (abs(self.cart_position) >= self.cart_emergency_zone) and abs(self.cart_position) < self.cart_danger_zone:
            if self.emergency_override:
                self.get_logger().info(f'Emergency cleared after {self.emergency_steps} steps')
            self.emergency_override = False
            self.emergency_steps = 0
        
        return action[0], effort
    
    def is_episode_done(self):
        """Episode bitme - daha sıkı"""
        # MUTLAK LİMİT
        if abs(self.cart_position) >= self.cart_hard_limit:
            self.get_logger().error(f'Episode bitti - Hard limit aşıldı: {self.cart_position:.4f}')
            return True
        
        # Güvenlik ihlali
        if self.safety_violation_count > 0:
            self.get_logger().error('Episode bitti - Güvenlik ihlali')
            return True
        
        # Çok uzun emergency (100Hz için düzeltme)
        if self.emergency_steps > 50:  # 500ms
            self.get_logger().warn('Episode bitti - Emergency çok uzun')
            return True
        
        # Max steps (100Hz için düzeltme)
        if self.episode_steps >= 3000:  # 30 saniye
            return True
        
        return False
    
    def control_loop(self):
        """Ana kontrol döngüsü - ekstra güvenlik"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_control_time
        self.last_control_time = current_time
        
        # MUTLAK GÜVENLİK KONTROLÜ
        if abs(self.cart_position) >= self.cart_hard_limit:
            # ACİL DURDUR
            self.get_logger().error(f'CONTROL LOOP SAFETY: Position {self.cart_position:.4f} exceeds hard limit!')
            cmd = Float64MultiArray()
            if self.cart_position > 0:
                cmd.data = [-self.max_effort]
            else:
                cmd.data = [self.max_effort]
            self.publisher.publish(cmd)
            self.training_active = False
            return
        
        # Cart return modunda
        if self.cart_returning:
            effort = self.cart_return_control()
            cmd = Float64MultiArray()
            cmd.data = [effort]
            self.publisher.publish(cmd)
            return
        
        # Training aktif değilse
        if not self.training_active:
            cmd = Float64MultiArray()
            cmd.data = [0.0]
            self.publisher.publish(cmd)
            return
        
        # Get state
        state = self.get_state()
        
        # Get action with MUTLAK safety
        action_normalized, effort = self.get_safe_action(state)
        
        # SON MUTLAK KONTROL
        predicted_pos = self.cart_position + self.cart_velocity * 0.02  # 100Hz için 20ms
        if abs(predicted_pos) > self.cart_emergency_zone:
            self.get_logger().error(f'SON MUTLAK KONTROL: Predicted pos {predicted_pos:.4f} dangerous!')
            if predicted_pos > 0:
                effort = -80.0
            else:
                effort = 80.0
        
        # Publish command
        cmd = Float64MultiArray()
        cmd.data = [effort]
        self.publisher.publish(cmd)
        
        # Update danger tracking
        if self.get_danger_factor() > 0.5:
            self.episode_danger_time += dt
        
        # Store experience
        if self.prev_state is not None:
            reward = self.calculate_reward()
            self.episode_reward += reward
            done = self.is_episode_done()
            
            self.replay_buffer.push(
                self.prev_state,
                [self.prev_action],
                reward,
                state,
                done
            )
            
            # Update networks (100Hz için düzeltme)
            if self.step_count % 2 == 0:  # Her 2 adımda bir update
                self.update_networks()
            
            if done:
                self.on_episode_end()
                return
        
        # Update state
        self.prev_state = state
        self.prev_action = action_normalized
        self.episode_steps += 1
        self.step_count += 1
        
        # Exploration decay
        if self.training_active:
            self.exploration_noise = max(self.min_exploration, 
                                       self.exploration_noise * self.exploration_decay)

def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveSwingUpController()
    
    print("\n=== ULTRA SAFE Adaptive Swing-Up Controller ===")
    print("ENTER: Eğitimi başlat/durdur")
    print("r: Cart'ı merkeze döndür") 
    print("s: Modeli kaydet")
    print("l: En iyi modeli yükle")
    print("q: Çıkış")
    print("=============================================\n")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_model('final')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()