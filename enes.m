%% CartPole Real-Time PID Controller - MATLAB ROS 2 Version with System ID
% Bu script Python PID kontrolcüsünün MATLAB versiyonu - Modern ROS 2 uyumlu + System Identification

clear all; close all; clc;

%% System Identification Mode Selection
fprintf('=== SISTEM TANIMA VE KONTROL MODLARı ===\n');
fprintf('1 - PID Kontrol (Normal)\n');
fprintf('2 - Sistem Tanıma (Step Response)\n');
fprintf('3 - Sistem Tanıma (Chirp Signal)\n');
fprintf('4 - Sistem Tanıma (PRBS Signal)\n');
fprintf('5 - Transfer Function Validation\n');
operation_mode = input('Mod seçiniz (1-5): ');

if isempty(operation_mode) || operation_mode < 1 || operation_mode > 5
    operation_mode = 1;
    fprintf('Varsayılan mod: PID Kontrol\n');
end

%% State değişkenleri
% Gerekli global değişkenlerin tanımı
global time_data pole_angle_data cart_position_data effort_data error_data sample_count
global test_input_data test_output_data test_time_data system_tf control_freq

% Başlangıçta boş veri
time_data = [];
pole_angle_data = [];
cart_position_data = [];
effort_data = [];
error_data = [];
sample_count = 0;

test_input_data = [];
test_output_data = [];
test_time_data = [];
system_tf = [];
prbs_state = 1; % PRBS state variable

% Diğer state değişkenleri
current_pole_angle = [];
current_cart_position = [];
current_cart_velocity = 0;
current_pole_velocity = 0;

pole_integral = 0.0;
pole_last_error = 0.0;
cart_integral = 0.0;
cart_last_error = 0.0;

% Önceki değerler (velocity hesaplama için)
prev_cart_pos = [];
prev_pole_angle = [];
prev_time = [];

%% Test Signal Parameters (System ID için)
test_duration = 20; % saniye (daha kısa)
test_amplitude = 30; % effort amplitude (daha güçlü: -30 to +30)
chirp_freq_range = [0.1, 10]; % Hz
step_amplitude = 25; % step amplitude (daha güçlü)
prbs_amplitude = 20;
test_signal = [];
prbs_period = 0.1; % 100ms period

% Test signal functions tanımla
switch operation_mode
    case 2 % Step Response
        fprintf('Step Response Test başlıyor...\n');
        test_signal = @(t) step_amplitude * (t > 0.5); % 0.5 saniye sonra step (daha erken)
        
    case 3 % Chirp Signal
        fprintf('Chirp Signal Test başlıyor...\n');
        fprintf('Frekans aralığı: %.1f - %.1f Hz\n', chirp_freq_range(1), chirp_freq_range(2));
        test_signal = @(t) test_amplitude * chirp(t, chirp_freq_range(1), test_duration, chirp_freq_range(2));
        
    case 4 % PRBS Signal
        fprintf('PRBS Signal Test başlıyor...\n');
        test_signal = @(t) prbs_amplitude * (2*round(rand(1)) - 1) * (mod(floor(t/prbs_period), 2) == 0);
        
    case 5 % Validation
        fprintf('Transfer Function Validation modu\n');
        % Load previously identified transfer function
        tf_file = '/home/enesb/fuzzy_ws/logs/identified_tf.mat';
        if exist(tf_file, 'file')
            load(tf_file, 'system_tf');
            fprintf('Transfer function yüklendi\n');
        else
            fprintf('Transfer function bulunamadı! Önce sistem tanıma yapın.\n');
            return;
        end
end

%% ROS 2 Node Initialization
fprintf('CartPole MATLAB PID Kontrolcüsü - ROS 2\n');
fprintf('======================================\n\n');

try
    % ROS 2 node oluştur
    node = ros2node("matlab_pid_controller");
    fprintf('ROS 2 node oluşturuldu: %s\n', node.Name);
    
    % Network setup için bekle
    pause(2);
    
catch ME
    fprintf('ROS 2 node oluşturma hatası: %s\n', ME.message);
    return;
end

%% PID Parametreleri
kp_pole = 1500.0;
ki_pole = 0.0;
kd_pole = 100.0;
desired_pole_angle = -pi;

kp_cart = 300.0;
ki_cart = 0.0;
kd_cart = 0.0;
desired_cart_position = 0.0;

fprintf('PID Parametreleri:\n');
fprintf('Pole: Kp=%.1f, Ki=%.1f, Kd=%.1f\n', kp_pole, ki_pole, kd_pole);
fprintf('Cart: Kp=%.1f, Ki=%.1f, Kd=%.1f\n\n', kp_cart, ki_cart, kd_cart);

%% ROS 2 Publishers ve Subscribers
try
    % Publisher oluştur
    cmd_pub = ros2publisher(node, "/effort_controllers/commands", "std_msgs/Float64MultiArray");
    
    % Subscriber oluştur
    js_sub = ros2subscriber(node, "/joint_states", "sensor_msgs/JointState");
    
    fprintf('ROS 2 publisher ve subscriber oluşturuldu\n');
    
    % Topic'leri kontrol et
    fprintf('\n=== ROS 2 TOPIC KONTROLÜ ===\n');
    topics = ros2("topic", "list");
    fprintf('Mevcut topic''ler:\n');
    for i = 1:length(topics)
        topic_name = char(topics{i});
        if contains(topic_name, 'effort') || contains(topic_name, 'joint') || contains(topic_name, 'command')
            fprintf('  %s\n', topic_name);
        end
    end
    
    % Publisher bilgilerini kontrol et
    fprintf('\nPublisher bilgileri:\n');
    fprintf('Topic: %s\n', cmd_pub.TopicName);
    fprintf('Message type: %s\n', cmd_pub.MessageType);
    
catch ME
    fprintf('ROS 2 interface oluşturma hatası: %s\n', ME.message);
    return;
end

%% Joint indekslerini bul
fprintf('Joint indeksleri aranıyor...\n');
joint_msg = [];
cart_idx = -1;
pole_idx = -1;

% Joint states mesajını bekle
timeout = 10; % 10 saniye timeout
start_time = tic;

while isempty(joint_msg) && toc(start_time) < timeout
    try
        joint_msg = receive(js_sub, 1); % 1 saniye timeout
        
        % Joint isimlerini kontrol et
        for i = 1:length(joint_msg.name)
            joint_name = char(joint_msg.name{i});
            fprintf('Joint %d: %s\n', i, joint_name);
            
            if contains(joint_name, 'slider_to_cart')
                cart_idx = i;
                fprintf('Cart joint bulundu: %s (indeks: %d)\n', joint_name, i);
            elseif contains(joint_name, 'cart_to_pole')
                pole_idx = i;
                fprintf('Pole joint bulundu: %s (indeks: %d)\n', joint_name, i);
            end
        end
        
    catch
        fprintf('Joint states mesajı bekleniyor...\n');
        pause(0.5);
    end
end

% Joint validation
if cart_idx > 0 && pole_idx > 0
    fprintf('Tüm joint''ler başarıyla bulundu!\n');
    fprintf('Cart indeks: %d, Pole indeks: %d\n', cart_idx, pole_idx);
else
    fprintf('HATA: Gerekli joint''ler bulunamadı!\n');
    return;
end

%% Timing ve kontrol parametreleri
control_freq = 200; % Hz
dt = 1/control_freq;
loop_timer = tic;

% Veri kaydetme arrays'leri initialize et
max_samples = 10000;
time_data = zeros(max_samples, 1);
pole_angle_data = zeros(max_samples, 1);
cart_position_data = zeros(max_samples, 1);
effort_data = zeros(max_samples, 1);
error_data = zeros(max_samples, 1);
setpoint_data = zeros(max_samples, 1);

%% Grafik hazırlama
if operation_mode == 1
    fig = figure('Position', [100 100 1200 800], 'Name', 'MATLAB PID Controller');
    % PID kontrol grafikleri
    subplot(2,2,1);
    h_angle = animatedline('Color', 'b', 'LineWidth', 1.5);
    hold on;
    h_setpoint = animatedline('Color', 'r', 'LineStyle', '--', 'LineWidth', 1);
    xlabel('Zaman (s)'); ylabel('Açı (rad)'); title('Pole Açısı vs Hedef');
    legend('Gerçek', 'Hedef', 'Location', 'best'); grid on;

    subplot(2,2,2);
    h_cart = animatedline('Color', 'g', 'LineWidth', 1.5);
    xlabel('Zaman (s)'); ylabel('Pozisyon (m)'); title('Cart Pozisyonu'); grid on;

    subplot(2,2,3);
    h_effort = animatedline('Color', 'm', 'LineWidth', 1.5);
    xlabel('Zaman (s)'); ylabel('Kontrol Kuvveti'); title('Kontrol Sinyali'); grid on;

    subplot(2,2,4);
    h_error = animatedline('Color', 'r', 'LineWidth', 1.5);
    xlabel('Zaman (s)'); ylabel('Hata (rad)'); title('Kontrol Hatası'); grid on;
else
    % System ID grafikleri
    fig = figure('Position', [100 100 1400 900], 'Name', 'System Identification');
    
    subplot(3,2,1);
    h_input = animatedline('Color', 'r', 'LineWidth', 1.5);
    xlabel('Zaman (s)'); ylabel('Input Effort'); title('Test Input Signal'); grid on;
    
    subplot(3,2,2);
    h_output = animatedline('Color', 'b', 'LineWidth', 1.5);
    xlabel('Zaman (s)'); ylabel('Pole Angle (rad)'); title('System Output (Pole Angle)'); grid on;
    
    subplot(3,2,3);
    h_io = animatedline('Color', 'g', 'LineWidth', 1.5);
    xlabel('Input Effort'); ylabel('Pole Angle (rad)'); title('Input-Output Relationship'); grid on;
    
    subplot(3,2,4:6);
    text(0.5, 0.5, 'System Analysis will appear here after data collection', ...
         'HorizontalAlignment', 'center', 'FontSize', 14);
    title('Analysis Results'); axis off;
end

%% İlk komut gönder (sıfır)
fprintf('\n=== İLK KOMUT GÖNDERİLİYOR ===\n');
send_zero_command(cmd_pub);

% Test için güçlü bir PWM gönder
fprintf('Test PWM gönderiliyor...\n');
test_cmd_msg = ros2message("std_msgs/Float64MultiArray");
test_cmd_msg.data = [50.0];  % +50 PWM
send(cmd_pub, test_cmd_msg);
pause(1);

test_cmd_msg.data = [-50.0]; % -50 PWM  
send(cmd_pub, test_cmd_msg);
pause(1);

test_cmd_msg.data = [0.0];   % Sıfırla
send(cmd_pub, test_cmd_msg);
fprintf('Test PWM tamamlandı\n');

%% Ana kontrol döngüsü
fprintf('\nKontrol döngüsü başlıyor... (Ctrl+C ile durdurun)\n');
fprintf('Zaman\t\tPole Açı\tCart Pos\tHata\t\tEffort\n');
fprintf('------------------------------------------------------------\n');

start_time = tic;
last_print_time = 0;
last_plot_time = 0;

try
    while ishandle(fig)
        current_time = toc(start_time);
        
        %% Sensor verilerini oku
        try
            latest_msg = receive(js_sub, 0.1);
            
            if ~isempty(latest_msg)
                joint_msg = latest_msg;
                
                if length(joint_msg.position) >= max(cart_idx, pole_idx)
                    new_cart_pos = joint_msg.position(cart_idx);
                    new_pole_angle = joint_msg.position(pole_idx);
                    
                    % Velocity hesaplama
                    if ~isempty(prev_cart_pos) && ~isempty(prev_time)
                        dt_sensor = current_time - prev_time;
                        if dt_sensor > 0
                            current_cart_velocity = (new_cart_pos - prev_cart_pos) / dt_sensor;
                            current_pole_velocity = (new_pole_angle - prev_pole_angle) / dt_sensor;
                        end
                    end
                    
                    current_cart_position = new_cart_pos;
                    current_pole_angle = new_pole_angle;
                    prev_cart_pos = new_cart_pos;
                    prev_pole_angle = new_pole_angle;
                    prev_time = current_time;
                end
            end
            
        catch ME
            if mod(sample_count, 100) == 0
                fprintf('Sensor okuma uyarısı: %s\n', ME.message);
            end
            send_zero_command(cmd_pub);
            continue;
        end
        
        %% Test Signal Generation veya PID Control
        if operation_mode > 1 && operation_mode <= 4
            % System ID modları
            if current_time <= test_duration
                if ~isempty(test_signal)
                    test_effort = test_signal(current_time);
                else
                    test_effort = 0;
                end
                
                % PRBS için özel random update
                if operation_mode == 4 && mod(sample_count, round(prbs_period*control_freq)) == 0
                    prbs_state = -prbs_state;
                    test_effort = prbs_amplitude * prbs_state;
                end
                
                % Effort saturation
                test_effort = max(min(test_effort, 100), -100);
                
                % Debug: Sürekli PWM bilgisi
                if mod(sample_count, 50) == 0  % Her 50 sample'da bir (daha sık)
                    fprintf('[DEBUG] Time: %.2f, PWM: %.2f, Pole: %.4f, Cart: %.4f\n', ...
                            current_time, test_effort, current_pole_angle, current_cart_position);
                end
                
                % Test sinyali başladı mı kontrolü
                if current_time > 0.5 && abs(test_effort) < 0.1
                    fprintf('[WARNING] Step signal should be active but effort is: %.2f\n', test_effort);
                end
                
                send_command(cmd_pub, test_effort);
                
                if ~isempty(current_pole_angle)
                    test_input_data(end+1) = test_effort;
                    test_output_data(end+1) = current_pole_angle;
                    test_time_data(end+1) = current_time;
                end
                
                effort = test_effort;
                
            else
                % Test tamamlandı
                send_zero_command(cmd_pub);
                if ~isempty(test_input_data) && length(test_input_data) > 100
                    fprintf('\nSistem tanıma verisi toplanıyor...\n');
                    calculate_transfer_function();
                end
                break;
            end
            
        else
            % Normal PID Control
            if ~isempty(current_pole_angle)
                error = desired_pole_angle - current_pole_angle;
                pole_integral = pole_integral + error * dt;
                
                if pole_last_error ~= 0 || error ~= 0
                    derivative = (error - pole_last_error) / dt;
                else
                    derivative = 0;
                end
                
                effort = -(kp_pole * error + ki_pole * pole_integral + kd_pole * derivative);
                pole_last_error = error;
                
                effort = max(min(effort, 100.0), -100.0);
                send_command(cmd_pub, effort);
                
            else
                effort = 0;
                error = 0;
                send_zero_command(cmd_pub);
            end
        end
        
        %% Veri kaydetme
        sample_count = sample_count + 1;
        if sample_count <= max_samples && ~isempty(current_pole_angle)
            time_data(sample_count) = current_time;
            pole_angle_data(sample_count) = current_pole_angle;
            cart_position_data(sample_count) = current_cart_position;
            effort_data(sample_count) = effort;
            
            if operation_mode == 1
                error_data(sample_count) = error;
                setpoint_data(sample_count) = desired_pole_angle;
            end
        end
        
        %% Periyodik analizler (sadece PID modunda)
        if operation_mode == 1
            % Her 1000 sample'da performans analizi
            if mod(sample_count, 1000) == 0
                generate_performance_report();
            end
            
            % Her 500 sample'da adaptive control
            if mod(sample_count, 500) == 0
                adaptive_pid_controller();
            end
            
            % Her 200 sample'da optimization
            if mod(sample_count, 200) == 0
                realtime_optimization();
            end
        end
        
        %% Konsol çıktısı
        if current_time - last_print_time > 0.5
            if ~isempty(current_pole_angle)
                if operation_mode == 1
                    fprintf('%.3f\t\t%.3f\t\t%.3f\t\t%.3f\t\t%.3f\n', ...
                            current_time, current_pole_angle, current_cart_position, error, effort);
                else
                    fprintf('%.3f\t\tInput: %.2f\t\tOutput: %.4f\n', ...
                            current_time, effort, current_pole_angle);
                end
            end
            last_print_time = current_time;
        end
        
        %% Grafik güncelleme
        if current_time - last_plot_time > 0.1 && ~isempty(current_pole_angle)
            if operation_mode == 1
                addpoints(h_angle, current_time, current_pole_angle);
                addpoints(h_setpoint, current_time, desired_pole_angle);
                addpoints(h_cart, current_time, current_cart_position);
                addpoints(h_effort, current_time, effort);
                addpoints(h_error, current_time, error);
            else
                addpoints(h_input, current_time, effort);
                addpoints(h_output, current_time, current_pole_angle);
                if ~isempty(test_input_data)
                    addpoints(h_io, effort, current_pole_angle);
                end
            end
            
            % Axis limits güncelle
            window_time = 10;
            if operation_mode == 1
                for i = 1:4
                    subplot(2, 2, i);
                    xlim([max(0, current_time-window_time), current_time+1]);
                end
            else
                for i = 1:3
                    subplot(3, 2, i);
                    xlim([max(0, current_time-window_time), current_time+1]);
                end
            end
            
            drawnow limitrate;
            last_plot_time = current_time;
        end
        
        %% Timing kontrolü
        elapsed = toc(loop_timer);
        if elapsed < dt
            pause(dt - elapsed);
        end
        loop_timer = tic;
    end
    
catch ME
    fprintf('\nKontrol döngüsü sonlandırıldı: %s\n', ME.message);
end

%% Temizlik ve final analiz
send_zero_command(cmd_pub);

if sample_count > 10
    fprintf('\n=== FINAL ANALİZ ===\n');
    if operation_mode == 1
        generate_performance_report();
    end
    
    % Veri kaydetme
    output_dir = '/home/enesb/fuzzy_ws/logs';
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end
    
    actual_samples = min(sample_count, max_samples);
    time_final = time_data(1:actual_samples);
    pole_angle_final = pole_angle_data(1:actual_samples);
    
    % CSV kaydetme
    csv_filename = fullfile(output_dir, 'matlab_ros2_data.csv');
    data_table = table(time_final, pole_angle_final, cart_position_data(1:actual_samples), ...
                       effort_data(1:actual_samples), ...
                       'VariableNames', {'time', 'pole_angle', 'cart_position', 'effort'});
    writetable(data_table, csv_filename);
    fprintf('Veriler kaydedildi: %s\n', csv_filename);
end

fprintf('\n=== MATLAB ROS 2 KONTROLCÜSÜ TAMAMLANDI ===\n');

%% Helper Functions - Script sonunda tanımla

function send_command(cmd_pub, effort_value)
    try
        % Float64MultiArray mesajı oluştur
        cmd_msg = ros2message("std_msgs/Float64MultiArray");
        cmd_msg.data = [double(effort_value)];
        
        % Mesajı gönder
        send(cmd_pub, cmd_msg);
        
        % Her komut için debug (geçici olarak)
        if abs(effort_value) > 0.1
            fprintf('[SEND] PWM: %.2f gönderildi\n', effort_value);
        end
        
        % Topic'e gerçekten publish oldu mu kontrol et
        info = ros2("topic", "info", cmd_pub.TopicName);
        if isempty(info)
            fprintf('[ERROR] Topic publish edilemiyor: %s\n', cmd_pub.TopicName);
        end
        
    catch ME
        fprintf('[ERROR] Komut gönderme hatası: %s\n', ME.message);
        
        % Alternatif topic dene
        try
            fprintf('[INFO] Alternatif topic deneniyor...\n');
            alt_pub = ros2publisher(node, "/effort_controller/commands", "std_msgs/Float64MultiArray");
            alt_msg = ros2message("std_msgs/Float64MultiArray");
            alt_msg.data = [double(effort_value)];
            send(alt_pub, alt_msg);
            fprintf('[SUCCESS] Alternatif topic ile gönderildi\n');
        catch
            fprintf('[ERROR] Alternatif topic de çalışmıyor\n');
        end
    end
end

function send_zero_command(cmd_pub)
    send_command(cmd_pub, 0.0);
end

function calculate_ideal_response(tf_sys)
    fprintf('Transfer Function: ');
    display(tf_sys);
    
    % System properties
    pole_vals = pole(tf_sys);
    zero_vals = zero(tf_sys);
    
    fprintf('Poles: ');
    disp(pole_vals');
    fprintf('Zeros: ');
    disp(zero_vals');
    
    % Stability analysis
    if all(real(pole_vals) < 0)
        fprintf('Sistem KARALI\n');
    else
        fprintf('Sistem KARARLI DEĞİL!\n');
    end
    
    % Step response characteristics
    try
        step_info = stepinfo(tf_sys);
        fprintf('\nStep Response Özellikleri:\n');
        fields = fieldnames(step_info);
        for i = 1:length(fields)
            fprintf('%s: %.4f\n', fields{i}, step_info.(fields{i}));
        end
    catch
        fprintf('Step response analizi yapılamadı\n');
    end
    
    % Bandwidth and margins
    try
        [Gm, Pm, Wgm, Wpm] = margin(tf_sys);
        fprintf('\nFrequency Domain Özellikler:\n');
        fprintf('Gain Margin: %.2f dB\n', 20*log10(Gm));
        fprintf('Phase Margin: %.2f deg\n', Pm);
        fprintf('Gain Crossover: %.2f rad/s\n', Wgm);
        fprintf('Phase Crossover: %.2f rad/s\n', Wpm);
    catch
        fprintf('Frequency domain analizi yapılamadı\n');
    end
end

function simple_tf_estimation()
    global test_input_data test_output_data test_time_data
    
    % Simple first-order approximation
    fprintf('Basit birinci derece model tahmini...\n');
    
    if length(test_input_data) < 20
        fprintf('Yeterli veri yok basit tahmin için\n');
        return;
    end
    
    % Find steady state values
    final_input = mean(test_input_data(end-10:end));
    final_output = mean(test_output_data(end-10:end));
    
    if abs(final_input) > 0.1
        dc_gain = final_output / final_input;
        fprintf('DC Gain: %.4f\n', dc_gain);
        
        % Simple first order: K/(s+a)
        % Estimate time constant from 63% rise time
        target_output = 0.632 * final_output;
        
        % Find time to reach 63% of final value
        time_constant = [];
        for i = 1:length(test_output_data)
            if abs(test_output_data(i) - target_output) < abs(final_output)*0.1
                time_constant = test_time_data(i);
                break;
            end
        end
        
        if ~isempty(time_constant)
            fprintf('Estimated Time Constant: %.2f s\n', time_constant);
            
            % Create simple transfer function
            K = dc_gain;
            tau = time_constant;
            if tau > 0
                simple_tf = tf(K, [tau 1]);
                fprintf('Estimated Transfer Function:\n');
                display(simple_tf);
            end
        end
    end
end

%% 1. SİSTEM TANIMLAMA
function calculate_transfer_function()
    % Global variables declaration for function access
    global test_input_data test_output_data test_time_data system_tf control_freq
    
    fprintf('\n=== TRANSFER FUNCTION HESAPLAMASI ===\n');
    
    % Veri kontrolü
    if isempty(test_input_data) || length(test_input_data) < 100
        fprintf('Yetersiz veri: %d samples (minimum 100 gerekli)\n', length(test_input_data));
        return;
    end
    
    try
        % Veriyi düzenle
        input_data = test_input_data(:);
        output_data = test_output_data(:);
        time_data = test_time_data(:);
        
        fprintf('Toplam veri sayısı: %d samples\n', length(input_data));
        fprintf('Veri aralığı: %.2f - %.2f saniye\n', min(time_data), max(time_data));
        
        % Sampling time
        Ts = 1/control_freq;
        
        % System Identification Toolbox kullan
        if exist('iddata', 'file')
            % Create iddata object
            data = iddata(output_data, input_data, Ts);
            
            % Different model orders to try
            orders = [1 1; 2 1; 2 2; 3 2; 3 3];
            best_fit = 0;
            best_tf = [];
            
            fprintf('Farklı model mertebeleri deneniyor...\n');
            
            for i = 1:size(orders, 1)
                try
                    na = orders(i, 1); % denominator order
                    nb = orders(i, 2); % numerator order
                    
                    % ARX model estimate
                    model = arx(data, [na nb 1]);
                    
                    % Convert to transfer function
                    tf_model = tf(model);
                    
                    % Validate model
                    [y_pred, fit_percent] = compare(data, model);
                    
                    fprintf('Model [%d,%d]: Fit = %.2f%%\n', na, nb, fit_percent);
                    
                    if fit_percent > best_fit
                        best_fit = fit_percent;
                        best_tf = tf_model;
                        system_tf = tf_model;
                    end
                    
                catch ME
                    fprintf('Model [%d,%d] hatası: %s\n', na, nb, ME.message);
                end
            end
            
            if ~isempty(best_tf)
                fprintf('\nEn iyi model (Fit: %.2f%%):\n', best_fit);
                display(best_tf);
                
                % Frequency response analysis
                figure('Name', 'System Analysis');
                
                subplot(2,2,1);
                step(best_tf);
                title('Step Response');
                grid on;
                
                subplot(2,2,2);
                bode(best_tf);
                title('Bode Plot');
                grid on;
                
                subplot(2,2,3);
                impulse(best_tf);
                title('Impulse Response');
                grid on;
                
                subplot(2,2,4);
                pzmap(best_tf);
                title('Pole-Zero Map');
                grid on;
                
                % Save transfer function
                output_dir = '/home/enesb/fuzzy_ws/logs';
                if ~exist(output_dir, 'dir')
                    mkdir(output_dir);
                end
                
                save(fullfile(output_dir, 'identified_tf.mat'), 'system_tf', 'best_tf', 'best_fit');
                
                % Ideal response calculation
                fprintf('\n=== İDEAL CEVAP ANALİZİ ===\n');
                calculate_ideal_response(best_tf);
                
                % Gelişmiş analizi çağır (BURADA ÇAĞIR)
                fprintf('\n=== Gelişmiş sistem analizi başlıyor ===\n');
                advanced_system_analysis();
                
            else
                fprintf('Transfer function hesaplanamadı!\n');
            end
            
        else
            fprintf('System Identification Toolbox bulunamadı!\n');
            fprintf('Basit yöntemle transfer function tahmini yapılıyor...\n');
            simple_tf_estimation();
            
            % Gelişmiş analizi yine de çağır
            advanced_system_analysis();
        end
        
    catch ME
        fprintf('Transfer function hesaplama hatası: %s\n', ME.message);
    end
end

%% GELİŞMİŞ SİSTEM ANALİZİ
function advanced_system_analysis()
    global test_input_data test_output_data test_time_data
    
    fprintf('\n=== GELİŞMİŞ SİSTEM ANALİZİ ===\n');
    
    % Veri kontrolü ve debug bilgisi
    fprintf('test_input_data length: %d\n', length(test_input_data));
    fprintf('test_output_data length: %d\n', length(test_output_data));
    fprintf('test_time_data length: %d\n', length(test_time_data));
    
    if isempty(test_input_data) || length(test_input_data) < 100
        fprintf('Yeterli veri yok! Mevcut: %d samples (minimum 100 gerekli)\n', length(test_input_data));
        return;
    end
    
    % Veriyi düzenle
    t = test_time_data(:);
    u = test_input_data(:);    % PWM input (-100 to 100)
    y = test_output_data(:);   % Pole angle output
    
    % Veri istatistikleri
    fprintf('Veri istatistikleri:\n');
    fprintf('  Input range: %.2f to %.2f\n', min(u), max(u));
    fprintf('  Output range: %.4f to %.4f rad\n', min(y), max(y));
    fprintf('  Time range: %.2f to %.2f s\n', min(t), max(t));
    
    % Sampling frequency
    if length(t) > 1
        Ts = mean(diff(t));
        fs = 1/Ts;
        fprintf('  Sampling frequency: %.1f Hz\n', fs);
    else
        fprintf('Tek veri noktası - analiz yapılamıyor\n');
        return;
    end
    
    %% A) Non-parametric Analysis
    figure('Position', [100 100 1400 1000], 'Name', 'Gelişmiş Sistem Analizi');
    
    % Time domain analysis
    subplot(3,3,1);
    plot(t, u, 'r-', 'LineWidth', 1.2);
    xlabel('Zaman (s)'); ylabel('PWM Input');
    title('Input Signal');
    grid on;
    
    subplot(3,3,2);
    plot(t, y, 'b-', 'LineWidth', 1.2);
    xlabel('Zaman (s)'); ylabel('Pole Angle (rad)');
    title('System Response');
    grid on;
    
    % Input-Output relationship
    subplot(3,3,3);
    scatter(u, y, 20, 'filled', 'alpha', 0.6);
    xlabel('PWM Input'); ylabel('Pole Angle (rad)');
    title('Input-Output Scatter');
    grid on;
    
    % Fit linear relationship
    p = polyfit(u, y, 1);
    hold on;
    plot(u, polyval(p, u), 'r-', 'LineWidth', 2);
    fprintf('Lineer ilişki: y = %.4f * u + %.4f\n', p(1), p(2));
    fprintf('PWM/Açı oranı: %.4f rad/PWM\n', p(1));
    
    %% B) Frequency Domain Analysis
    if length(u) > 256
        % FFT Analysis
        N = length(u);
        f = (0:N-1) * fs / N;
        f = f(1:floor(N/2));
        
        U_fft = fft(u);
        Y_fft = fft(y);
        
        U_mag = abs(U_fft(1:floor(N/2)));
        Y_mag = abs(Y_fft(1:floor(N/2)));
        
        subplot(3,3,4);
        semilogx(f, 20*log10(U_mag + eps));
        xlabel('Frequency (Hz)'); ylabel('Magnitude (dB)');
        title('Input Spectrum');
        grid on;
        
        subplot(3,3,5);
        semilogx(f, 20*log10(Y_mag + eps));
        xlabel('Frequency (Hz)'); ylabel('Magnitude (dB)');
        title('Output Spectrum');
        grid on;
        
        % Frequency Response Estimation
        H_est = Y_fft ./ (U_fft + eps);
        H_mag = abs(H_est(1:floor(N/2)));
        H_phase = angle(H_est(1:floor(N/2))) * 180/pi;
        
        subplot(3,3,6);
        semilogx(f, 20*log10(H_mag + eps));
        xlabel('Frequency (Hz)'); ylabel('Magnitude (dB)');
        title('Estimated Frequency Response');
        grid on;
    end
    
    %% C) System Properties
    % Step response characteristics (if step test)
    if any(abs(diff(u)) > 10) % Step detected
        step_idx = find(abs(diff(u)) > 10, 1);
        if ~isempty(step_idx)
            step_response = y(step_idx:end) - y(step_idx);
            step_time = t(step_idx:end) - t(step_idx);
            step_input = u(step_idx+1);
            
            % Normalize response
            steady_state = mean(step_response(end-10:end));
            normalized_response = step_response / steady_state;
            
            % Find rise time (10% to 90%)
            idx_10 = find(normalized_response >= 0.1, 1);
            idx_90 = find(normalized_response >= 0.9, 1);
            
            if ~isempty(idx_10) && ~isempty(idx_90)
                rise_time = step_time(idx_90) - step_time(idx_10);
                fprintf('Rise Time (10%%-90%%): %.3f s\n', rise_time);
            end
            
            % Settling time (2% criterion)
            settling_idx = find(abs(normalized_response - 1) <= 0.02);
            if ~isempty(settling_idx)
                settling_time = step_time(settling_idx(1));
                fprintf('Settling Time (2%%): %.3f s\n', settling_time);
            end
            
            % DC Gain
            dc_gain = steady_state / step_input;
            fprintf('DC Gain: %.6f rad/PWM\n', dc_gain);
            
            subplot(3,3,7);
            plot(step_time, normalized_response, 'b-', 'LineWidth', 1.5);
            xlabel('Zaman (s)'); ylabel('Normalized Response');
            title('Step Response Analysis');
            grid on;
        end
    end
    
    %% D) Linearity Analysis
    % Check for nonlinearities
    residuals = y - polyval(p, u);
    
    subplot(3,3,8);
    plot(u, residuals, 'ro', 'MarkerSize', 4);
    xlabel('PWM Input'); ylabel('Residuals');
    title('Linearity Check');
    grid on;
    
    rms_residual = sqrt(mean(residuals.^2));
    fprintf('RMS Residual: %.6f rad\n', rms_residual);
    
    % Correlation analysis
    [correlation, lags] = xcorr(u, y, 'normalized');
    [~, max_idx] = max(abs(correlation));
    delay_samples = lags(max_idx);
    delay_time = delay_samples * Ts;
    
    subplot(3,3,9);
    plot(lags * Ts, correlation, 'g-', 'LineWidth', 1.2);
    xlabel('Lag (s)'); ylabel('Correlation');
    title('Cross-Correlation');
    grid on;
    
    fprintf('System Delay: %.3f s (%.0f samples)\n', delay_time, delay_samples);
    
    %% E) Model Recommendations
    fprintf('\n=== MODEL ÖNERİLERİ ===\n');
    
    % Recommend model structure based on analysis
    if abs(delay_time) > 2*Ts
        fprintf('Sistem gecikmeli - Pure delay modeli ekle\n');
    end
    
    if rms_residual/std(y) > 0.1
        fprintf('Nonlinearite var - Hammerstein/Wiener modeli dene\n');
    end
    
    bandwidth_est = estimate_bandwidth(f, H_mag);
    fprintf('Tahmini bant genişliği: %.2f Hz\n', bandwidth_est);
    
    if bandwidth_est < 1
        fprintf('Düşük frekanslı sistem - 1. veya 2. derece model yeterli\n');
    elseif bandwidth_est < 5
        fprintf('Orta frekanslı sistem - 2. veya 3. derece model dene\n');
    else
        fprintf('Yüksek frekanslı sistem - Yüksek dereceli model gerekli\n');
    end
end

%% 2. ADAPTIVE PID CONTROLLER
function adaptive_pid_controller()
    % Online PID parameter adaptation
    
    persistent param_history error_history performance_metric
    
    if isempty(param_history)
        param_history = [];
        error_history = [];
        performance_metric = inf;
    end
    
    % Current performance metric (örnek: IAE - Integral Absolute Error)
    current_metric = calculate_performance_metric();
    
    % Parameter adaptation logic
    if current_metric < performance_metric * 0.95  % %5 improvement
        fprintf('PID parametreleri iyileşti - mevcut ayarları koru\n');
        performance_metric = current_metric;
    else
        fprintf('Performans kötüleşti - parametre ayarı gerekli\n');
        suggest_pid_tuning();
    end
end

function metric = calculate_performance_metric()
    global error_data sample_count
    
    if sample_count > 10
        recent_errors = error_data(max(1, sample_count-100):sample_count);
        metric = sum(abs(recent_errors)); % IAE
    else
        metric = inf;
    end
end

function suggest_pid_tuning()
    fprintf('\n=== PID AYAR ÖNERİLERİ ===\n');
    
    % Ziegler-Nichols based suggestions
    fprintf('Sistem davranışına göre öneriler:\n');
    fprintf('- Oscillasyon çok fazla ise: Kd artır, Kp azalt\n');
    fprintf('- Çok yavaş yanıt ise: Kp artır\n');
    fprintf('- Steady-state error var ise: Ki artır\n');
    fprintf('- Sistem unstable ise: Tüm gains azalt\n');
end

%% 3. PERFORMANS ANALİZİ VE RAPORLAMA
function generate_performance_report()
    global time_data pole_angle_data cart_position_data effort_data error_data sample_count
    
    if sample_count < 10
        return;
    end
    
    % Aktif veriyi al
    t = time_data(1:sample_count);
    angle = pole_angle_data(1:sample_count);
    position = cart_position_data(1:sample_count);
    effort = effort_data(1:sample_count);
    error = error_data(1:sample_count);
    
    fprintf('\n=== PERFORMANS RAPORU ===\n');
    
    %% Control Performance Metrics
    % Error metrics
    mae = mean(abs(error));
    rmse = sqrt(mean(error.^2));
    max_error = max(abs(error));
    steady_state_error = mean(abs(error(end-min(50,length(error)):end)));
    
    % Settling criteria
    settling_threshold = 0.02; % 2% of setpoint
    settled_indices = find(abs(error) <= settling_threshold);
    if ~isempty(settled_indices)
        settling_time = t(settled_indices(1));
    else
        settling_time = inf;
    end
    
    % Control effort metrics
    total_energy = sum(effort.^2) * mean(diff(t));
    max_effort = max(abs(effort));
    effort_variation = std(effort);
    
    % Stability metrics
    angle_stability = std(angle(end-min(100,length(angle)):end));
    position_stability = std(position(end-min(100,length(position)):end));
    
    fprintf('=== HATA METRİKLERİ ===\n');
    fprintf('MAE (Mean Absolute Error): %.4f rad\n', mae);
    fprintf('RMSE (Root Mean Square Error): %.4f rad\n', rmse);
    fprintf('Maksimum Hata: %.4f rad\n', max_error);
    fprintf('Steady-State Hata: %.4f rad\n', steady_state_error);
    fprintf('Settling Time: %.2f s\n', settling_time);
    
    fprintf('\n=== KONTROL METRİKLERİ ===\n');
    fprintf('Toplam Kontrol Enerjisi: %.2f\n', total_energy);
    fprintf('Maksimum PWM: %.0f\n', max_effort);
    fprintf('PWM Varyasyonu: %.2f\n', effort_variation);
    
    fprintf('\n=== STABİLİTE METRİKLERİ ===\n');
    fprintf('Açı Stabilitesi (std): %.4f rad\n', angle_stability);
    fprintf('Pozisyon Stabilitesi (std): %.4f m\n', position_stability);
    
    %% Performance Score
    % Composite performance score (0-100)
    error_score = max(0, 100 - mae*1000);
    effort_score = max(0, 100 - max_effort);
    stability_score = max(0, 100 - angle_stability*1000);
    
    overall_score = (error_score + effort_score + stability_score) / 3;
    
    fprintf('\n=== PERFORMANS SKORU ===\n');
    fprintf('Hata Skoru: %.1f/100\n', error_score);
    fprintf('Kontrol Skoru: %.1f/100\n', effort_score);
    fprintf('Stabilite Skoru: %.1f/100\n', stability_score);
    fprintf('GENEL SKOR: %.1f/100\n', overall_score);
    
    %% Recommendations
    fprintf('\n=== İYİLEŞTİRME ÖNERİLERİ ===\n');
    
    if mae > 0.1
        fprintf('- Hata çok yüksek: PID parametrelerini optimize et\n');
    end
    
    if max_effort > 80
        fprintf('- PWM saturation: Kontrol limitlerini gözden geçir\n');
    end
    
    if effort_variation > 20
        fprintf('- PWM çok değişken: Türev gain (Kd) ayarını kontrol et\n');
    end
    
    if settling_time > 5
        fprintf('- Yavaş yanıt: Proportional gain (Kp) artır\n');
    end
    
    if steady_state_error > 0.05
        fprintf('- Steady-state hata: Integral gain (Ki) ekle/artır\n');
    end
    
    %% Advanced Analysis Plots
    create_advanced_plots(t, angle, position, effort, error);
end

function create_advanced_plots(t, angle, position, effort, error)
    figure('Position', [200 100 1400 900], 'Name', 'Detaylı Performans Analizi');
    
    % Time domain performance
    subplot(3,3,1);
    plot(t, error, 'r-', 'LineWidth', 1.2);
    xlabel('Zaman (s)'); ylabel('Hata (rad)');
    title('Kontrol Hatası');
    grid on;
    
    % Error histogram
    subplot(3,3,2);
    histogram(error, 50, 'Normalization', 'probability');
    xlabel('Hata (rad)'); ylabel('Probability');
    title('Hata Dağılımı');
    grid on;
    
    % Control effort analysis
    subplot(3,3,3);
    plot(t, effort, 'm-', 'LineWidth', 1.2);
    xlabel('Zaman (s)'); ylabel('PWM');
    title('Kontrol Sinyali');
    grid on;
    ylim([-105 105]);
    
    % Phase plane (angle vs angular velocity)
    angle_vel = [0; diff(angle)./diff(t)];
    subplot(3,3,4);
    plot(angle, angle_vel, 'b-', 'LineWidth', 1);
    xlabel('Açı (rad)'); ylabel('Açısal Hız (rad/s)');
    title('Faz Düzlemi');
    grid on;
    
    % Control effort vs error relationship
    subplot(3,3,5);
    scatter(error, effort, 20, 'filled', 'alpha', 0.6);
    xlabel('Hata (rad)'); ylabel('PWM');
    title('Hata-Kontrol İlişkisi');
    grid on;
    
    % Frequency analysis of error
    if length(error) > 256
        N = length(error);
        fs = 1/mean(diff(t));
        f = (0:N-1) * fs / N;
        f = f(1:floor(N/2));
        
        Error_fft = fft(error);
        Error_mag = abs(Error_fft(1:floor(N/2)));
        
        subplot(3,3,6);
        semilogx(f, 20*log10(Error_mag + eps));
        xlabel('Frequency (Hz)'); ylabel('Magnitude (dB)');
        title('Hata Spektrumu');
        grid on;
    end
    
    % Moving statistics
    window_size = min(50, floor(length(t)/10));
    moving_mae = movmean(abs(error), window_size);
    moving_std = movstd(error, window_size);
    
    subplot(3,3,7);
    plot(t, moving_mae, 'r-', 'LineWidth', 1.5);
    hold on;
    plot(t, moving_std, 'b--', 'LineWidth', 1.5);
    xlabel('Zaman (s)'); ylabel('Error Metrics');
    title('Hareketli İstatistikler');
    legend('Moving MAE', 'Moving STD');
    grid on;
    
    % Control activity analysis
    control_activity = abs(diff(effort));
    subplot(3,3,8);
    plot(t(2:end), control_activity, 'g-', 'LineWidth', 1.2);
    xlabel('Zaman (s)'); ylabel('PWM Değişimi');
    title('Kontrol Aktivitesi');
    grid on;
    
    % Performance over time
    cumulative_error = cumsum(abs(error)) ./ (1:length(error))';
    subplot(3,3,9);
    plot(t, cumulative_error, 'k-', 'LineWidth', 1.5);
    xlabel('Zaman (s)'); ylabel('Kümülatif MAE');
    title('Zaman İçinde Performans');
    grid on;
end

%% 4. REAL-TIME OPTIMIZATION
function realtime_optimization()
    % Online parameter optimization using simple gradient descent
    
    persistent kp_history ki_history kd_history performance_history step_size
    
    if isempty(kp_history)
        kp_history = [];
        ki_history = [];
        kd_history = [];
        performance_history = [];
        step_size = 10; % Initial step size
    end
    
    current_performance = calculate_performance_metric();
    
    % Simple parameter perturbation for optimization
    if length(performance_history) > 2
        if current_performance < min(performance_history)
            fprintf('Performans iyileşiyor - parametre yönünü koru\n');
        else
            fprintf('Performans kötüleşiyor - parametre yönünü değiştir\n');
            step_size = step_size * 0.8; % Reduce step size
        end
    end
    
    performance_history(end+1) = current_performance;
end

%% 5. HELPER FUNCTIONS
function bw = estimate_bandwidth(freq, magnitude)
    % -3dB bandwidth estimation
    if length(magnitude) < 10
        bw = 1; % Default
        return;
    end
    
    max_mag = max(magnitude);
    cutoff_mag = max_mag / sqrt(2); % -3dB point
    
    cutoff_idx = find(magnitude >= cutoff_mag, 1, 'last');
    if ~isempty(cutoff_idx) && cutoff_idx < length(freq)
        bw = freq(cutoff_idx);
    else
        bw = freq(end);
    end
end