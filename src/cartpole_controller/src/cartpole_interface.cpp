// src/cartpole_interface.cpp
#include "cartpole_controller/cartpole_interface.hpp"
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string>
#include <sstream>
#include <cstring>

const float PoleEncoderRes =4096;
const float CartEncoderRes = 8417;
const float CartReductionRatio = 131;
const float CartShaftRadius = 0.033/2;
const float CartLength = 0.47;

const float PoleRadConst = 2*M_PI/PoleEncoderRes;
const float CartPosConst = CartLength/CartEncoderRes;

namespace cartpole_controller {

CartpoleHardwareInterface::CartpoleHardwareInterface()
: connection_handle_(-1), read_buffer_("") {
}

hardware_interface::CallbackReturn CartpoleHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  device_path_ = info_.hardware_parameters["device_path"];
  baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);

  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> CartpoleHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, "position", &hw_positions_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, "velocity", &hw_velocities_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, "effort", &hw_efforts_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> CartpoleHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, "effort", &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn CartpoleHardwareInterface::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  // Mark unused parameter to avoid warnings
  (void)previous_state;
  
  RCLCPP_INFO(rclcpp::get_logger("CartpoleHardwareInterface"), "Starting controller");
  
  // Connect to hardware
  if (!connect_to_hardware()) {
    RCLCPP_ERROR(rclcpp::get_logger("CartpoleHardwareInterface"), "Failed to connect to hardware");
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  // Clear buffer
  read_buffer_.clear();
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CartpoleHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  // Mark unused parameter to avoid warnings
  (void)previous_state;
  
  RCLCPP_INFO(rclcpp::get_logger("CartpoleHardwareInterface"), "Stopping controller");
  
  // Send stop commands (e.g., stop the motor)
  for (size_t i = 0; i < hw_commands_.size(); i++) {
    hw_commands_[i] = 0.0;
  }
  
  // Close hardware connection
  if (connection_handle_ >= 0) {
    close(connection_handle_);
    connection_handle_ = -1;
  }
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type CartpoleHardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Mark unused parameters to avoid warnings
  (void)time;
  (void)period;
  
  if (connection_handle_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("CartpoleHardwareInterface"), "Cannot read: device not connected");
    return hardware_interface::return_type::ERROR;
  }
  
  // Read new data and add to buffer
  char temp_buffer[256];
  int bytes_read = ::read(connection_handle_, temp_buffer, sizeof(temp_buffer)-1);
  
  if (bytes_read > 0) {
    temp_buffer[bytes_read] = '\0';
    read_buffer_ += std::string(temp_buffer);
  }
  
  // Process complete lines from buffer
  size_t newline_pos;
  int processed_lines = 0;
  
  while ((newline_pos = read_buffer_.find('\n')) != std::string::npos && processed_lines < 10) {
    std::string line = read_buffer_.substr(0, newline_pos);
    read_buffer_ = read_buffer_.substr(newline_pos + 1);
    processed_lines++;
    
    // Remove any carriage returns
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    
    // Parse the complete line
    if (parse_and_update_data(line)) {
      // Successfully parsed a line
      RCLCPP_DEBUG(rclcpp::get_logger("CartpoleHardwareInterface"), 
                   "Successfully parsed line: '%s'", line.c_str());
      break;
    } else {
      RCLCPP_DEBUG(rclcpp::get_logger("CartpoleHardwareInterface"), 
                   "Failed to parse line: '%s'", line.c_str());
    }
  }
  
  // Keep buffer size reasonable and clear if too much garbage
  if (read_buffer_.size() > 500) {
    RCLCPP_WARN(rclcpp::get_logger("CartpoleHardwareInterface"), 
                "Buffer too large (%zu), clearing", read_buffer_.size());
    read_buffer_.clear();
  }
  
  return hardware_interface::return_type::OK;
}

bool CartpoleHardwareInterface::parse_and_update_data(const std::string& line)
{
  if (line.empty() || line.length() < 5) {  // Minimum reasonable line length
    return false;
  }
  
  // Skip lines that don't look like CSV data
  if (line.find(',') == std::string::npos) {
    return false;
  }
  
  // Parse CSV formatted data
  std::istringstream ss(line);
  std::string token;
  std::vector<float> values;
  
  while (std::getline(ss, token, ',')) {
    // Trim whitespace
    token.erase(0, token.find_first_not_of(" \t\r\n"));
    token.erase(token.find_last_not_of(" \t\r\n") + 1);
    
    if (token.empty()) {
      continue;
    }
    
    try {
      float value = std::stof(token);
      // Check for reasonable ranges - increased tolerance
      if (std::abs(value) > 1e6) {  
        RCLCPP_DEBUG(rclcpp::get_logger("CartpoleHardwareInterface"), 
                     "Value out of range: %f", value);
        return false;
      }
      values.push_back(value);
    } catch (const std::exception& e) {
      RCLCPP_DEBUG(rclcpp::get_logger("CartpoleHardwareInterface"), 
                   "Failed to parse value: '%s' in line: '%s'", token.c_str(), line.c_str());
      return false;
    }
  }
  
  // Check if we have exactly 4 values (be strict about data format)
  if (values.size() != 4) {
    RCLCPP_DEBUG(rclcpp::get_logger("CartpoleHardwareInterface"), 
                 "Expected 4 values, got %zu in line: '%s'", values.size(), line.c_str());
    return false;
  }
  
  // Check for NaN/inf values 
  for (size_t i = 0; i < 4; i++) {
    if (!std::isfinite(values[i])) {
      RCLCPP_DEBUG(rclcpp::get_logger("CartpoleHardwareInterface"), 
                   "Non-finite value at index %zu: %f", i, values[i]);
      return false;
    }
  }

  // Only update if we have valid arrays
  if (hw_positions_.size() >= 2 && hw_velocities_.size() >= 2) {
    // Store previous values for comparison
    double prev_cart_pos = hw_positions_[0];
    double prev_pole_pos = hw_positions_[1];
    
    // Calculate new values
    double new_cart_pos = values[0] * -1 * CartPosConst;
    double new_pole_pos = values[2] * -1 * PoleRadConst;
    
    // Get velocities for adaptive thresholds
    double pole_velocity = values[3];
    double cart_velocity = values[1];
    
    // Adaptive cart position threshold - hızla birlikte artar
    double cart_threshold = 0.2;  // Base threshold
    if (std::abs(cart_velocity) > 20.0) {
      cart_threshold = 1.0;  // Çok yüksek hızda çok toleranslı
    } else if (std::abs(cart_velocity) > 10.0) {
      cart_threshold = 0.5;  // Yüksek hızda toleranslı
    } else if (std::abs(cart_velocity) > 5.0) {
      cart_threshold = 0.3;  // Orta hızda biraz toleranslı
    }
    
    // Smart validation for cart position with dynamic threshold
    if (!std::isnan(prev_cart_pos) && std::abs(new_cart_pos - prev_cart_pos) > cart_threshold) {
      RCLCPP_DEBUG(rclcpp::get_logger("CartpoleHardwareInterface"),
                   "Large cart position jump detected: %f -> %f (threshold: %f), rejecting", 
                   prev_cart_pos, new_cart_pos, cart_threshold);
      return false;
    }
    
    // Adaptive pole angle threshold - çok daha toleranslı hızlı düşmelerde
    double pole_threshold = 2.0;  // Base threshold (115 degrees) - artırıldı
    if (std::abs(pole_velocity) > 50.0) {
      pole_threshold = 5.0;  // Çok hızlı düşüyorsa çok çok toleranslı (287 degrees)
    } else if (std::abs(pole_velocity) > 30.0) {
      pole_threshold = 4.0;  // Çok hızlı düşüyorsa çok toleranslı (229 degrees)
    } else if (std::abs(pole_velocity) > 15.0) {
      pole_threshold = 3.0;  // Hızlı düşüyorsa toleranslı (172 degrees)
    } else if (std::abs(pole_velocity) > 5.0) {
      pole_threshold = 2.5;  // Orta hızda (143 degrees)
    }
    
    // Smart validation for pole position with wrap-around handling
    if (!std::isnan(prev_pole_pos)) {
      double angle_diff = new_pole_pos - prev_pole_pos;
      
      // Handle wrap-around: if difference is large, check if it's actually a small movement across 0/2π boundary
      if (std::abs(angle_diff) > M_PI) {
        // Check for wrap-around scenarios
        double wrapped_diff1 = angle_diff + 2*M_PI;  // Wrapped one way
        double wrapped_diff2 = angle_diff - 2*M_PI;  // Wrapped other way
        
        // Use the smallest absolute difference
        if (std::abs(wrapped_diff1) < std::abs(angle_diff)) {
          angle_diff = wrapped_diff1;
        } else if (std::abs(wrapped_diff2) < std::abs(angle_diff)) {
          angle_diff = wrapped_diff2;
        }
      }
      
      // Now check if the actual movement is reasonable with adaptive threshold
      if (std::abs(angle_diff) > pole_threshold) {
        RCLCPP_DEBUG(rclcpp::get_logger("CartpoleHardwareInterface"),
                     "Large pole position jump detected: %f -> %f (diff: %f, threshold: %f), rejecting", 
                     prev_pole_pos, new_pole_pos, angle_diff, pole_threshold);
        return false;
      }
    }
    
    // More lenient velocity checks - sadece çok aşırı durumlarda reddet
    if (std::abs(cart_velocity) > 500.0) {  // Çok çok yüksek cart hızı
      RCLCPP_DEBUG(rclcpp::get_logger("CartpoleHardwareInterface"),
                   "Unreasonable cart velocity %f, rejecting", cart_velocity);
      return false;
    }
    
    if (std::abs(pole_velocity) > 500.0) {  // Çok çok yüksek açısal hız
      RCLCPP_DEBUG(rclcpp::get_logger("CartpoleHardwareInterface"),
                   "Unreasonable pole velocity %f, rejecting", pole_velocity);
      return false;
    }
    
    // Update values
    hw_positions_[0] = new_cart_pos;    // Cart position
    hw_velocities_[0] = cart_velocity;  // Cart velocity
    hw_positions_[1] = new_pole_pos;    // Pole angle
    hw_velocities_[1] = pole_velocity;  // Pole angular velocity
    
    RCLCPP_DEBUG(rclcpp::get_logger("CartpoleHardwareInterface"),
            "Updated: cart_pos=%.4f cart_vel=%.4f pole_ang=%.4f pole_vel=%.4f (thresholds: cart=%.2f, pole=%.2f)",
            hw_positions_[0], hw_velocities_[0], hw_positions_[1], hw_velocities_[1], 
            cart_threshold, pole_threshold);
    return true;
  }
  
  return false;
}

hardware_interface::return_type CartpoleHardwareInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Mark unused parameters to avoid warnings
  (void)time;
  (void)period;
  
  if (connection_handle_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("CartpoleHardwareInterface"), "Cannot write: device not connected");
    return hardware_interface::return_type::ERROR;
  }
  
  // Send cart effort command to STM32
  if (hw_commands_.size() > 0) {
    // Format command as string in the format string float number
    // where X is the control value (PWM)
    std::string cmd =  "s" + std::to_string(hw_commands_[0]) + "\n";
    
    if (::write(connection_handle_, cmd.c_str(), cmd.length()) != static_cast<ssize_t>(cmd.length())) {
      RCLCPP_WARN(rclcpp::get_logger("CartpoleHardwareInterface"), 
                 "Failed to send complete command to STM32");
      return hardware_interface::return_type::ERROR;
    }
    
    RCLCPP_DEBUG(rclcpp::get_logger("CartpoleHardwareInterface"), 
               "Sent command: %s", cmd.c_str());
  }
  
  return hardware_interface::return_type::OK;
}

bool CartpoleHardwareInterface::connect_to_hardware()
{
  RCLCPP_INFO(rclcpp::get_logger("CartpoleHardwareInterface"), 
              "Connecting to Arduino on %s at %d baud", device_path_.c_str(), baud_rate_);
  
  // Open serial port
  connection_handle_ = open(device_path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  
  if (connection_handle_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("CartpoleHardwareInterface"), 
                "Failed to open serial port: %s", strerror(errno));
    return false;
  }
  
  // Configure serial port
  struct termios tty;
  memset(&tty, 0, sizeof(tty));
  
  if (tcgetattr(connection_handle_, &tty) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("CartpoleHardwareInterface"), 
                "Error from tcgetattr: %s", strerror(errno));
    close(connection_handle_);
    return false;
  }
  
  // Set baud rate
  speed_t baud;
  switch (baud_rate_) {
    case 9600:   baud = B9600;   break;
    case 19200:  baud = B19200;  break;
    case 38400:  baud = B38400;  break;
    case 57600:  baud = B57600;  break;
    case 115200: baud = B115200; break;
    default:     baud = B115200; break;
  }
  
  cfsetospeed(&tty, baud);
  cfsetispeed(&tty, baud);
  
  // Port settings: 8N1, raw mode
  tty.c_cflag |= (CLOCAL | CREAD);    // Ignore modem controls, enable reading
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;                 // 8-bit
  tty.c_cflag &= ~PARENB;             // No parity
  tty.c_cflag &= ~CSTOPB;             // 1 stop bit
  tty.c_cflag &= ~CRTSCTS;            // No hardware flow control
  
  // Raw mode - no canonical mode
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // Raw mode
  tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL);  // No flow control, no CR to NL
  tty.c_oflag &= ~OPOST;                           // Raw output
  
  // Timeout settings for non-blocking read
  tty.c_cc[VMIN] = 0;   // Non-blocking
  tty.c_cc[VTIME] = 1;  // 100ms timeout
  
  if (tcsetattr(connection_handle_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("CartpoleHardwareInterface"), 
                "Error from tcsetattr: %s", strerror(errno));
    close(connection_handle_);
    return false;
  }
  
  // Flush any pending data
  tcflush(connection_handle_, TCIOFLUSH);
  
  RCLCPP_INFO(rclcpp::get_logger("CartpoleHardwareInterface"), "Serial port opened successfully");
  return true;
}

}  // namespace cartpole_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  cartpole_controller::CartpoleHardwareInterface,
  hardware_interface::SystemInterface
)