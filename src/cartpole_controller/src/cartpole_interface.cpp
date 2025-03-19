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

namespace cartpole_controller {

CartpoleHardwareInterface::CartpoleHardwareInterface()
: connection_handle_(-1) {
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
  
  // Read data from Arduino
  char buffer[100];
  memset(buffer, 0, sizeof(buffer));
  
  // Line reading
  int bytes_read = 0;
  for (size_t i = 0; i < sizeof(buffer) - 1; i++) {
    if (::read(connection_handle_, &buffer[i], 1) > 0) {
      bytes_read++;
      if (buffer[i] == '\n') {
        buffer[i] = '\0'; // Null-terminate
        break;
      }
    } else {
      break;
    }
  }
  
  if (bytes_read <= 1) {
    // No data read or only newline character read
    return hardware_interface::return_type::OK; // Don't treat as error, just continue
  }
  
  // Parse CSV formatted data
  std::string line(buffer);
  std::istringstream ss(line);
  std::string token;
  std::vector<float> values;
  
  while (std::getline(ss, token, ',')) {
    try {
      values.push_back(std::stof(token));
    } catch (const std::exception& e) {
      RCLCPP_WARN(rclcpp::get_logger("CartpoleHardwareInterface"), 
                 "Failed to parse value: %s", token.c_str());
    }
  }
  
  // Check if we have enough data
  if (values.size() >= 4) {
    // Check for NaN values 
    bool has_nan = false;
    for (size_t i = 0; i < 4; i++) {
        if (std::isnan(values[i])) {
            has_nan = true;
            RCLCPP_WARN(rclcpp::get_logger("CartpoleHardwareInterface"), 
                       "Received NaN value at index %zu", i);
            break;
        }
    }

    // Assign values to state interfaces
    if (!has_nan && hw_positions_.size() >= 2 && hw_velocities_.size() >= 2) {
            hw_positions_[0] = values[0];  // Cart position
            hw_velocities_[0] = values[1]; // Cart velocity
            hw_positions_[1] = values[2];  // Pole angle
            hw_velocities_[1] = values[3]; // Pole angular velocity
            
            RCLCPP_DEBUG(rclcpp::get_logger("CartpoleHardwareInterface"),
                    "Read: pos=%.3f vel=%.3f ang=%.3f angvel=%.3f",
                    hw_positions_[0], hw_velocities_[0], hw_positions_[1], hw_velocities_[1]);
        } else {
            // Use default safe values if NaN detected
            hw_positions_[0] = 0.0;  // Cart position
            hw_velocities_[0] = 0.0; // Cart velocity
            hw_positions_[1] = 0.0;  // Pole angle
            hw_velocities_[1] = 0.0; // Pole angular velocity
            
            RCLCPP_WARN(rclcpp::get_logger("CartpoleHardwareInterface"),
                    "Using default values due to NaN in received data");
        }
    } else {
        RCLCPP_WARN(rclcpp::get_logger("CartpoleHardwareInterface"), 
                "Received incomplete data: %s", line.c_str());
        
        // Use default safe values for incomplete data too
        if (hw_positions_.size() >= 2 && hw_velocities_.size() >= 2) {
            hw_positions_[0] = 0.0;  // Cart position
            hw_velocities_[0] = 0.0; // Cart velocity
            hw_positions_[1] = 0.0;  // Pole angle
            hw_velocities_[1] = 0.0; // Pole angular velocity
        }
    }
  
  return hardware_interface::return_type::OK;
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
  
  // Send cart effort command to Arduino (using only the first joint's command)
  if (hw_commands_.size() > 0) {
    // Format command as string and send
    std::string cmd = std::to_string(hw_commands_[0]) + "\n";
    
    if (::write(connection_handle_, cmd.c_str(), cmd.length()) != static_cast<ssize_t>(cmd.length())) {
      RCLCPP_WARN(rclcpp::get_logger("CartpoleHardwareInterface"), 
                 "Failed to send complete command to Arduino");
      return hardware_interface::return_type::ERROR;
    }
    
    RCLCPP_DEBUG(rclcpp::get_logger("CartpoleHardwareInterface"), 
               "Sent command: %.3f", hw_commands_[0]);
  }
  
  return hardware_interface::return_type::OK;
}

bool CartpoleHardwareInterface::connect_to_hardware()
{
  RCLCPP_INFO(rclcpp::get_logger("CartpoleHardwareInterface"), 
              "Connecting to Arduino on %s at %d baud", device_path_.c_str(), baud_rate_);
  
  // Open serial port
  connection_handle_ = open(device_path_.c_str(), O_RDWR | O_NOCTTY);
  
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
  
  // Port settings: 8N1, canonical mode (line-based reading)
  tty.c_cflag |= (CLOCAL | CREAD);    // Ignore modem controls, enable reading
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;                 // 8-bit
  tty.c_cflag &= ~PARENB;             // No parity
  tty.c_cflag &= ~CSTOPB;             // 1 stop bit
  tty.c_cflag &= ~CRTSCTS;            // No hardware flow control
  
  // Canonical mode (line-based)
  tty.c_lflag |= (ICANON | ECHOE);
  tty.c_iflag |= (ICRNL);             // CR to NL
  
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