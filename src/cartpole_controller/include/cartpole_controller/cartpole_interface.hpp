// include/cartpole_controller/cartpole_interface.hpp
#ifndef CARTPOLE_HARDWARE_INTERFACE_HPP
#define CARTPOLE_HARDWARE_INTERFACE_HPP

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <string>

namespace cartpole_controller {

class CartpoleHardwareInterface : public hardware_interface::SystemInterface {
public:
  CartpoleHardwareInterface();
  
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Hardware connection - burayı fiziksel donanıma göre düzenlemeniz gerekir
  // Örneğin: SerialPort, GPIO, PWM, vs.
  bool connect_to_hardware();
  
  // Parametreler
  std::string device_path_; // örneğin: "/dev/ttyUSB0"
  int baud_rate_;
  
  // Durum ve komut değişkenleri
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;
  std::vector<double> hw_commands_;
  
  // Donanım bağlantısını yönetecek değişkenler
  // (örneğin bir seri port için file descriptor)
  int connection_handle_;
};

}  // namespace cartpole_controller

#endif // CARTPOLE_HARDWARE_INTERFACE_HPP