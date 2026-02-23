#ifndef DIFFDRIVE_CONTROL__ESP32_HW_INTERFACE_HPP_
#define DIFFDRIVE_CONTROL__ESP32_HW_INTERFACE_HPP_

#include <string>
#include <vector>
#include <mutex>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace diffdrive_control
{
class ESP32DiffDriveHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ESP32DiffDriveHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool open_serial();
  void close_serial();

  int fd_ = -1;
  std::string port_;
  std::string serial_buffer_;
  std::mutex serial_mutex_;

  double left_pos_ = 0.0;
  double right_pos_ = 0.0;
  double left_vel_ = 0.0;
  double right_vel_ = 0.0;
  double cmd_left_ = 0.0;
  double cmd_right_ = 0.0;

  double left_prev_error_ = 0.0;
  double right_prev_error_ = 0.0;
  double left_integral_ = 0.0;
  double right_integral_ = 0.0;
  
  double p_gain_ = 50.0; 
  double i_gain_ = 15.0;  
  double d_gain_ = 0.5;   
  
  std::shared_ptr<rclcpp::Node> node_;
};
}  // namespace diffdrive_control

#endif  // DIFFDRIVE_CONTROL__ESP32_HW_INTERFACE_HPP_