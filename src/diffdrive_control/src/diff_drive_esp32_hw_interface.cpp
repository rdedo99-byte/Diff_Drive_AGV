#include "diffdrive_control/diff_drive_esp32_hw_interface.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace diffdrive_control
{

const double TICKS_PER_REVOLUTION = 1070.0; 
const double RADIANS_PER_TICK = (2.0 * M_PI) / TICKS_PER_REVOLUTION;
const double VELOCITY_TO_PWM_RATIO = 45.0; // Feedforward Gain
const int MAX_COMMAND_PWM = 255;
const int MIN_COMMAND_THRESHOLD_PWM = 80;

hardware_interface::CallbackReturn ESP32DiffDriveHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  node_ = rclcpp::Node::make_shared("esp32_hw_interface");
  serial_buffer_ = ""; 
  
  port_ = info_.hardware_parameters.count("port") ? info_.hardware_parameters.at("port") : "/dev/ttyESP32";

  if (!open_serial()) {
    RCLCPP_ERROR(node_->get_logger(), "❌ Impossibile aprire porta %s", port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
   
  RCLCPP_INFO(node_->get_logger(), "✅ Hardware Interface Inizializzata (Jazzy API)");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ESP32DiffDriveHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  left_pos_ = 0.0; right_pos_ = 0.0; left_vel_ = 0.0; right_vel_ = 0.0;
  cmd_left_ = 0.0; cmd_right_ = 0.0;
  
  // Reset PID
  left_prev_error_ = 0.0; right_prev_error_ = 0.0;
  left_integral_ = 0.0; right_integral_ = 0.0;
  
  RCLCPP_INFO(node_->get_logger(), "Hardware Attivato");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ESP32DiffDriveHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (fd_ >= 0) {
    if (::write(fd_, "L0R0\n", 5) == -1) {
        RCLCPP_WARN(node_->get_logger(), "Errore invio stop comando in disattivazione");
    }
  }
  close_serial();
  RCLCPP_INFO(node_->get_logger(), "Hardware Disattivato");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ESP32DiffDriveHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back("joint_left_wheel", "position", &left_pos_);
  state_interfaces.emplace_back("joint_left_wheel", "velocity", &left_vel_);
  state_interfaces.emplace_back("joint_right_wheel", "position", &right_pos_);
  state_interfaces.emplace_back("joint_right_wheel", "velocity", &right_vel_);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ESP32DiffDriveHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back("joint_left_wheel", "velocity", &cmd_left_);
  command_interfaces.emplace_back("joint_right_wheel", "velocity", &cmd_right_);
  return command_interfaces;
}

hardware_interface::return_type ESP32DiffDriveHardware::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  std::lock_guard<std::mutex> lock(serial_mutex_);
  if (fd_ < 0) return hardware_interface::return_type::ERROR;

  double dt = period.seconds();
  char buf[1024]; 
  ssize_t n = ::read(fd_, buf, sizeof(buf) - 1);
  
  if (n > 0) {
    buf[n] = '\0';
    serial_buffer_.append(buf);
    size_t newline_pos;
    while ((newline_pos = serial_buffer_.find('\n')) != std::string::npos) {
      std::string line = serial_buffer_.substr(0, newline_pos);
      serial_buffer_.erase(0, newline_pos + 1);

      if (line.find("ENC") != std::string::npos) {
        std::replace(line.begin(), line.end(), 'E', ' ');
        std::replace(line.begin(), line.end(), 'N', ' ');
        std::replace(line.begin(), line.end(), 'C', ' ');
        
        std::istringstream iss(line);
        long l_ticks, r_ticks;
        if (iss >> l_ticks >> r_ticks) {
          double old_l = left_pos_; 
          double old_r = right_pos_;
          left_pos_ = l_ticks * RADIANS_PER_TICK;
          right_pos_ = r_ticks * RADIANS_PER_TICK;
          if (dt > 0.0) {
            left_vel_ = (left_pos_ - old_l) / dt;
            right_vel_ = (right_pos_ - old_r) / dt;
          }
        }
      }
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ESP32DiffDriveHardware::write(const rclcpp::Time &, const rclcpp::Duration & period)
{
  std::lock_guard<std::mutex> lock(serial_mutex_);
  if (fd_ < 0) return hardware_interface::return_type::ERROR;

  double dt = period.seconds();
  if (dt <= 0.0001) return hardware_interface::return_type::OK;

  auto calculate_pid_pwm = [&](double target_vel, double current_vel, double &prev_error, double &integral) -> int {
    if (std::abs(target_vel) < 0.01) {
      prev_error = 0.0;
      integral = 0.0;
      return 0; 
    }

    double error = target_vel - current_vel;
    integral += error * dt;
    integral = std::clamp(integral, -100.0, 100.0); 

    double derivative = (error - prev_error) / dt;
    prev_error = error;

    double pid_output = (p_gain_ * error) + (i_gain_ * integral) + (d_gain_ * derivative);
    double pwm_val = (target_vel * VELOCITY_TO_PWM_RATIO) + pid_output;

    return static_cast<int>(pwm_val);
  };

  int pwm_l = calculate_pid_pwm(cmd_left_, left_vel_, left_prev_error_, left_integral_);
  int pwm_r = calculate_pid_pwm(cmd_right_, right_vel_, right_prev_error_, right_integral_);

  auto limit_pwm = [](int val) {
    if (val == 0) return 0;
    if (std::abs(val) < MIN_COMMAND_THRESHOLD_PWM) {
       return (val > 0) ? MIN_COMMAND_THRESHOLD_PWM : -MIN_COMMAND_THRESHOLD_PWM;
    }
    return std::clamp(val, -MAX_COMMAND_PWM, MAX_COMMAND_PWM);
  };

  pwm_l = limit_pwm(pwm_l);
  pwm_r = limit_pwm(pwm_r);

  std::ostringstream ss;
  ss << "L" << pwm_l << "R" << pwm_r << "\n";
  std::string cmd = ss.str();
  
  if (::write(fd_, cmd.c_str(), cmd.size()) == -1) {
  }

  return hardware_interface::return_type::OK;
}

bool ESP32DiffDriveHardware::open_serial()
{
  fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) return false;

  struct termios tty;
  if (tcgetattr(fd_, &tty) != 0) { ::close(fd_); return false; }

  cfsetospeed(&tty, B115200);
  cfsetispeed(&tty, B115200);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; 
  tty.c_iflag &= ~IGNBRK;
  tty.c_lflag = 0; 
  tty.c_oflag = 0; 
  tty.c_cc[VMIN]  = 0; 
  tty.c_cc[VTIME] = 5; 

  tty.c_cflag |= (CLOCAL | CREAD); 
  tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS); 

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) { ::close(fd_); return false; }

  int flags;
  ioctl(fd_, TIOCMGET, &flags);
  flags &= ~TIOCM_DTR; flags &= ~TIOCM_RTS;
  ioctl(fd_, TIOCMSET, &flags);

  tcflush(fd_, TCIFLUSH);
  return true;
}

void ESP32DiffDriveHardware::close_serial() { if (fd_ >= 0) ::close(fd_); fd_ = -1; }

} // namespace diffdrive_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(diffdrive_control::ESP32DiffDriveHardware, hardware_interface::SystemInterface)