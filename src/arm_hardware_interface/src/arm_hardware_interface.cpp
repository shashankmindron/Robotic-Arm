#include "arm_hardware_interface/arm_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <fcntl.h>     // For open, O_RDWR, etc.
#include <termios.h>   // For tcgetattr, tcsetattr, cfsetospeed, etc.
#include <unistd.h>    // For read, write, close
#include <sys/ioctl.h> // For ioctl, FIONREAD
#include <cstring>     // For std::memcpy

namespace arm_hardware_interface
{
// Explicitly bring CallbackReturn into this namespace scope
using hardware_interface::CallbackReturn;

// Global or class member variable for the serial file descriptor
int serial_fd = -1;

CallbackReturn ArmHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  hw_states_positions_.resize(info_.joints.size(), 0.0);
  hw_commands_positions_.resize(info_.joints.size(), 0.0);

  std::string port = info_.hardware_parameters.at("serial_port");
  
  // Open with O_RDWR and O_NOCTTY
  // We remove O_NDELAY here if we want to handle blocking properly, 
  // or keep it if we are using a robust polling loop.
  serial_fd = open(port.c_str(), O_RDWR | O_NOCTTY); 
  
  if (serial_fd == -1) {
    RCLCPP_ERROR(rclcpp::get_logger("ArmHardwareInterface"), "Failed to open serial port: %s", port.c_str());
    return CallbackReturn::ERROR;
  }

  // 1. Flush the buffer immediately to clear out stale data
  tcflush(serial_fd, TCIOFLUSH);

  // 2. Configure Serial Port
  struct termios options;
  tcgetattr(serial_fd, &options);

  // This sets 8N1, turns off parity, echo, canonical mode, etc. 
  // It effectively replaces all those bitwise operations you had.
  cfmakeraw(&options); 

  // Set Speed
  cfsetispeed(&options, B115200);
  cfsetospeed(&options, B115200);

  // Set timeout for read (important for non-blocking read calls)
  options.c_cc[VMIN] = 0;  // Return immediately if no data
  options.c_cc[VTIME] = 1; // 0.1s timeout

  tcsetattr(serial_fd, TCSANOW, &options);

  RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface"), "Serial Pipeline initialized successfully on %s", port.c_str());

  return CallbackReturn::SUCCESS;
}

CallbackReturn ArmHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface"), "Activating hardware interface...");
  return CallbackReturn::SUCCESS;
}

CallbackReturn ArmHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface"), "Deactivating hardware interface...");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArmHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArmHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
  }
  return command_interfaces;
}

hardware_interface::return_type ArmHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (serial_fd == -1) return hardware_interface::return_type::ERROR;

  int available = 0;
  ioctl(serial_fd, FIONREAD, &available);

  if (available >= 10) {
    // Read the whole buffer so we don't leave stale data behind
    uint8_t buffer[128]; 
    int n = ::read(serial_fd, buffer, std::min(available, (int)sizeof(buffer)));

    // Scan through the buffer to find the LATEST valid packet
    // (We iterate backwards or forward to find the most recent 0xCC)
    for (int i = 0; i <= n - 10; i++) {
      if (buffer[i] == 0xCC && buffer[i + 9] == 0xDD) {
        float fb_a, fb_b;
        std::memcpy(&fb_a, &buffer[i + 1], 4);
        std::memcpy(&fb_b, &buffer[i + 5], 4);

        hw_states_positions_[0] = static_cast<double>(fb_a);
        hw_states_positions_[1] = static_cast<double>(fb_b);
        
        // We found a packet, we can break or continue to find a newer one
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArmHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (serial_fd == -1) return hardware_interface::return_type::ERROR;

  uint8_t tx_buffer[10];
  tx_buffer[0] = 0xAA; // Start marker
  
  float cmd_a = static_cast<float>(hw_commands_positions_[0]);

  RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface"), "Writing Command: %f to serial", cmd_a);

  float cmd_b = static_cast<float>(hw_commands_positions_[1]);
  
  std::memcpy(&tx_buffer[1], &cmd_a, 4);
  std::memcpy(&tx_buffer[5], &cmd_b, 4);
  tx_buffer[9] = 0xBB; // End marker

  // Non-blocking write
  ::write(serial_fd, tx_buffer, 10);

  return hardware_interface::return_type::OK;
}

} // namespace arm_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(arm_hardware_interface::ArmHardwareInterface, hardware_interface::SystemInterface)