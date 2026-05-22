#include "arm_hardware_interface/arm_hardware_interface.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <cmath>
#include <iostream>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace arm_hardware_interface
{

hardware_interface::CallbackReturn ArmHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Open serial port
  serial_fd_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ArmHardwareInterface"), "Cannot open /dev/ttyUSB0");
    return CallbackReturn::ERROR;
  }

  // Configure serial port (115200 8N1 raw)
  struct termios tty;
  memset(&tty, 0, sizeof(tty));
  if (tcgetattr(serial_fd_, &tty) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ArmHardwareInterface"), "tcgetattr failed");
    close(serial_fd_);
    return CallbackReturn::ERROR;
  }
  cfsetospeed(&tty, B115200);
  cfsetispeed(&tty, B115200);
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // 8-bit
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
  tty.c_oflag &= ~OPOST;
  tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;
  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ArmHardwareInterface"), "tcsetattr failed");
    close(serial_fd_);
    return CallbackReturn::ERROR;
  }

  // Resize vectors based on joint info
  hw_commands_.resize(info_.joints.size(), 0.0);
  hw_states_.resize(info_.joints.size(), 0.0);

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArmHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArmHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::return_type ArmHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Read all available bytes from serial and parse packets
  uint8_t temp[256];
  ssize_t n = ::read(serial_fd_, temp, sizeof(temp));
  if (n <= 0) {
    return hardware_interface::return_type::OK;
  }

  for (ssize_t i = 0; i < n; ++i) {
    // Wait for start marker
    if (temp[i] == 0xCC) {
      // We need 10 bytes: 0xCC + 8 payload + 0xDD
      if (i + 9 < n) {
        // Check end marker
        if (temp[i + 9] == 0xDD) {
          float pos_a, pos_b;
          memcpy(&pos_a, &temp[i + 1], sizeof(float));
          memcpy(&pos_b, &temp[i + 5], sizeof(float));
          hw_states_[0] = pos_a;
          hw_states_[1] = pos_b;
          return hardware_interface::return_type::OK;
        }
      }
      // If we don't have enough bytes in this chunk, ignore and wait for next read
      // (Incomplete packet will be lost, but next complete one will be used)
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArmHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Build command packet: 0xAA + float joint_a + float joint_b + 0xBB
  uint8_t packet[10];
  packet[0] = 0xAA;
  memcpy(&packet[1], &hw_commands_[0], sizeof(float));  // joint_a
  memcpy(&packet[5], &hw_commands_[1], sizeof(float));  // joint_b
  packet[9] = 0xBB;

  ::write(serial_fd_, packet, 10);
  return hardware_interface::return_type::OK;
}

}  // namespace arm_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  arm_hardware_interface::ArmHardwareInterface, hardware_interface::SystemInterface)
