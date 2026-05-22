#include "arm_hardware_interface/arm_hardware_interface.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <cmath>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace arm_hardware_interface
{

// ── Ring-buffer helpers ─────────────────────────────────────────────────────

std::size_t ArmHardwareInterface::ring_size() const
{
  return (ring_head_ - ring_tail_ + RING_BUF_SIZE) % RING_BUF_SIZE;
}

void ArmHardwareInterface::ring_push(uint8_t byte)
{
  // Overwrite oldest byte if full (shouldn't happen at 50 Hz with 64-byte buffer)
  ring_[ring_head_ % RING_BUF_SIZE] = byte;
  ring_head_ = (ring_head_ + 1) % RING_BUF_SIZE;
  // If full, advance tail to discard oldest byte
  if (ring_head_ == ring_tail_) {
    ring_tail_ = (ring_tail_ + 1) % RING_BUF_SIZE;
  }
}

uint8_t ArmHardwareInterface::ring_pop()
{
  uint8_t byte = ring_[ring_tail_];
  ring_tail_ = (ring_tail_ + 1) % RING_BUF_SIZE;
  return byte;
}

// Peek at a byte without consuming it; offset 0 = next byte to be popped
uint8_t ArmHardwareInterface::ring_peek(std::size_t offset) const
{
  return ring_[(ring_tail_ + offset) % RING_BUF_SIZE];
}

// ── Packet parser ───────────────────────────────────────────────────────────
// Returns true and updates hw_states_ when a valid complete packet is found.
// Discards bytes one at a time until a valid frame is found or the buffer
// is exhausted. Called repeatedly inside read() until it returns false.

bool ArmHardwareInterface::try_parse_packet()
{
  // Need at least PACKET_SIZE bytes in the ring buffer before we can try
  while (ring_size() >= PACKET_SIZE) {
    // 1. Sync: discard bytes until we see the start marker at position 0
    if (ring_peek(0) != RX_START_MARKER) {
      ring_pop();   // discard one stale byte and try again
      continue;
    }

    // 2. Check the end marker is exactly where it should be (offset 9)
    if (ring_peek(PACKET_SIZE - 1) != RX_END_MARKER) {
      // Start marker found but end marker wrong → corrupt frame.
      // Discard the start marker and re-sync on the next byte.
      ring_pop();
      continue;
    }

    // 3. Valid frame – consume all 10 bytes
    ring_pop();  // discard 0xCC start marker

    uint8_t payload[8];
    for (std::size_t i = 0; i < 8; ++i) {
      payload[i] = ring_pop();
    }
    ring_pop();  // discard 0xDD end marker

    float pos_a, pos_b;
    memcpy(&pos_a, &payload[0], sizeof(float));
    memcpy(&pos_b, &payload[4], sizeof(float));

    hw_states_[0] = static_cast<double>(pos_a);
    hw_states_[1] = static_cast<double>(pos_b);
    return true;
  }
  return false;
}

// ── Lifecycle ───────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn ArmHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Read serial_port from URDF <param> (falls back to /dev/ttyUSB0)
  if (info_.hardware_parameters.count("serial_port")) {
    serial_port_ = info_.hardware_parameters.at("serial_port");
  }
  RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface"),
    "Using serial port: %s", serial_port_.c_str());

  // Open serial port
  serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ArmHardwareInterface"),
      "Cannot open serial port: %s", serial_port_.c_str());
    return CallbackReturn::ERROR;
  }

  // Configure 115200 8N1 raw (non-blocking)
  struct termios tty;
  memset(&tty, 0, sizeof(tty));
  if (tcgetattr(serial_fd_, &tty) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ArmHardwareInterface"), "tcgetattr failed");
    close(serial_fd_);
    return CallbackReturn::ERROR;
  }
  cfsetospeed(&tty, B115200);
  cfsetispeed(&tty, B115200);
  tty.c_cflag  = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF);
  tty.c_oflag &= ~OPOST;
  tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  // Non-blocking read: return immediately with whatever is available
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 0;
  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ArmHardwareInterface"), "tcsetattr failed");
    close(serial_fd_);
    return CallbackReturn::ERROR;
  }

  // Flush any stale bytes left in the OS buffer from a previous run
  tcflush(serial_fd_, TCIOFLUSH);

  hw_commands_.resize(info_.joints.size(), 0.0);
  hw_states_.resize(info_.joints.size(), 0.0);

  RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface"),
    "Serial port opened OK. %zu joints configured.", info_.joints.size());
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface"), "Activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface"), "Deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── ros2_control interfaces ─────────────────────────────────────────────────

std::vector<hardware_interface::StateInterface>
ArmHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ArmHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
  }
  return command_interfaces;
}

// ── read() – drain OS buffer into ring buffer, then parse ──────────────────

hardware_interface::return_type ArmHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Drain whatever the OS has buffered into our ring buffer
  uint8_t temp[RING_BUF_SIZE];
  ssize_t n = ::read(serial_fd_, temp, sizeof(temp));
  if (n > 0) {
    for (ssize_t i = 0; i < n; ++i) {
      ring_push(temp[i]);
    }
  }

  // Parse as many complete packets as are available.
  // We keep the last successfully decoded positions in hw_states_.
  while (try_parse_packet()) {}

  return hardware_interface::return_type::OK;
}

// ── write() – send command packet ──────────────────────────────────────────

hardware_interface::return_type ArmHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  uint8_t packet[PACKET_SIZE];
  packet[0] = TX_START_MARKER;

  float cmd_a = static_cast<float>(hw_commands_[0]);
  float cmd_b = static_cast<float>(hw_commands_[1]);
  memcpy(&packet[1], &cmd_a, sizeof(float));
  memcpy(&packet[5], &cmd_b, sizeof(float));

  packet[9] = TX_END_MARKER;

  ssize_t written = ::write(serial_fd_, packet, PACKET_SIZE);
  if (written != static_cast<ssize_t>(PACKET_SIZE)) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("ArmHardwareInterface"),
      *rclcpp::Clock::make_shared(), 2000,
      "write() sent %zd/%zu bytes", written, PACKET_SIZE);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace arm_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  arm_hardware_interface::ArmHardwareInterface, hardware_interface::SystemInterface)