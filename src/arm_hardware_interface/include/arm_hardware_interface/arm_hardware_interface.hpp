#ifndef ARM_HARDWARE_INTERFACE_HPP_
#define ARM_HARDWARE_INTERFACE_HPP_

#include <vector>
#include <string>
#include <array>
#include <cstdint>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace arm_hardware_interface
{

// Telemetry packet layout: 0xCC | float pos_a (4B) | float pos_b (4B) | 0xDD  = 10 bytes
// Command packet layout:   0xAA | float cmd_a (4B) | float cmd_b (4B) | 0xBB  = 10 bytes
static constexpr std::size_t PACKET_SIZE      = 10;
static constexpr uint8_t     RX_START_MARKER  = 0xCC;
static constexpr uint8_t     RX_END_MARKER    = 0xDD;
static constexpr uint8_t     TX_START_MARKER  = 0xAA;
static constexpr uint8_t     TX_END_MARKER    = 0xBB;
// Ring-buffer size: must be > 2× PACKET_SIZE to absorb bursts
static constexpr std::size_t RING_BUF_SIZE    = 64;

class ArmHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ArmHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ── Serial ──────────────────────────────────────────────────────────────
  int         serial_fd_{-1};
  std::string serial_port_{"/dev/ttyUSB0"};

  // ── Joint state / command mirrors ───────────────────────────────────────
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;

  // ── Persistent ring buffer for incoming bytes ────────────────────────────
  // head_ = next write index, tail_ = next read index
  std::array<uint8_t, RING_BUF_SIZE> ring_{};
  std::size_t ring_head_{0};
  std::size_t ring_tail_{0};

  // ── Helpers ──────────────────────────────────────────────────────────────
  std::size_t ring_size() const;
  void        ring_push(uint8_t byte);
  uint8_t     ring_pop();
  uint8_t     ring_peek(std::size_t offset) const;
  bool        try_parse_packet();
};

}  // namespace arm_hardware_interface

#endif  // ARM_HARDWARE_INTERFACE_HPP_