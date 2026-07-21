#ifndef ROBOT_ARM_HARDWARE__DYNAMIXEL_SYSTEM_HPP_
#define ROBOT_ARM_HARDWARE__DYNAMIXEL_SYSTEM_HPP_

#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace robot_arm_hardware
{

class DynamixelSystem : public hardware_interface::SystemInterface
{
public:
  ~DynamixelSystem() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
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
  struct JointConfig
  {
    uint8_t id;
    uint16_t expected_model;
    uint8_t expected_operating_mode;
    uint8_t expected_drive_mode;
    int64_t zero_ticks;
    double direction;
    double radians_per_tick;
    double min_position;
    double max_position;
  };

  bool ping_and_validate();
  bool set_torque(bool enabled);
  void close_port();
  static std::vector<uint8_t> make_4byte_payload(uint32_t value);

  rclcpp::Logger logger_{rclcpp::get_logger("robot_arm_hardware")};
  std::string device_name_;
  int baud_rate_{1000000};
  double protocol_version_{2.0};
  uint16_t drive_mode_address_{10};
  uint16_t operating_mode_address_{11};
  uint16_t torque_address_{64};
  uint16_t goal_position_address_{116};
  uint16_t present_velocity_address_{128};
  uint16_t present_position_address_{132};
  double velocity_rad_s_per_raw_unit_{0.023980824};

  std::vector<JointConfig> joints_;
  std::vector<double> position_states_;
  std::vector<double> velocity_states_;
  std::vector<double> position_commands_;
  std::vector<uint32_t> last_position_ticks_;

  dynamixel::PortHandler * port_handler_{nullptr};
  dynamixel::PacketHandler * packet_handler_{nullptr};
  std::unique_ptr<dynamixel::GroupSyncRead> state_reader_;
  bool torque_enabled_{false};
};

}  // namespace robot_arm_hardware

#endif  // ROBOT_ARM_HARDWARE__DYNAMIXEL_SYSTEM_HPP_
