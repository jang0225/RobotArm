#include "robot_arm_hardware/dynamixel_system.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot_arm_hardware
{

namespace
{
template<typename T>
T parameter_as(const std::unordered_map<std::string, std::string> & parameters, const std::string & name);

template<>
int parameter_as<int>(
  const std::unordered_map<std::string, std::string> & parameters, const std::string & name)
{
  return std::stoi(parameters.at(name));
}

template<>
int64_t parameter_as<int64_t>(
  const std::unordered_map<std::string, std::string> & parameters, const std::string & name)
{
  return std::stoll(parameters.at(name));
}

template<>
double parameter_as<double>(
  const std::unordered_map<std::string, std::string> & parameters, const std::string & name)
{
  return std::stod(parameters.at(name));
}
}  // namespace

DynamixelSystem::~DynamixelSystem()
{
  if (port_handler_ != nullptr) {
    if (torque_enabled_) {
      configure_bus_watchdog(false);
      set_torque(false);
    }
    close_port();
  }
}

hardware_interface::CallbackReturn DynamixelSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    device_name_ = info_.hardware_parameters.at("device_name");
    baud_rate_ = parameter_as<int>(info_.hardware_parameters, "baud_rate");
    protocol_version_ = parameter_as<double>(info_.hardware_parameters, "protocol_version");
    drive_mode_address_ = static_cast<uint16_t>(
      parameter_as<int>(info_.hardware_parameters, "drive_mode_address"));
    operating_mode_address_ = static_cast<uint16_t>(
      parameter_as<int>(info_.hardware_parameters, "operating_mode_address"));
    torque_address_ = static_cast<uint16_t>(
      parameter_as<int>(info_.hardware_parameters, "torque_enable_address"));
    goal_position_address_ = static_cast<uint16_t>(
      parameter_as<int>(info_.hardware_parameters, "goal_position_address"));
    present_velocity_address_ = static_cast<uint16_t>(
      parameter_as<int>(info_.hardware_parameters, "present_velocity_address"));
    present_position_address_ = static_cast<uint16_t>(
      parameter_as<int>(info_.hardware_parameters, "present_position_address"));
    present_current_address_ = static_cast<uint16_t>(
      parameter_as<int>(info_.hardware_parameters, "present_current_address"));
    present_input_voltage_address_ = static_cast<uint16_t>(
      parameter_as<int>(info_.hardware_parameters, "present_input_voltage_address"));
    present_temperature_address_ = static_cast<uint16_t>(
      parameter_as<int>(info_.hardware_parameters, "present_temperature_address"));
    hardware_error_status_address_ = static_cast<uint16_t>(
      parameter_as<int>(info_.hardware_parameters, "hardware_error_status_address"));
    bus_watchdog_address_ = static_cast<uint16_t>(
      parameter_as<int>(info_.hardware_parameters, "bus_watchdog_address"));
    const int bus_watchdog_raw = parameter_as<int>(
      info_.hardware_parameters, "bus_watchdog_raw");
    const int diagnostic_read_interval = parameter_as<int>(
      info_.hardware_parameters, "diagnostic_read_interval");
    const int max_consecutive_read_failures = parameter_as<int>(
      info_.hardware_parameters, "max_consecutive_read_failures");
    velocity_rad_s_per_raw_unit_ = parameter_as<double>(
      info_.hardware_parameters, "velocity_rad_s_per_raw_unit");
    current_amp_per_raw_unit_ = parameter_as<double>(
      info_.hardware_parameters, "current_amp_per_raw_unit");
    if (bus_watchdog_raw < 1 || bus_watchdog_raw > 127 ||
      diagnostic_read_interval < 1 || max_consecutive_read_failures < 1 ||
      velocity_rad_s_per_raw_unit_ <= 0.0 || current_amp_per_raw_unit_ <= 0.0)
    {
      throw std::runtime_error("invalid Dynamixel watchdog or diagnostic parameter");
    }
    bus_watchdog_raw_ = static_cast<uint8_t>(bus_watchdog_raw);
    diagnostic_read_interval_ = static_cast<std::size_t>(diagnostic_read_interval);
    max_consecutive_read_failures_ =
      static_cast<std::size_t>(max_consecutive_read_failures);

    joints_.reserve(info_.joints.size());
    for (const auto & joint : info_.joints) {
      if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        throw std::runtime_error(joint.name + " must have one position command interface");
      }
      static const std::vector<std::string> required_state_interfaces{
        hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
        "current", "voltage", "temperature", "hardware_error", "bus_watchdog",
        "communication_ok"};
      if (joint.state_interfaces.size() != required_state_interfaces.size()) {
        throw std::runtime_error(joint.name + " has an unexpected state interface count");
      }
      for (std::size_t i = 0; i < required_state_interfaces.size(); ++i) {
        if (joint.state_interfaces[i].name != required_state_interfaces[i]) {
          throw std::runtime_error(
                  joint.name + " state interfaces must follow the documented order");
        }
      }

      JointConfig config;
      config.id = static_cast<uint8_t>(parameter_as<int>(joint.parameters, "motor_id"));
      config.expected_model = static_cast<uint16_t>(
        parameter_as<int>(joint.parameters, "expected_model"));
      config.expected_operating_mode = static_cast<uint8_t>(
        parameter_as<int>(joint.parameters, "expected_operating_mode"));
      config.expected_drive_mode = static_cast<uint8_t>(
        parameter_as<int>(joint.parameters, "expected_drive_mode"));
      config.zero_ticks = parameter_as<int64_t>(joint.parameters, "zero_ticks");
      config.direction = parameter_as<double>(joint.parameters, "direction");
      config.radians_per_tick = parameter_as<double>(joint.parameters, "radians_per_tick");
      config.min_position = std::stod(joint.command_interfaces[0].min);
      config.max_position = std::stod(joint.command_interfaces[0].max);
      if (config.direction == 0.0 || config.radians_per_tick <= 0.0 ||
        config.min_position > config.max_position)
      {
        throw std::runtime_error("invalid parameters for " + joint.name);
      }
      joints_.push_back(config);
    }
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(logger_, "Hardware parameter error: %s", exception.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto nan = std::numeric_limits<double>::quiet_NaN();
  position_states_.assign(joints_.size(), nan);
  velocity_states_.assign(joints_.size(), nan);
  position_commands_.assign(joints_.size(), nan);
  current_states_.assign(joints_.size(), nan);
  voltage_states_.assign(joints_.size(), nan);
  temperature_states_.assign(joints_.size(), nan);
  hardware_error_states_.assign(joints_.size(), nan);
  bus_watchdog_states_.assign(joints_.size(), nan);
  communication_ok_states_.assign(joints_.size(), 0.0);
  last_position_ticks_.assign(joints_.size(), std::numeric_limits<uint32_t>::max());
  command_saturated_.assign(joints_.size(), false);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DynamixelSystem::on_configure(
  const rclcpp_lifecycle::State &)
{
  port_handler_ = dynamixel::PortHandler::getPortHandler(device_name_.c_str());
  packet_handler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version_);
  if (!port_handler_->openPort()) {
    RCLCPP_ERROR(
      logger_, "Cannot open %s: %s. Close Dynamixel Wizard and check dialout permission.",
      device_name_.c_str(), std::strerror(errno));
    close_port();
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!port_handler_->setBaudRate(baud_rate_)) {
    RCLCPP_ERROR(logger_, "Cannot set %s to %d bps", device_name_.c_str(), baud_rate_);
    close_port();
    return hardware_interface::CallbackReturn::ERROR;
  }
  const uint16_t state_start_address = std::min(
    bus_watchdog_address_, std::min(present_velocity_address_, present_position_address_));
  const uint16_t state_end_address = static_cast<uint16_t>(
    std::max(present_velocity_address_, present_position_address_) + 4);
  state_reader_ = std::make_unique<dynamixel::GroupSyncRead>(
    port_handler_, packet_handler_, state_start_address,
    static_cast<uint16_t>(state_end_address - state_start_address));
  const uint16_t diagnostic_start_address = std::min(
    present_current_address_,
    std::min(present_input_voltage_address_, present_temperature_address_));
  const uint16_t diagnostic_end_address = std::max(
    static_cast<uint16_t>(present_current_address_ + 2),
    std::max(
      static_cast<uint16_t>(present_input_voltage_address_ + 2),
      static_cast<uint16_t>(present_temperature_address_ + 1)));
  diagnostic_reader_ = std::make_unique<dynamixel::GroupSyncRead>(
    port_handler_, packet_handler_, diagnostic_start_address,
    static_cast<uint16_t>(diagnostic_end_address - diagnostic_start_address));
  const uint16_t fault_start_address = std::min(
    hardware_error_status_address_, bus_watchdog_address_);
  const uint16_t fault_end_address = static_cast<uint16_t>(
    std::max(hardware_error_status_address_, bus_watchdog_address_) + 1);
  fault_reader_ = std::make_unique<dynamixel::GroupSyncRead>(
    port_handler_, packet_handler_, fault_start_address,
    static_cast<uint16_t>(fault_end_address - fault_start_address));
  for (const auto & joint : joints_) {
    if (!state_reader_->addParam(joint.id) ||
      !diagnostic_reader_->addParam(joint.id) ||
      !fault_reader_->addParam(joint.id))
    {
      RCLCPP_ERROR(logger_, "Failed to prepare sync read for ID %u", joint.id);
      close_port();
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  if (!ping_and_validate()) {
    close_port();
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (read(rclcpp::Time(0), rclcpp::Duration(0, 0)) != hardware_interface::return_type::OK) {
    close_port();
    return hardware_interface::CallbackReturn::ERROR;
  }
  position_commands_ = position_states_;
  bool critical_fault = false;
  read_diagnostics(critical_fault);
  if (critical_fault) {
    RCLCPP_ERROR(logger_, "Dynamixel reported a hardware fault during configuration");
    close_port();
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(logger_, "Configured %zu Dynamixel joints on %s", joints_.size(), device_name_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DynamixelSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (read(rclcpp::Time(0), rclcpp::Duration(0, 0)) != hardware_interface::return_type::OK) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  position_commands_ = position_states_;
  std::fill(
    last_position_ticks_.begin(), last_position_ticks_.end(),
    std::numeric_limits<uint32_t>::max());
  std::fill(command_saturated_.begin(), command_saturated_.end(), false);
  consecutive_read_failures_ = 0;
  read_cycle_count_ = 0;
  if (!configure_bus_watchdog(false) ||
    write(rclcpp::Time(0), rclcpp::Duration(0, 0)) != hardware_interface::return_type::OK ||
    !set_torque(true) || !configure_bus_watchdog(true))
  {
    set_torque(false);
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(logger_, "Dynamixel torque enabled");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DynamixelSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  const bool watchdog_disabled = configure_bus_watchdog(false);
  const bool torque_disabled = set_torque(false);
  if (!watchdog_disabled || !torque_disabled) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  std::fill(
    last_position_ticks_.begin(), last_position_ticks_.end(),
    std::numeric_limits<uint32_t>::max());
  RCLCPP_INFO(logger_, "Dynamixel torque disabled");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DynamixelSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(joints_.size() * 8);
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]);
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]);
    interfaces.emplace_back(info_.joints[i].name, "current", &current_states_[i]);
    interfaces.emplace_back(info_.joints[i].name, "voltage", &voltage_states_[i]);
    interfaces.emplace_back(info_.joints[i].name, "temperature", &temperature_states_[i]);
    interfaces.emplace_back(
      info_.joints[i].name, "hardware_error", &hardware_error_states_[i]);
    interfaces.emplace_back(
      info_.joints[i].name, "bus_watchdog", &bus_watchdog_states_[i]);
    interfaces.emplace_back(
      info_.joints[i].name, "communication_ok", &communication_ok_states_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> DynamixelSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(joints_.size());
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]);
  }
  return interfaces;
}

hardware_interface::return_type DynamixelSystem::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  const int result = state_reader_->txRxPacket();
  if (result != COMM_SUCCESS) {
    std::fill(communication_ok_states_.begin(), communication_ok_states_.end(), 0.0);
    return handle_read_failure(
      std::string("sync read failed: ") + packet_handler_->getTxRxResult(result));
  }

  bool all_valid = true;
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    uint8_t error = 0;
    if (!state_reader_->getError(joints_[i].id, &error) || error != 0 ||
      !state_reader_->isAvailable(joints_[i].id, bus_watchdog_address_, 1) ||
      !state_reader_->isAvailable(joints_[i].id, present_position_address_, 4) ||
      !state_reader_->isAvailable(joints_[i].id, present_velocity_address_, 4))
    {
      communication_ok_states_[i] = 0.0;
      all_valid = false;
      continue;
    }
    communication_ok_states_[i] = 1.0;
    const uint8_t watchdog = static_cast<uint8_t>(
      state_reader_->getData(joints_[i].id, bus_watchdog_address_, 1));
    bus_watchdog_states_[i] = watchdog == 0xFFU ? -1.0 : static_cast<double>(watchdog);
    if (watchdog == 0xFFU) {
      RCLCPP_ERROR(logger_, "ID %u Bus Watchdog has stopped the motor", joints_[i].id);
      return hardware_interface::return_type::ERROR;
    }
    const uint32_t raw_position = state_reader_->getData(
      joints_[i].id, present_position_address_, 4);
    const uint32_t raw_velocity = state_reader_->getData(
      joints_[i].id, present_velocity_address_, 4);

    const int64_t signed_velocity = raw_velocity <=
      static_cast<uint32_t>(std::numeric_limits<int32_t>::max()) ?
      static_cast<int64_t>(raw_velocity) : static_cast<int64_t>(raw_velocity) - (1LL << 32);
    position_states_[i] = joints_[i].direction *
      (static_cast<double>(raw_position) - static_cast<double>(joints_[i].zero_ticks)) *
      joints_[i].radians_per_tick;
    velocity_states_[i] = joints_[i].direction * static_cast<double>(signed_velocity) *
      velocity_rad_s_per_raw_unit_;
  }
  if (!all_valid) {
    return handle_read_failure("one or more Dynamixel state responses were invalid");
  }
  consecutive_read_failures_ = 0;

  ++read_cycle_count_;
  if (read_cycle_count_ >= diagnostic_read_interval_) {
    read_cycle_count_ = 0;
    bool critical_fault = false;
    if (!read_diagnostics(critical_fault)) {
      RCLCPP_WARN(logger_, "Dynamixel diagnostic read failed; state control remains active");
    }
    if (critical_fault) {
      RCLCPP_ERROR(logger_, "Critical Dynamixel hardware/watchdog fault detected");
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DynamixelSystem::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  std::vector<uint32_t> position_ticks(joints_.size());
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const double command = position_commands_[i];
    double bounded_command = command;
    if (!std::isfinite(command)) {
      // A bad command on one interface must not stop valid commands for the
      // other motors. Hold only the affected joint at its measured position.
      bounded_command = std::isfinite(position_states_[i]) ?
        std::clamp(position_states_[i], joints_[i].min_position, joints_[i].max_position) :
        0.0;
      if (!command_saturated_[i]) {
        RCLCPP_ERROR(
          logger_, "%s received a non-finite command; holding this joint only",
          info_.joints[i].name.c_str());
      }
      command_saturated_[i] = true;
    } else {
      bounded_command = std::clamp(
        command, joints_[i].min_position, joints_[i].max_position);
      const bool saturated = bounded_command != command;
      if (saturated && !command_saturated_[i]) {
        RCLCPP_WARN(
          logger_, "%s command %.9f saturated to %.9f within [%.9f, %.9f]",
          info_.joints[i].name.c_str(), command, bounded_command,
          joints_[i].min_position, joints_[i].max_position);
      } else if (!saturated && command_saturated_[i]) {
        RCLCPP_INFO(logger_, "%s command returned inside its limits", info_.joints[i].name.c_str());
      }
      command_saturated_[i] = saturated;
    }

    const double raw_value = static_cast<double>(joints_[i].zero_ticks) +
      bounded_command / (joints_[i].direction * joints_[i].radians_per_tick);
    position_ticks[i] = static_cast<uint32_t>(
      std::llround(std::clamp(raw_value, 0.0, 4095.0)));
  }

  dynamixel::GroupSyncWrite writer(port_handler_, packet_handler_, goal_position_address_, 4);
  std::vector<std::vector<uint8_t>> payloads;
  payloads.reserve(joints_.size());
  std::vector<std::size_t> written_indices;
  written_indices.reserve(joints_.size());
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    if (position_ticks[i] == last_position_ticks_[i]) {
      continue;
    }
    payloads.push_back(make_4byte_payload(position_ticks[i]));
    if (!writer.addParam(joints_[i].id, payloads.back().data())) {
      // Keep preparing the packet so a local failure for one motor does not
      // discard commands already prepared for the other motors.
      RCLCPP_ERROR(
        logger_, "Failed to prepare write for ID %u; other motor commands will continue",
        joints_[i].id);
      payloads.pop_back();
      continue;
    }
    written_indices.push_back(i);
  }
  if (written_indices.empty()) {
    return hardware_interface::return_type::OK;
  }

  const int result = writer.txPacket();
  if (result != COMM_SUCCESS) {
    RCLCPP_ERROR(logger_, "Sync write failed: %s", packet_handler_->getTxRxResult(result));
    return hardware_interface::return_type::ERROR;
  }
  for (const auto i : written_indices) {
    last_position_ticks_[i] = position_ticks[i];
  }
  return hardware_interface::return_type::OK;
}

bool DynamixelSystem::ping_and_validate()
{
  for (const auto & joint : joints_) {
    uint16_t model = 0;
    uint8_t error = 0;
    int result = packet_handler_->ping(port_handler_, joint.id, &model, &error);
    if (result != COMM_SUCCESS || error != 0 || model != joint.expected_model) {
      RCLCPP_ERROR(
        logger_, "ID %u ping/model check failed (expected %u, actual %u)",
        joint.id, joint.expected_model, model);
      return false;
    }

    uint8_t operating_mode = 0;
    error = 0;
    result = packet_handler_->read1ByteTxRx(
      port_handler_, joint.id, operating_mode_address_, &operating_mode, &error);
    if (result != COMM_SUCCESS || error != 0 ||
      operating_mode != joint.expected_operating_mode)
    {
      RCLCPP_ERROR(
        logger_, "ID %u Operating Mode must be %u (actual %u)",
        joint.id, joint.expected_operating_mode, operating_mode);
      return false;
    }

    uint8_t drive_mode = 0;
    error = 0;
    result = packet_handler_->read1ByteTxRx(
      port_handler_, joint.id, drive_mode_address_, &drive_mode, &error);
    if (result != COMM_SUCCESS || error != 0 || drive_mode != joint.expected_drive_mode) {
      RCLCPP_ERROR(
        logger_, "ID %u Drive Mode must be %u (actual %u)",
        joint.id, joint.expected_drive_mode, drive_mode);
      return false;
    }
  }
  return true;
}

bool DynamixelSystem::set_torque(bool enabled)
{
  bool success = true;
  for (const auto & joint : joints_) {
    uint8_t error = 0;
    const int result = packet_handler_->write1ByteTxRx(
      port_handler_, joint.id, torque_address_, enabled ? 1 : 0, &error);
    if (result != COMM_SUCCESS || error != 0) {
      RCLCPP_ERROR(logger_, "Failed to set torque on ID %u", joint.id);
      success = false;
    }
  }
  torque_enabled_ = enabled && success;
  if (enabled && !success) {
    for (const auto & joint : joints_) {
      uint8_t error = 0;
      packet_handler_->write1ByteTxRx(port_handler_, joint.id, torque_address_, 0, &error);
    }
    torque_enabled_ = false;
  }
  return success;
}

bool DynamixelSystem::configure_bus_watchdog(bool enabled)
{
  bool success = true;
  const uint8_t value = enabled ? bus_watchdog_raw_ : 0;
  for (const auto & joint : joints_) {
    uint8_t error = 0;
    const int result = packet_handler_->write1ByteTxRx(
      port_handler_, joint.id, bus_watchdog_address_, value, &error);
    if (result != COMM_SUCCESS || error != 0) {
      RCLCPP_ERROR(
        logger_, "Failed to %s Bus Watchdog on ID %u",
        enabled ? "enable" : "clear", joint.id);
      success = false;
    }
  }
  if (success) {
    RCLCPP_INFO(
      logger_, "Dynamixel Bus Watchdog %s (%u ms)",
      enabled ? "enabled" : "disabled",
      enabled ? static_cast<unsigned int>(bus_watchdog_raw_) * 20U : 0U);
  }
  return success;
}

bool DynamixelSystem::read_diagnostics(bool & critical_fault)
{
  critical_fault = false;
  const int diagnostic_result = diagnostic_reader_->txRxPacket();
  const int fault_result = fault_reader_->txRxPacket();
  if (diagnostic_result != COMM_SUCCESS || fault_result != COMM_SUCCESS) {
    return false;
  }

  bool complete = true;
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const auto id = joints_[i].id;
    if (!diagnostic_reader_->isAvailable(id, present_current_address_, 2) ||
      !diagnostic_reader_->isAvailable(id, present_input_voltage_address_, 2) ||
      !diagnostic_reader_->isAvailable(id, present_temperature_address_, 1) ||
      !fault_reader_->isAvailable(id, hardware_error_status_address_, 1) ||
      !fault_reader_->isAvailable(id, bus_watchdog_address_, 1))
    {
      complete = false;
      continue;
    }

    const uint16_t raw_current = static_cast<uint16_t>(
      diagnostic_reader_->getData(id, present_current_address_, 2));
    const int32_t signed_current = raw_current <=
      static_cast<uint16_t>(std::numeric_limits<int16_t>::max()) ?
      static_cast<int32_t>(raw_current) : static_cast<int32_t>(raw_current) - (1 << 16);
    current_states_[i] = static_cast<double>(signed_current) * current_amp_per_raw_unit_;
    voltage_states_[i] = static_cast<double>(
      diagnostic_reader_->getData(id, present_input_voltage_address_, 2)) * 0.1;
    temperature_states_[i] = static_cast<double>(
      diagnostic_reader_->getData(id, present_temperature_address_, 1));
    const uint8_t hardware_error = static_cast<uint8_t>(
      fault_reader_->getData(id, hardware_error_status_address_, 1));
    const uint8_t watchdog = static_cast<uint8_t>(
      fault_reader_->getData(id, bus_watchdog_address_, 1));
    hardware_error_states_[i] = static_cast<double>(hardware_error);
    bus_watchdog_states_[i] = watchdog == 0xFFU ? -1.0 : static_cast<double>(watchdog);
    if (hardware_error != 0 || watchdog == 0xFFU) {
      critical_fault = true;
      RCLCPP_ERROR(
        logger_, "ID %u fault: Hardware Error=0x%02X, Bus Watchdog=%d",
        id, hardware_error, watchdog == 0xFFU ? -1 : static_cast<int>(watchdog));
    }
  }
  return complete;
}

hardware_interface::return_type DynamixelSystem::handle_read_failure(const std::string & reason)
{
  ++consecutive_read_failures_;
  if (!torque_enabled_ || consecutive_read_failures_ >= max_consecutive_read_failures_) {
    RCLCPP_ERROR(
      logger_, "%s (%zu/%zu); stopping the complete arm for a communication fault",
      reason.c_str(), consecutive_read_failures_, max_consecutive_read_failures_);
    return hardware_interface::return_type::ERROR;
  }
  RCLCPP_WARN(
    logger_, "%s (%zu/%zu); keeping previous state for this cycle",
    reason.c_str(), consecutive_read_failures_, max_consecutive_read_failures_);
  return hardware_interface::return_type::OK;
}

void DynamixelSystem::close_port()
{
  state_reader_.reset();
  diagnostic_reader_.reset();
  fault_reader_.reset();
  if (port_handler_ != nullptr) {
    port_handler_->closePort();
    port_handler_ = nullptr;
  }
}

std::vector<uint8_t> DynamixelSystem::make_4byte_payload(uint32_t value)
{
  return {
    DXL_LOBYTE(DXL_LOWORD(value)), DXL_HIBYTE(DXL_LOWORD(value)),
    DXL_LOBYTE(DXL_HIWORD(value)), DXL_HIBYTE(DXL_HIWORD(value))};
}

}  // namespace robot_arm_hardware

PLUGINLIB_EXPORT_CLASS(
  robot_arm_hardware::DynamixelSystem, hardware_interface::SystemInterface)
