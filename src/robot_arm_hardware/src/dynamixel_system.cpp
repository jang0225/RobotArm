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
    velocity_rad_s_per_raw_unit_ = parameter_as<double>(
      info_.hardware_parameters, "velocity_rad_s_per_raw_unit");

    joints_.reserve(info_.joints.size());
    for (const auto & joint : info_.joints) {
      if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        throw std::runtime_error(joint.name + " must have one position command interface");
      }
      if (joint.state_interfaces.size() != 2 ||
        joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
        joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        throw std::runtime_error(joint.name + " must have position and velocity state interfaces");
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
  last_position_ticks_.assign(joints_.size(), std::numeric_limits<uint32_t>::max());
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
    present_velocity_address_, present_position_address_);
  const uint16_t state_end_address = static_cast<uint16_t>(
    std::max(present_velocity_address_, present_position_address_) + 4);
  state_reader_ = std::make_unique<dynamixel::GroupSyncRead>(
    port_handler_, packet_handler_, state_start_address,
    static_cast<uint16_t>(state_end_address - state_start_address));
  for (const auto & joint : joints_) {
    if (!state_reader_->addParam(joint.id)) {
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
  if (write(rclcpp::Time(0), rclcpp::Duration(0, 0)) != hardware_interface::return_type::OK ||
    !set_torque(true))
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
  if (!set_torque(false)) {
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
  interfaces.reserve(joints_.size() * 2);
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]);
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]);
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
    RCLCPP_ERROR(logger_, "Sync read failed: %s", packet_handler_->getTxRxResult(result));
    return hardware_interface::return_type::ERROR;
  }

  for (std::size_t i = 0; i < joints_.size(); ++i) {
    uint8_t error = 0;
    if (!state_reader_->getError(joints_[i].id, &error) || error != 0 ||
      !state_reader_->isAvailable(joints_[i].id, present_position_address_, 4) ||
      !state_reader_->isAvailable(joints_[i].id, present_velocity_address_, 4))
    {
      RCLCPP_ERROR(logger_, "Invalid sync-read response from ID %u", joints_[i].id);
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
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DynamixelSystem::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  std::vector<uint32_t> position_ticks;
  position_ticks.reserve(joints_.size());
  bool command_changed = false;
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const double command = position_commands_[i];
    if (!std::isfinite(command) || command < joints_[i].min_position ||
      command > joints_[i].max_position)
    {
      RCLCPP_ERROR(
        logger_, "%s command %.4f outside [%.4f, %.4f]",
        info_.joints[i].name.c_str(), command, joints_[i].min_position,
        joints_[i].max_position);
      return hardware_interface::return_type::ERROR;
    }
    const double raw_value = static_cast<double>(joints_[i].zero_ticks) +
      command / (joints_[i].direction * joints_[i].radians_per_tick);
    if (raw_value < 0.0 || raw_value > 4095.0) {
      RCLCPP_ERROR(logger_, "%s converted tick is outside [0, 4095]", info_.joints[i].name.c_str());
      return hardware_interface::return_type::ERROR;
    }
    const auto ticks = static_cast<uint32_t>(std::llround(raw_value));
    position_ticks.push_back(ticks);
    command_changed = command_changed || ticks != last_position_ticks_[i];
  }
  if (!command_changed) {
    return hardware_interface::return_type::OK;
  }

  dynamixel::GroupSyncWrite writer(port_handler_, packet_handler_, goal_position_address_, 4);
  std::vector<std::vector<uint8_t>> payloads;
  payloads.reserve(joints_.size());
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    payloads.push_back(make_4byte_payload(position_ticks[i]));
    if (!writer.addParam(joints_[i].id, payloads.back().data())) {
      RCLCPP_ERROR(logger_, "Failed to prepare write for ID %u", joints_[i].id);
      return hardware_interface::return_type::ERROR;
    }
  }
  const int result = writer.txPacket();
  if (result != COMM_SUCCESS) {
    RCLCPP_ERROR(logger_, "Sync write failed: %s", packet_handler_->getTxRxResult(result));
    return hardware_interface::return_type::ERROR;
  }
  last_position_ticks_ = std::move(position_ticks);
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

void DynamixelSystem::close_port()
{
  state_reader_.reset();
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
