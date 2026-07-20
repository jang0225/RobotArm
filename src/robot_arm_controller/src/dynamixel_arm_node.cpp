#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <unordered_map>
#include <utility>
#include <vector>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

class DynamixelArmNode : public rclcpp::Node
{
public:
  DynamixelArmNode()
  : Node("dynamixel_arm_controller")
  {
    declare_parameters();
    load_parameters();
    validate_parameters();

    port_handler_ = dynamixel::PortHandler::getPortHandler(device_name_.c_str());
    packet_handler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version_);

    if (!port_handler_->openPort()) {
      throw std::runtime_error("Dynamixel port를 열 수 없습니다: " + device_name_);
    }
    if (!port_handler_->setBaudRate(baud_rate_)) {
      port_handler_->closePort();
      throw std::runtime_error("baud rate를 설정할 수 없습니다: " + std::to_string(baud_rate_));
    }

    for (std::size_t i = 0; i < motor_ids_.size(); ++i) {
      uint16_t model_number = 0;
      uint8_t error = 0;
      const int result = packet_handler_->ping(
        port_handler_, static_cast<uint8_t>(motor_ids_[i]), &model_number, &error);
      if (result != COMM_SUCCESS || error != 0) {
        close_port();
        throw std::runtime_error(
                "모터 ping 실패: " + joint_names_[i] + " (ID " +
                std::to_string(motor_ids_[i]) + ")");
      }
      if (model_number != expected_model_numbers_[i]) {
        close_port();
        throw std::runtime_error(
                "모터 모델 불일치: ID " + std::to_string(motor_ids_[i]) +
                ", expected=" + std::to_string(expected_model_numbers_[i]) +
                ", actual=" + std::to_string(model_number));
      }

      uint8_t operating_mode = 0;
      error = 0;
      const int mode_result = packet_handler_->read1ByteTxRx(
        port_handler_, static_cast<uint8_t>(motor_ids_[i]),
        static_cast<uint16_t>(operating_mode_address_), &operating_mode, &error);
      if (mode_result != COMM_SUCCESS || error != 0) {
        close_port();
        throw std::runtime_error(
                "Operating Mode 읽기 실패: ID " + std::to_string(motor_ids_[i]));
      }
      if (operating_mode != expected_operating_modes_[i]) {
        close_port();
        throw std::runtime_error(
                "Operating Mode 불일치: ID " + std::to_string(motor_ids_[i]) +
                ", expected=" + std::to_string(expected_operating_modes_[i]) +
                ", actual=" + std::to_string(operating_mode));
      }
      RCLCPP_INFO(
        get_logger(), "%s: ID=%ld, model=%u, mode=%u 연결됨",
        joint_names_[i].c_str(), motor_ids_[i], model_number, operating_mode);
    }

    state_publisher_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    command_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_commands", 10,
      std::bind(&DynamixelArmNode::command_callback, this, std::placeholders::_1));
    torque_service_ = create_service<std_srvs::srv::SetBool>(
      "set_torque",
      std::bind(
        &DynamixelArmNode::torque_callback, this, std::placeholders::_1,
        std::placeholders::_2));

    if (auto_enable_torque_ && !set_all_torque(true)) {
      close_port();
      throw std::runtime_error("초기 torque 활성화에 실패했습니다.");
    }

    const auto period = std::chrono::duration<double>(1.0 / state_publish_rate_hz_);
    state_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&DynamixelArmNode::publish_state, this));

    RCLCPP_INFO(
      get_logger(), "준비됨. command='joint_commands', state='joint_states', torque='set_torque'");
  }

  ~DynamixelArmNode() override
  {
    std::lock_guard<std::mutex> lock(bus_mutex_);
    if (port_handler_ == nullptr) {
      return;
    }
    if (disable_torque_on_shutdown_ && torque_enabled_) {
      write_torque_unlocked(false);
    }
    close_port();
  }

private:
  void declare_parameters()
  {
    declare_parameter<std::string>("device_name", "/dev/ttyUSB0");
    declare_parameter<int>("baud_rate", 1000000);
    declare_parameter<double>("protocol_version", 2.0);
    declare_parameter<std::vector<std::string>>(
      "joint_names", {"joint1", "joint2", "gripper_joint"});
    declare_parameter<std::vector<int64_t>>("motor_ids", {1, 2, 3});
    declare_parameter<std::vector<int64_t>>("expected_model_numbers", {1001, 1001, 1020});
    declare_parameter<std::vector<int64_t>>("expected_operating_modes", {3, 3, 3});
    declare_parameter<std::vector<double>>("direction", {1.0, 1.0, 1.0});
    declare_parameter<std::vector<int64_t>>("zero_position_ticks", {2048, 2048, 2048});
    declare_parameter<std::vector<double>>(
      "radians_per_tick", {0.001533981, 0.001533981, 0.001533981});
    declare_parameter<std::vector<double>>("min_position_rad", {-3.14, -3.14, -0.25});
    declare_parameter<std::vector<double>>("max_position_rad", {3.14, 3.14, 0.25});
    declare_parameter<int>("torque_enable_address", 64);
    declare_parameter<int>("operating_mode_address", 11);
    declare_parameter<int>("goal_position_address", 116);
    declare_parameter<int>("present_position_address", 132);
    declare_parameter<int>("position_data_length", 4);
    declare_parameter<bool>("auto_enable_torque", false);
    declare_parameter<bool>("disable_torque_on_shutdown", true);
    declare_parameter<double>("state_publish_rate_hz", 20.0);
  }

  void load_parameters()
  {
    device_name_ = get_parameter("device_name").as_string();
    baud_rate_ = get_parameter("baud_rate").as_int();
    protocol_version_ = get_parameter("protocol_version").as_double();
    joint_names_ = get_parameter("joint_names").as_string_array();
    motor_ids_ = get_parameter("motor_ids").as_integer_array();
    expected_model_numbers_ = get_parameter("expected_model_numbers").as_integer_array();
    expected_operating_modes_ = get_parameter("expected_operating_modes").as_integer_array();
    directions_ = get_parameter("direction").as_double_array();
    zero_ticks_ = get_parameter("zero_position_ticks").as_integer_array();
    radians_per_tick_ = get_parameter("radians_per_tick").as_double_array();
    min_positions_ = get_parameter("min_position_rad").as_double_array();
    max_positions_ = get_parameter("max_position_rad").as_double_array();
    torque_address_ = get_parameter("torque_enable_address").as_int();
    operating_mode_address_ = get_parameter("operating_mode_address").as_int();
    goal_position_address_ = get_parameter("goal_position_address").as_int();
    present_position_address_ = get_parameter("present_position_address").as_int();
    position_data_length_ = get_parameter("position_data_length").as_int();
    auto_enable_torque_ = get_parameter("auto_enable_torque").as_bool();
    disable_torque_on_shutdown_ = get_parameter("disable_torque_on_shutdown").as_bool();
    state_publish_rate_hz_ = get_parameter("state_publish_rate_hz").as_double();

    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
      joint_index_.emplace(joint_names_[i], i);
    }
  }

  void validate_parameters() const
  {
    const auto size = joint_names_.size();
    if (size == 0 || motor_ids_.size() != size || expected_model_numbers_.size() != size ||
      expected_operating_modes_.size() != size || directions_.size() != size ||
      zero_ticks_.size() != size || radians_per_tick_.size() != size ||
      min_positions_.size() != size || max_positions_.size() != size)
    {
      throw std::invalid_argument("모든 joint별 parameter 배열의 길이가 같아야 합니다.");
    }
    if (position_data_length_ != 2 && position_data_length_ != 4) {
      throw std::invalid_argument("position_data_length는 2 또는 4여야 합니다.");
    }
    if (state_publish_rate_hz_ <= 0.0) {
      throw std::invalid_argument("state_publish_rate_hz는 0보다 커야 합니다.");
    }
    for (std::size_t i = 0; i < size; ++i) {
      if (motor_ids_[i] < 0 || motor_ids_[i] > 252 || directions_[i] == 0.0 ||
        expected_model_numbers_[i] <= 0 || expected_model_numbers_[i] > 65535 ||
        expected_operating_modes_[i] < 0 || expected_operating_modes_[i] > 255 ||
        radians_per_tick_[i] <= 0.0 || min_positions_[i] > max_positions_[i])
      {
        throw std::invalid_argument("잘못된 joint parameter: " + joint_names_[i]);
      }
    }
    const std::unordered_set<std::string> unique_names(joint_names_.begin(), joint_names_.end());
    const std::unordered_set<int64_t> unique_ids(motor_ids_.begin(), motor_ids_.end());
    if (unique_names.size() != size || unique_ids.size() != size) {
      throw std::invalid_argument("joint 이름과 motor ID는 각각 중복될 수 없습니다.");
    }
  }

  void command_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->name.size() != msg->position.size()) {
      RCLCPP_WARN(get_logger(), "명령의 name과 position 배열 길이가 다릅니다.");
      return;
    }
    if (!torque_enabled_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "torque가 꺼져 있어 위치 명령을 무시합니다.");
      return;
    }

    std::vector<std::pair<std::size_t, uint32_t>> goals;
    goals.reserve(msg->name.size());
    std::unordered_set<std::size_t> commanded_joints;

    for (std::size_t i = 0; i < msg->name.size(); ++i) {
      const auto found = joint_index_.find(msg->name[i]);
      if (found == joint_index_.end()) {
        RCLCPP_ERROR(get_logger(), "알 수 없는 joint로 인해 명령을 거부합니다: %s", msg->name[i].c_str());
        return;
      }
      const std::size_t joint = found->second;
      if (!commanded_joints.insert(joint).second) {
        RCLCPP_ERROR(get_logger(), "중복된 joint로 인해 명령을 거부합니다: %s", msg->name[i].c_str());
        return;
      }
      if (!std::isfinite(msg->position[i]) || msg->position[i] < min_positions_[joint] ||
        msg->position[i] > max_positions_[joint])
      {
        RCLCPP_ERROR(
          get_logger(), "%s 명령 %.3f rad가 제한 [%.3f, %.3f] 밖입니다.",
          joint_names_[joint].c_str(), msg->position[i], min_positions_[joint],
          max_positions_[joint]);
        return;
      }

      const double raw_value = static_cast<double>(zero_ticks_[joint]) +
        msg->position[i] / (directions_[joint] * radians_per_tick_[joint]);
      const double raw_max = position_data_length_ == 2 ?
        static_cast<double>(std::numeric_limits<uint16_t>::max()) :
        static_cast<double>(std::numeric_limits<uint32_t>::max());
      if (raw_value < 0.0 || raw_value > raw_max) {
        RCLCPP_ERROR(get_logger(), "%s의 변환된 tick 값이 유효하지 않습니다.", msg->name[i].c_str());
        return;
      }
      const auto raw = static_cast<uint32_t>(std::llround(raw_value));
      goals.emplace_back(joint, raw);
    }

    if (goals.empty()) {
      return;
    }

    dynamixel::GroupSyncWrite writer(
      port_handler_, packet_handler_, goal_position_address_, position_data_length_);
    std::vector<std::vector<uint8_t>> payloads;
    payloads.reserve(goals.size());
    for (const auto & [joint, raw] : goals) {
      payloads.push_back(make_payload(raw));
      if (!writer.addParam(
          static_cast<uint8_t>(motor_ids_[joint]), payloads.back().data()))
      {
        RCLCPP_ERROR(
          get_logger(), "%s sync-write parameter 추가 실패", joint_names_[joint].c_str());
        return;
      }
    }

    std::lock_guard<std::mutex> lock(bus_mutex_);
    const int result = writer.txPacket();
    if (result != COMM_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "위치 명령 전송 실패: %s", packet_handler_->getTxRxResult(result));
    }
  }

  std::vector<uint8_t> make_payload(uint32_t value) const
  {
    if (position_data_length_ == 2) {
      return {DXL_LOBYTE(value), DXL_HIBYTE(value)};
    }
    return {
      DXL_LOBYTE(DXL_LOWORD(value)), DXL_HIBYTE(DXL_LOWORD(value)),
      DXL_LOBYTE(DXL_HIWORD(value)), DXL_HIBYTE(DXL_HIWORD(value))};
  }

  void publish_state()
  {
    sensor_msgs::msg::JointState state;
    state.header.stamp = now();
    state.name = joint_names_;
    state.position.resize(joint_names_.size(), std::numeric_limits<double>::quiet_NaN());

    std::lock_guard<std::mutex> lock(bus_mutex_);
    for (std::size_t i = 0; i < motor_ids_.size(); ++i) {
      uint8_t error = 0;
      int result = COMM_TX_FAIL;
      uint32_t raw = 0;
      if (position_data_length_ == 2) {
        uint16_t raw16 = 0;
        result = packet_handler_->read2ByteTxRx(
          port_handler_, static_cast<uint8_t>(motor_ids_[i]),
          static_cast<uint16_t>(present_position_address_), &raw16, &error);
        raw = raw16;
      } else {
        result = packet_handler_->read4ByteTxRx(
          port_handler_, static_cast<uint8_t>(motor_ids_[i]),
          static_cast<uint16_t>(present_position_address_), &raw, &error);
      }
      if (result != COMM_SUCCESS || error != 0) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "%s 위치 읽기 실패", joint_names_[i].c_str());
        continue;
      }
      state.position[i] = directions_[i] *
        (static_cast<double>(raw) - static_cast<double>(zero_ticks_[i])) * radians_per_tick_[i];
    }
    state_publisher_->publish(state);
  }

  void torque_callback(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    response->success = set_all_torque(request->data);
    response->message = response->success ?
      (request->data ? "torque enabled" : "torque disabled") : "torque command failed";
  }

  bool set_all_torque(bool enabled)
  {
    std::lock_guard<std::mutex> lock(bus_mutex_);
    const bool success = write_torque_unlocked(enabled);
    if (enabled && !success) {
      RCLCPP_ERROR(get_logger(), "부분 활성화를 방지하기 위해 전체 torque 해제를 시도합니다.");
      write_torque_unlocked(false);
      torque_enabled_ = false;
    }
    return success;
  }

  bool write_torque_unlocked(bool enabled)
  {
    bool success = true;
    for (const auto id : motor_ids_) {
      uint8_t error = 0;
      const int result = packet_handler_->write1ByteTxRx(
        port_handler_, static_cast<uint8_t>(id), static_cast<uint16_t>(torque_address_),
        enabled ? 1 : 0, &error);
      if (result != COMM_SUCCESS || error != 0) {
        RCLCPP_ERROR(get_logger(), "ID %ld torque 변경 실패", id);
        success = false;
      }
    }
    if (success) {
      torque_enabled_ = enabled;
      RCLCPP_INFO(get_logger(), "torque %s", enabled ? "enabled" : "disabled");
    }
    return success;
  }

  void close_port()
  {
    if (port_handler_ != nullptr) {
      port_handler_->closePort();
      port_handler_ = nullptr;
    }
  }

  std::string device_name_;
  int64_t baud_rate_{1000000};
  double protocol_version_{2.0};
  std::vector<std::string> joint_names_;
  std::vector<int64_t> motor_ids_;
  std::vector<int64_t> expected_model_numbers_;
  std::vector<int64_t> expected_operating_modes_;
  std::vector<double> directions_;
  std::vector<int64_t> zero_ticks_;
  std::vector<double> radians_per_tick_;
  std::vector<double> min_positions_;
  std::vector<double> max_positions_;
  std::unordered_map<std::string, std::size_t> joint_index_;
  int64_t torque_address_{64};
  int64_t operating_mode_address_{11};
  int64_t goal_position_address_{116};
  int64_t present_position_address_{132};
  int64_t position_data_length_{4};
  bool auto_enable_torque_{false};
  bool disable_torque_on_shutdown_{true};
  bool torque_enabled_{false};
  double state_publish_rate_hz_{20.0};

  dynamixel::PortHandler * port_handler_{nullptr};
  dynamixel::PacketHandler * packet_handler_{nullptr};
  std::mutex bus_mutex_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_subscription_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr torque_service_;
  rclcpp::TimerBase::SharedPtr state_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<DynamixelArmNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("dynamixel_arm_node"), "%s", exception.what());
  }
  rclcpp::shutdown();
  return 0;
}
