#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class DynamixelDiagnosticsNode : public rclcpp::Node
{
public:
  DynamixelDiagnosticsNode()
  : Node("dynamixel_diagnostics_node")
  {
    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names", std::vector<std::string>{});
    stale_timeout_sec_ = declare_parameter<double>("stale_timeout_sec", 2.0);
    warning_temperature_c_ = declare_parameter<double>("warning_temperature_c", 65.0);
    error_temperature_c_ = declare_parameter<double>("error_temperature_c", 75.0);
    min_voltage_v_ = declare_parameter<double>("min_voltage_v", 9.5);
    max_voltage_v_ = declare_parameter<double>("max_voltage_v", 16.0);
    if (joint_names_.empty() || !std::isfinite(stale_timeout_sec_) ||
      stale_timeout_sec_ <= 0.0 || warning_temperature_c_ >= error_temperature_c_ ||
      min_voltage_v_ >= max_voltage_v_)
    {
      throw std::invalid_argument("invalid Dynamixel diagnostic parameters");
    }

    diagnostics_publisher_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    state_subscription_ = create_subscription<control_msgs::msg::DynamicJointState>(
      "dynamic_joint_states", rclcpp::SensorDataQoS(),
      std::bind(&DynamixelDiagnosticsNode::state_callback, this, std::placeholders::_1));
    timer_ = create_wall_timer(1s, std::bind(&DynamixelDiagnosticsNode::publish, this));
  }

private:
  struct JointDiagnostic
  {
    std::unordered_map<std::string, double> values;
    std::chrono::steady_clock::time_point received;
  };

  static std::string number(double value, int precision = 3)
  {
    if (!std::isfinite(value)) {
      return "unavailable";
    }
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(precision) << value;
    return stream.str();
  }

  static void add_value(
    diagnostic_msgs::msg::DiagnosticStatus & status,
    const std::string & key, const std::string & value)
  {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = value;
    status.values.push_back(std::move(item));
  }

  void state_callback(const control_msgs::msg::DynamicJointState::SharedPtr message)
  {
    const auto count = std::min(message->joint_names.size(), message->interface_values.size());
    const auto received = std::chrono::steady_clock::now();
    for (std::size_t i = 0; i < count; ++i) {
      auto & diagnostic = states_[message->joint_names[i]];
      diagnostic.received = received;
      const auto & interfaces = message->interface_values[i];
      const auto interface_count = std::min(
        interfaces.interface_names.size(), interfaces.values.size());
      for (std::size_t j = 0; j < interface_count; ++j) {
        diagnostic.values[interfaces.interface_names[j]] = interfaces.values[j];
      }
    }
  }

  double value_or_nan(const JointDiagnostic & state, const std::string & name) const
  {
    const auto found = state.values.find(name);
    return found == state.values.end() ?
           std::numeric_limits<double>::quiet_NaN() : found->second;
  }

  void publish()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    const auto now_steady = std::chrono::steady_clock::now();

    for (const auto & joint_name : joint_names_) {
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = "robot_arm/Dynamixel/" + joint_name;
      status.hardware_id = joint_name;
      const auto found = states_.find(joint_name);
      if (found == states_.end() ||
        std::chrono::duration<double>(now_steady - found->second.received).count() >
        stale_timeout_sec_)
      {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
        status.message = "No recent Dynamixel diagnostic state";
        array.status.push_back(std::move(status));
        continue;
      }

      const auto & state = found->second;
      const double current = value_or_nan(state, "current");
      const double voltage = value_or_nan(state, "voltage");
      const double temperature = value_or_nan(state, "temperature");
      const double hardware_error = value_or_nan(state, "hardware_error");
      const double bus_watchdog = value_or_nan(state, "bus_watchdog");
      const double communication_ok = value_or_nan(state, "communication_ok");
      add_value(status, "current_A", number(current));
      add_value(status, "voltage_V", number(voltage));
      add_value(status, "temperature_C", number(temperature, 1));
      add_value(status, "hardware_error", number(hardware_error, 0));
      add_value(status, "bus_watchdog", number(bus_watchdog, 0));
      add_value(status, "communication_ok", number(communication_ok, 0));

      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "Dynamixel state is healthy";
      if (!std::isfinite(hardware_error) || !std::isfinite(bus_watchdog) ||
        !std::isfinite(communication_ok))
      {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
        status.message = "Diagnostic interfaces are not initialized";
      } else if (hardware_error != 0.0) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status.message = "Dynamixel Hardware Error Status is nonzero";
      } else if (bus_watchdog < 0.0) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status.message = "Dynamixel Bus Watchdog has stopped the motor";
      } else if (communication_ok < 0.5) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status.message = "Dynamixel communication is unavailable";
      } else if (std::isfinite(temperature) && temperature >= error_temperature_c_) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status.message = "Dynamixel temperature is above the error threshold";
      } else if (std::isfinite(temperature) && temperature >= warning_temperature_c_) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "Dynamixel temperature is high";
      } else if (std::isfinite(voltage) &&
        (voltage < min_voltage_v_ || voltage > max_voltage_v_))
      {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "Dynamixel input voltage is outside the configured range";
      }
      array.status.push_back(std::move(status));
    }
    diagnostics_publisher_->publish(array);
  }

  std::vector<std::string> joint_names_;
  double stale_timeout_sec_{2.0};
  double warning_temperature_c_{65.0};
  double error_temperature_c_{75.0};
  double min_voltage_v_{9.5};
  double max_voltage_v_{16.0};
  std::unordered_map<std::string, JointDiagnostic> states_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr state_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<DynamixelDiagnosticsNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("dynamixel_diagnostics_node"), "%s", exception.what());
  }
  rclcpp::shutdown();
  return 0;
}
