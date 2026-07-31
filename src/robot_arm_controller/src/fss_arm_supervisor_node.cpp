#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "controller_manager_msgs/srv/set_hardware_component_state.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "fss_interfaces/msg/system_health.hpp"
#include "fss_interfaces/msg/system_mode.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_arm_controller/trajectory_validator.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;

class FssArmSupervisorNode : public rclcpp::Node
{
public:
  FssArmSupervisorNode()
  : Node("fss_arm_supervisor_node")
  {
    requested_trajectory_topic_ = declare_parameter<std::string>(
      "requested_trajectory_topic", "requested_joint_trajectory");
    controller_trajectory_topic_ = declare_parameter<std::string>(
      "controller_trajectory_topic", "arm_trajectory_controller/joint_trajectory");
    joint_states_topic_ = declare_parameter<std::string>("joint_states_topic", "joint_states");
    mode_topic_ = declare_parameter<std::string>("mode_topic", "/system/mode");
    health_topic_ = declare_parameter<std::string>("health_topic", "/system/health");
    controller_manager_name_ = declare_parameter<std::string>(
      "controller_manager_name", "controller_manager");
    hardware_component_name_ = declare_parameter<std::string>(
      "hardware_component_name", "DynamixelRobotArm");
    trajectory_controller_name_ = declare_parameter<std::string>(
      "trajectory_controller_name", "arm_trajectory_controller");
    joint_state_controller_name_ = declare_parameter<std::string>(
      "joint_state_controller_name", "joint_state_broadcaster");
    mode_timeout_sec_ = declare_parameter<double>("mode_timeout_sec", 2.5);
    health_timeout_sec_ = declare_parameter<double>("health_timeout_sec", 2.5);
    hold_duration_sec_ = declare_parameter<double>("hold_duration_sec", 0.25);
    require_healthy_ = declare_parameter<bool>("require_healthy", true);
    manage_hardware_lifecycle_ = declare_parameter<bool>("manage_hardware_lifecycle", true);
    torque_off_on_mode_timeout_ = declare_parameter<bool>(
      "torque_off_on_mode_timeout", true);

    const auto joint_names = declare_parameter<std::vector<std::string>>(
      "joint_names", std::vector<std::string>{});
    const auto min_positions = declare_parameter<std::vector<double>>(
      "min_position_rad", std::vector<double>{});
    const auto max_positions = declare_parameter<std::vector<double>>(
      "max_position_rad", std::vector<double>{});
    const auto max_velocities = declare_parameter<std::vector<double>>(
      "max_velocity_rad_s", std::vector<double>{});
    const int max_trajectory_points = declare_parameter<int>("max_trajectory_points", 1000);
    if (max_trajectory_points <= 0) {
      throw std::invalid_argument("max_trajectory_points must be positive");
    }
    validation_options_.max_points = static_cast<std::size_t>(max_trajectory_points);
    validation_options_.max_duration_sec = declare_parameter<double>(
      "max_trajectory_duration_sec", 120.0);
    validation_options_.check_average_velocity = declare_parameter<bool>(
      "check_average_velocity", true);

    validate_parameters(joint_names, min_positions, max_positions, max_velocities);
    joint_order_ = joint_names;
    for (std::size_t i = 0; i < joint_names.size(); ++i) {
      joint_limits_.emplace(
        joint_names[i],
        robot_arm_controller::JointLimit{
          min_positions[i], max_positions[i], max_velocities[i]});
    }

    trajectory_publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      controller_trajectory_topic_, 10);
    status_publisher_ = create_publisher<std_msgs::msg::String>("fss_supervisor_status", 10);
    requested_trajectory_subscription_ =
      create_subscription<trajectory_msgs::msg::JointTrajectory>(
      requested_trajectory_topic_, 10,
      std::bind(
        &FssArmSupervisorNode::requested_trajectory_callback, this,
        std::placeholders::_1));
    joint_states_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_, rclcpp::SensorDataQoS(),
      std::bind(&FssArmSupervisorNode::joint_states_callback, this, std::placeholders::_1));
    mode_subscription_ = create_subscription<fss_interfaces::msg::SystemMode>(
      mode_topic_, 10,
      std::bind(&FssArmSupervisorNode::mode_callback, this, std::placeholders::_1));
    health_subscription_ = create_subscription<fss_interfaces::msg::SystemHealth>(
      health_topic_, 10,
      std::bind(&FssArmSupervisorNode::health_callback, this, std::placeholders::_1));

    switch_controller_client_ =
      create_client<controller_manager_msgs::srv::SwitchController>(
      controller_manager_name_ + "/switch_controller");
    hardware_state_client_ =
      create_client<controller_manager_msgs::srv::SetHardwareComponentState>(
      controller_manager_name_ + "/set_hardware_component_state");

    watchdog_timer_ = create_wall_timer(100ms, std::bind(&FssArmSupervisorNode::update_gate, this));
    status_timer_ = create_wall_timer(1s, std::bind(&FssArmSupervisorNode::publish_status, this));

    RCLCPP_INFO(
      get_logger(), "FSS arm gate: '%s' -> '%s'; hardware starts locked until ACTIVE",
      requested_trajectory_topic_.c_str(), controller_trajectory_topic_.c_str());
  }

private:
  enum class SafetyAction {ENABLE, HOLD, TORQUE_OFF};

  static builtin_interfaces::msg::Duration duration_from_seconds(double seconds)
  {
    const auto nanoseconds = static_cast<int64_t>(std::llround(seconds * 1e9));
    builtin_interfaces::msg::Duration duration;
    duration.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
    duration.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
    return duration;
  }

  void validate_parameters(
    const std::vector<std::string> & names,
    const std::vector<double> & minimums,
    const std::vector<double> & maximums,
    const std::vector<double> & velocities)
  {
    if (!std::isfinite(mode_timeout_sec_) || mode_timeout_sec_ <= 0.0 ||
      !std::isfinite(health_timeout_sec_) || health_timeout_sec_ <= 0.0 ||
      !std::isfinite(hold_duration_sec_) || hold_duration_sec_ <= 0.0 ||
      validation_options_.max_points == 0 ||
      !std::isfinite(validation_options_.max_duration_sec) ||
      validation_options_.max_duration_sec <= 0.0)
    {
      throw std::invalid_argument("FSS supervisor durations and limits must be positive");
    }
    if (names.empty() || minimums.size() != names.size() ||
      maximums.size() != names.size() || velocities.size() != names.size())
    {
      throw std::invalid_argument("FSS supervisor joint limit arrays must have matching lengths");
    }
    std::unordered_map<std::string, bool> seen;
    for (std::size_t i = 0; i < names.size(); ++i) {
      if (names[i].empty() || !seen.emplace(names[i], true).second ||
        !std::isfinite(minimums[i]) || !std::isfinite(maximums[i]) ||
        !std::isfinite(velocities[i]) || minimums[i] > maximums[i] ||
        velocities[i] <= 0.0)
      {
        throw std::invalid_argument("invalid FSS supervisor joint limit");
      }
    }
  }

  void requested_trajectory_callback(
    const trajectory_msgs::msg::JointTrajectory::SharedPtr message)
  {
    update_gate();
    if (!commands_enabled_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Robot-arm trajectory rejected: %s",
        block_reason_.c_str());
      return;
    }

    auto validation = robot_arm_controller::validate_and_sanitize_trajectory(
      *message, joint_limits_, validation_options_, latest_joint_positions_);
    if (!validation.valid) {
      RCLCPP_ERROR(
        get_logger(), "Robot-arm trajectory rejected: %s", validation.error.c_str());
      return;
    }
    for (const auto & warning : validation.warnings) {
      // Limit saturation remains joint-local: the rest of the trajectory is preserved.
      RCLCPP_WARN(get_logger(), "%s", warning.c_str());
    }
    trajectory_publisher_->publish(validation.trajectory);
  }

  void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    const auto count = std::min(message->name.size(), message->position.size());
    for (std::size_t i = 0; i < count; ++i) {
      if (joint_limits_.count(message->name[i]) != 0 && std::isfinite(message->position[i])) {
        latest_joint_positions_[message->name[i]] = message->position[i];
      }
    }
  }

  void mode_callback(const fss_interfaces::msg::SystemMode::SharedPtr message)
  {
    mode_received_ = true;
    current_mode_ = message->mode;
    last_mode_time_ = std::chrono::steady_clock::now();
    if (current_mode_ == fss_interfaces::msg::SystemMode::EMERGENCY_STOP) {
      estop_latched_ = true;
    }
    update_gate();
  }

  void health_callback(const fss_interfaces::msg::SystemHealth::SharedPtr message)
  {
    health_received_ = true;
    fss_healthy_ =
      message->navigation_ok && message->actuators_ok && message->errors.empty();
    last_health_time_ = std::chrono::steady_clock::now();
    update_gate();
  }

  void update_gate()
  {
    const auto now_steady = std::chrono::steady_clock::now();
    std::string reason;
    SafetyAction requested_action = SafetyAction::HOLD;

    if (!mode_received_) {
      reason = "waiting for /system/mode";
      requested_action = SafetyAction::TORQUE_OFF;
    } else if (
      std::chrono::duration<double>(now_steady - last_mode_time_).count() > mode_timeout_sec_)
    {
      reason = "/system/mode timeout";
      requested_action = torque_off_on_mode_timeout_ ?
        SafetyAction::TORQUE_OFF : SafetyAction::HOLD;
    } else if (current_mode_ == fss_interfaces::msg::SystemMode::EMERGENCY_STOP) {
      reason = "FSS EMERGENCY_STOP";
      requested_action = SafetyAction::TORQUE_OFF;
    } else if (current_mode_ != fss_interfaces::msg::SystemMode::ACTIVE) {
      reason = "FSS mode is not ACTIVE";
    } else if (require_healthy_ && !health_received_) {
      reason = "waiting for /system/health";
    } else if (
      require_healthy_ &&
      std::chrono::duration<double>(now_steady - last_health_time_).count() > health_timeout_sec_)
    {
      reason = "/system/health timeout";
    } else if (require_healthy_ && !fss_healthy_) {
      reason = "FSS health is not OK";
    } else {
      requested_action = SafetyAction::ENABLE;
      estop_latched_ = false;
    }

    if (estop_latched_ && requested_action != SafetyAction::ENABLE) {
      requested_action = SafetyAction::TORQUE_OFF;
      if (reason.empty()) {
        reason = "EMERGENCY_STOP remains latched until healthy ACTIVE mode";
      }
    }

    desired_action_ = requested_action;
    block_reason_ = requested_action == SafetyAction::ENABLE ? "" : std::move(reason);
    commands_enabled_ = requested_action == SafetyAction::ENABLE &&
      (!manage_hardware_lifecycle_ || hardware_active_) && !transition_in_progress_;

    if (requested_action == SafetyAction::HOLD && hardware_active_) {
      publish_hold_trajectory();
    } else if (requested_action != SafetyAction::HOLD) {
      hold_sent_for_block_ = false;
    }
    drive_lifecycle_transition();
  }

  void drive_lifecycle_transition()
  {
    if (!manage_hardware_lifecycle_ || transition_in_progress_) {
      return;
    }
    if (desired_action_ == SafetyAction::TORQUE_OFF && hardware_active_) {
      request_controllers(false);
    } else if (desired_action_ == SafetyAction::ENABLE && !hardware_active_) {
      request_hardware_state(true);
    } else if (desired_action_ == SafetyAction::ENABLE && hardware_active_ &&
      !controllers_active_)
    {
      request_controllers(true);
    }
  }

  void request_controllers(bool activate)
  {
    if (!switch_controller_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Waiting for controller-manager switch service");
      return;
    }
    transition_in_progress_ = true;
    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    auto & targets = activate ?
      request->activate_controllers : request->deactivate_controllers;
    targets = {joint_state_controller_name_, trajectory_controller_name_};
    request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
    request->activate_asap = true;
    request->timeout = duration_from_seconds(2.0);
    switch_controller_client_->async_send_request(
      request,
      [this, activate](
        rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future)
      {
        transition_in_progress_ = false;
        if (!future.get()->ok) {
          RCLCPP_ERROR(
            get_logger(), "Failed to %s robot-arm controllers",
            activate ? "activate" : "deactivate");
          return;
        }
        controllers_active_ = activate;
        if (!activate && desired_action_ == SafetyAction::TORQUE_OFF) {
          request_hardware_state(false);
        } else {
          commands_enabled_ = activate && desired_action_ == SafetyAction::ENABLE;
          drive_lifecycle_transition();
        }
      });
  }

  void request_hardware_state(bool activate)
  {
    if (!hardware_state_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Waiting for controller-manager hardware-state service");
      return;
    }
    transition_in_progress_ = true;
    auto request =
      std::make_shared<controller_manager_msgs::srv::SetHardwareComponentState::Request>();
    request->name = hardware_component_name_;
    request->target_state.id = activate ?
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE :
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
    request->target_state.label = activate ? "active" : "inactive";
    hardware_state_client_->async_send_request(
      request,
      [this, activate](
        rclcpp::Client<
          controller_manager_msgs::srv::SetHardwareComponentState>::SharedFuture future)
      {
        transition_in_progress_ = false;
        if (!future.get()->ok) {
          RCLCPP_ERROR(
            get_logger(), "Failed to set robot-arm hardware %s",
            activate ? "active" : "inactive");
          return;
        }
        hardware_active_ = activate;
        if (!activate) {
          commands_enabled_ = false;
          RCLCPP_ERROR(get_logger(), "Robot-arm motor torque disabled by FSS safety policy");
        }
        drive_lifecycle_transition();
      });
  }

  void publish_hold_trajectory()
  {
    if (hold_sent_for_block_) {
      return;
    }
    trajectory_msgs::msg::JointTrajectory hold;
    hold.header.stamp = now();
    trajectory_msgs::msg::JointTrajectoryPoint point;
    for (const auto & joint_name : joint_order_) {
      const auto found = latest_joint_positions_.find(joint_name);
      if (found == latest_joint_positions_.end()) {
        continue;
      }
      hold.joint_names.push_back(joint_name);
      point.positions.push_back(found->second);
      point.velocities.push_back(0.0);
    }
    if (hold.joint_names.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cannot publish hold trajectory before joint states are received");
      return;
    }
    point.time_from_start = duration_from_seconds(hold_duration_sec_);
    hold.points.push_back(std::move(point));
    trajectory_publisher_->publish(hold);
    hold_sent_for_block_ = true;
    RCLCPP_WARN(
      get_logger(), "Holding %zu robot-arm joints: %s",
      hold.joint_names.size(), block_reason_.c_str());
  }

  void publish_status()
  {
    std_msgs::msg::String status;
    if (commands_enabled_) {
      status.data = "COMMANDS_ENABLED";
    } else if (transition_in_progress_) {
      status.data = "SAFETY_TRANSITION: " + block_reason_;
    } else if (!hardware_active_) {
      status.data = "TORQUE_OFF: " + block_reason_;
    } else {
      status.data = "HOLD: " + block_reason_;
    }
    status_publisher_->publish(status);
  }

  std::string requested_trajectory_topic_;
  std::string controller_trajectory_topic_;
  std::string joint_states_topic_;
  std::string mode_topic_;
  std::string health_topic_;
  std::string controller_manager_name_;
  std::string hardware_component_name_;
  std::string trajectory_controller_name_;
  std::string joint_state_controller_name_;
  double mode_timeout_sec_{2.5};
  double health_timeout_sec_{2.5};
  double hold_duration_sec_{0.25};
  bool require_healthy_{true};
  bool manage_hardware_lifecycle_{true};
  bool torque_off_on_mode_timeout_{true};

  bool mode_received_{false};
  bool health_received_{false};
  bool fss_healthy_{false};
  bool commands_enabled_{false};
  bool hold_sent_for_block_{false};
  bool hardware_active_{false};
  bool controllers_active_{false};
  bool transition_in_progress_{false};
  bool estop_latched_{false};
  uint8_t current_mode_{fss_interfaces::msg::SystemMode::IDLE};
  SafetyAction desired_action_{SafetyAction::TORQUE_OFF};
  std::string block_reason_{"waiting for /system/mode"};
  std::chrono::steady_clock::time_point last_mode_time_{};
  std::chrono::steady_clock::time_point last_health_time_{};
  std::vector<std::string> joint_order_;
  std::unordered_map<std::string, robot_arm_controller::JointLimit> joint_limits_;
  std::unordered_map<std::string, double> latest_joint_positions_;
  robot_arm_controller::TrajectoryValidationOptions validation_options_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr
    requested_trajectory_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscription_;
  rclcpp::Subscription<fss_interfaces::msg::SystemMode>::SharedPtr mode_subscription_;
  rclcpp::Subscription<fss_interfaces::msg::SystemHealth>::SharedPtr health_subscription_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr
    switch_controller_client_;
  rclcpp::Client<controller_manager_msgs::srv::SetHardwareComponentState>::SharedPtr
    hardware_state_client_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<FssArmSupervisorNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("fss_arm_supervisor_node"), "%s", exception.what());
  }
  rclcpp::shutdown();
  return 0;
}
