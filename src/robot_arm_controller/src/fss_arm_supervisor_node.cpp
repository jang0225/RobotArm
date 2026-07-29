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

#include "fss_interfaces/msg/system_health.hpp"
#include "fss_interfaces/msg/system_mode.hpp"
#include "rclcpp/rclcpp.hpp"
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
    mode_timeout_sec_ = declare_parameter<double>("mode_timeout_sec", 2.5);
    health_timeout_sec_ = declare_parameter<double>("health_timeout_sec", 2.5);
    hold_duration_sec_ = declare_parameter<double>("hold_duration_sec", 0.25);
    require_healthy_ = declare_parameter<bool>("require_healthy", true);

    if (!std::isfinite(mode_timeout_sec_) || mode_timeout_sec_ <= 0.0 ||
      !std::isfinite(health_timeout_sec_) || health_timeout_sec_ <= 0.0 ||
      !std::isfinite(hold_duration_sec_) || hold_duration_sec_ <= 0.0)
    {
      throw std::invalid_argument("FSS supervisor timeout and hold durations must be positive");
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

    watchdog_timer_ = create_wall_timer(100ms, std::bind(&FssArmSupervisorNode::update_gate, this));
    status_timer_ = create_wall_timer(1s, std::bind(&FssArmSupervisorNode::publish_status, this));

    RCLCPP_INFO(
      get_logger(), "FSS arm gate: '%s' -> '%s'; waiting for ACTIVE and healthy FSS state",
      requested_trajectory_topic_.c_str(), controller_trajectory_topic_.c_str());
  }

private:
  static builtin_interfaces::msg::Duration duration_from_seconds(double seconds)
  {
    const auto nanoseconds = static_cast<int64_t>(std::llround(seconds * 1e9));
    builtin_interfaces::msg::Duration duration;
    duration.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
    duration.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
    return duration;
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
    if (message->joint_names.empty() || message->points.empty()) {
      RCLCPP_ERROR(get_logger(), "Robot-arm trajectory rejected: empty joint or point array");
      return;
    }
    trajectory_publisher_->publish(*message);
  }

  void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    const auto count = std::min(message->name.size(), message->position.size());
    for (std::size_t i = 0; i < count; ++i) {
      if (std::isfinite(message->position[i])) {
        latest_joint_positions_[message->name[i]] = message->position[i];
      }
    }
  }

  void mode_callback(const fss_interfaces::msg::SystemMode::SharedPtr message)
  {
    mode_received_ = true;
    current_mode_ = message->mode;
    last_mode_time_ = now();
    update_gate();
  }

  void health_callback(const fss_interfaces::msg::SystemHealth::SharedPtr message)
  {
    health_received_ = true;
    fss_healthy_ =
      message->navigation_ok && message->actuators_ok && message->errors.empty();
    last_health_time_ = now();
    update_gate();
  }

  void update_gate()
  {
    const auto current_time = now();
    std::string reason;

    if (!mode_received_) {
      reason = "waiting for /system/mode";
    } else if ((current_time - last_mode_time_).seconds() > mode_timeout_sec_) {
      reason = "/system/mode timeout";
    } else if (current_mode_ != fss_interfaces::msg::SystemMode::ACTIVE) {
      reason = "FSS mode is not ACTIVE";
    } else if (require_healthy_ && !health_received_) {
      reason = "waiting for /system/health";
    } else if (
      require_healthy_ && (current_time - last_health_time_).seconds() > health_timeout_sec_)
    {
      reason = "/system/health timeout";
    } else if (require_healthy_ && !fss_healthy_) {
      reason = "FSS health is not OK";
    }

    const bool enable = reason.empty();
    if (enable == commands_enabled_) {
      block_reason_ = enable ? "" : std::move(reason);
      if (!enable) {
        publish_hold_trajectory();
      }
      return;
    }

    commands_enabled_ = enable;
    block_reason_ = enable ? "" : std::move(reason);
    if (commands_enabled_) {
      hold_sent_for_block_ = false;
      RCLCPP_INFO(get_logger(), "Robot-arm trajectory commands enabled");
      return;
    }

    RCLCPP_WARN(get_logger(), "Robot-arm trajectory commands blocked: %s", block_reason_.c_str());
    publish_hold_trajectory();
  }

  void publish_hold_trajectory()
  {
    if (hold_sent_for_block_) {
      return;
    }

    static const std::vector<std::string> joint_order{
      "joint1", "joint2", "joint3", "gripper_joint"};
    trajectory_msgs::msg::JointTrajectory hold;
    hold.header.stamp = now();
    trajectory_msgs::msg::JointTrajectoryPoint point;

    for (const auto & joint_name : joint_order) {
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
      get_logger(), "Holding %zu robot-arm joints at their latest measured positions",
      hold.joint_names.size());
  }

  void publish_status()
  {
    std_msgs::msg::String status;
    status.data = commands_enabled_ ? "COMMANDS_ENABLED" : "BLOCKED: " + block_reason_;
    status_publisher_->publish(status);
  }

  std::string requested_trajectory_topic_;
  std::string controller_trajectory_topic_;
  std::string joint_states_topic_;
  std::string mode_topic_;
  std::string health_topic_;
  double mode_timeout_sec_;
  double health_timeout_sec_;
  double hold_duration_sec_;
  bool require_healthy_;

  bool mode_received_{false};
  bool health_received_{false};
  bool fss_healthy_{false};
  bool commands_enabled_{false};
  bool hold_sent_for_block_{false};
  uint8_t current_mode_{fss_interfaces::msg::SystemMode::IDLE};
  std::string block_reason_{"waiting for /system/mode"};
  rclcpp::Time last_mode_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_health_time_{0, 0, RCL_ROS_TIME};
  std::unordered_map<std::string, double> latest_joint_positions_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr
    requested_trajectory_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscription_;
  rclcpp::Subscription<fss_interfaces::msg::SystemMode>::SharedPtr mode_subscription_;
  rclcpp::Subscription<fss_interfaces::msg::SystemHealth>::SharedPtr health_subscription_;
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
