#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robot_arm_controller/angle_utils.hpp"
#include "robot_arm_controller/msg/joint_command_degrees.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

class DegreeTrajectoryBridge : public rclcpp::Node
{
public:
  DegreeTrajectoryBridge()
  : Node("degree_trajectory_bridge")
  {
    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names", std::vector<std::string>{});
    min_positions_deg_ = declare_parameter<std::vector<double>>(
      "min_position_deg", std::vector<double>{});
    max_positions_deg_ = declare_parameter<std::vector<double>>(
      "max_position_deg", std::vector<double>{});
    controller_min_positions_deg_ = declare_parameter<std::vector<double>>(
      "controller_min_position_deg", std::vector<double>{});
    controller_max_positions_deg_ = declare_parameter<std::vector<double>>(
      "controller_max_position_deg", std::vector<double>{});
    command_origins_deg_ = declare_parameter<std::vector<double>>(
      "command_origin_deg", std::vector<double>{});
    command_directions_ = declare_parameter<std::vector<double>>(
      "command_direction", std::vector<double>{});
    max_duration_sec_ = declare_parameter<double>("max_duration_sec", 120.0);
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/arm_trajectory_controller/joint_trajectory");

    if (joint_names_.empty() || min_positions_deg_.size() != joint_names_.size() ||
      max_positions_deg_.size() != joint_names_.size())
    {
      throw std::invalid_argument(
              "degree bridge requires matching joint_names/min_position_deg/max_position_deg; "
              "start it through robot_arm_bringup");
    }
    if (!std::isfinite(max_duration_sec_) || max_duration_sec_ <= 0.0) {
      throw std::invalid_argument("max_duration_sec must be positive and finite");
    }
    if (controller_min_positions_deg_.empty()) {
      controller_min_positions_deg_ = min_positions_deg_;
    }
    if (controller_max_positions_deg_.empty()) {
      controller_max_positions_deg_ = max_positions_deg_;
    }
    if (command_origins_deg_.empty()) {
      command_origins_deg_.assign(joint_names_.size(), 0.0);
    }
    if (command_directions_.empty()) {
      command_directions_.assign(joint_names_.size(), 1.0);
    }
    if (controller_min_positions_deg_.size() != joint_names_.size() ||
      controller_max_positions_deg_.size() != joint_names_.size() ||
      command_origins_deg_.size() != joint_names_.size() ||
      command_directions_.size() != joint_names_.size())
    {
      throw std::invalid_argument("degree bridge command-frame parameter sizes must match joint_names");
    }
    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
      if (joint_names_[i].empty() ||
        !std::isfinite(min_positions_deg_[i]) ||
        !std::isfinite(max_positions_deg_[i]) ||
        min_positions_deg_[i] > max_positions_deg_[i] ||
        !std::isfinite(controller_min_positions_deg_[i]) ||
        !std::isfinite(controller_max_positions_deg_[i]) ||
        controller_min_positions_deg_[i] > controller_max_positions_deg_[i] ||
        !std::isfinite(command_origins_deg_[i]) ||
        !std::isfinite(command_directions_[i]) || command_directions_[i] == 0.0 ||
        !joint_index_.emplace(joint_names_[i], i).second)
      {
        throw std::invalid_argument("invalid or duplicate degree bridge joint parameter");
      }
    }

    publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(output_topic, 10);
    subscription_ = create_subscription<robot_arm_controller::msg::JointCommandDegrees>(
      "joint_commands_deg", 10,
      std::bind(&DegreeTrajectoryBridge::command_callback, this, std::placeholders::_1));
    joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS(),
      std::bind(&DegreeTrajectoryBridge::joint_state_callback, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(), "Absolute degree commands: 'joint_commands_deg' -> '%s'", output_topic.c_str());
  }

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    const auto count = std::min(message->name.size(), message->position.size());
    for (std::size_t i = 0; i < count; ++i) {
      if (joint_index_.count(message->name[i]) != 0 && std::isfinite(message->position[i])) {
        current_positions_[message->name[i]] = message->position[i];
      }
    }
  }

  void command_callback(
    const robot_arm_controller::msg::JointCommandDegrees::SharedPtr msg)
  {
    if (msg->joint_names.empty() || msg->joint_names.size() != msg->positions_deg.size()) {
      RCLCPP_ERROR(get_logger(), "joint_names and positions_deg must have the same non-zero length");
      return;
    }
    if (!std::isfinite(msg->duration_sec) || msg->duration_sec <= 0.0 ||
      msg->duration_sec > max_duration_sec_)
    {
      RCLCPP_ERROR(
        get_logger(), "duration_sec must be within (0, %.3f]", max_duration_sec_);
      return;
    }

    trajectory_msgs::msg::JointTrajectory trajectory;
    // Leave the stamp at zero so the controller starts immediately.  The
    // first point below is the measured state, while the second is always an
    // absolute target; no command is treated as a position increment.
    trajectory_msgs::msg::JointTrajectoryPoint target_point;
    std::unordered_set<std::string> commanded;
    bool have_current_start = true;
    std::vector<double> current_start_positions;

    for (std::size_t i = 0; i < msg->joint_names.size(); ++i) {
      const auto found = joint_index_.find(msg->joint_names[i]);
      if (found == joint_index_.end() || !commanded.insert(msg->joint_names[i]).second) {
        RCLCPP_ERROR(
          get_logger(), "unknown or duplicate joint: %s", msg->joint_names[i].c_str());
        return;
      }
      const auto joint = found->second;
      const double degrees = msg->positions_deg[i];
      if (!std::isfinite(degrees)) {
        RCLCPP_ERROR(
          get_logger(), "%s command is not finite; skipping this joint",
          msg->joint_names[i].c_str());
        continue;
      }
      const double bounded_degrees = std::clamp(
        degrees, min_positions_deg_[joint], max_positions_deg_[joint]);
      if (bounded_degrees != degrees) {
        RCLCPP_WARN(
          get_logger(), "%s command %.3f deg saturated to %.3f deg",
          msg->joint_names[i].c_str(), degrees, bounded_degrees);
      }
      const double controller_degrees = std::clamp(
        command_origins_deg_[joint] + command_directions_[joint] * bounded_degrees,
        controller_min_positions_deg_[joint], controller_max_positions_deg_[joint]);
      trajectory.joint_names.push_back(msg->joint_names[i]);
      target_point.positions.push_back(
        robot_arm_controller::angle_utils::degrees_to_radians(controller_degrees));
      target_point.velocities.push_back(0.0);

      const auto current = current_positions_.find(msg->joint_names[i]);
      if (current == current_positions_.end()) {
        have_current_start = false;
      } else {
        current_start_positions.push_back(current->second);
      }
    }

    if (trajectory.joint_names.empty()) {
      RCLCPP_ERROR(get_logger(), "No valid joint command to publish");
      return;
    }

    const auto duration_nanoseconds = static_cast<int64_t>(
      std::llround(msg->duration_sec * 1e9));
    target_point.time_from_start.sec = static_cast<int32_t>(duration_nanoseconds / 1000000000LL);
    target_point.time_from_start.nanosec = static_cast<uint32_t>(
      duration_nanoseconds % 1000000000LL);
    if (have_current_start) {
      trajectory_msgs::msg::JointTrajectoryPoint start_point;
      start_point.positions = std::move(current_start_positions);
      start_point.velocities.assign(start_point.positions.size(), 0.0);
      // A zero-time point makes the measured state the explicit start of the
      // absolute trajectory, including when the requested target is 0.
      trajectory.points.push_back(std::move(start_point));
    }
    trajectory.points.push_back(std::move(target_point));
    publisher_->publish(trajectory);
  }

  std::vector<std::string> joint_names_;
  std::vector<double> min_positions_deg_;
  std::vector<double> max_positions_deg_;
  std::vector<double> controller_min_positions_deg_;
  std::vector<double> controller_max_positions_deg_;
  std::vector<double> command_origins_deg_;
  std::vector<double> command_directions_;
  double max_duration_sec_{120.0};
  std::unordered_map<std::string, std::size_t> joint_index_;
  std::unordered_map<std::string, double> current_positions_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
  rclcpp::Subscription<robot_arm_controller::msg::JointCommandDegrees>::SharedPtr subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<DegreeTrajectoryBridge>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("degree_trajectory_bridge"), "%s", exception.what());
  }
  rclcpp::shutdown();
  return 0;
}
