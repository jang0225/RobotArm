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
    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
      if (joint_names_[i].empty() ||
        !std::isfinite(min_positions_deg_[i]) ||
        !std::isfinite(max_positions_deg_[i]) ||
        min_positions_deg_[i] > max_positions_deg_[i] ||
        !joint_index_.emplace(joint_names_[i], i).second)
      {
        throw std::invalid_argument("invalid or duplicate degree bridge joint parameter");
      }
    }

    publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(output_topic, 10);
    subscription_ = create_subscription<robot_arm_controller::msg::JointCommandDegrees>(
      "joint_commands_deg", 10,
      std::bind(&DegreeTrajectoryBridge::command_callback, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(), "Degree commands: 'joint_commands_deg' -> '%s'", output_topic.c_str());
  }

private:
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
    trajectory.header.stamp = now();
    trajectory_msgs::msg::JointTrajectoryPoint point;
    std::unordered_set<std::string> commanded;

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
      trajectory.joint_names.push_back(msg->joint_names[i]);
      point.positions.push_back(
        robot_arm_controller::angle_utils::degrees_to_radians(bounded_degrees));
      point.velocities.push_back(0.0);
    }

    if (trajectory.joint_names.empty()) {
      RCLCPP_ERROR(get_logger(), "No valid joint command to publish");
      return;
    }

    const auto duration_nanoseconds = static_cast<int64_t>(
      std::llround(msg->duration_sec * 1e9));
    point.time_from_start.sec = static_cast<int32_t>(duration_nanoseconds / 1000000000LL);
    point.time_from_start.nanosec = static_cast<uint32_t>(
      duration_nanoseconds % 1000000000LL);
    trajectory.points.push_back(std::move(point));
    publisher_->publish(trajectory);
  }

  std::vector<std::string> joint_names_;
  std::vector<double> min_positions_deg_;
  std::vector<double> max_positions_deg_;
  double max_duration_sec_{120.0};
  std::unordered_map<std::string, std::size_t> joint_index_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
  rclcpp::Subscription<robot_arm_controller::msg::JointCommandDegrees>::SharedPtr subscription_;
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
