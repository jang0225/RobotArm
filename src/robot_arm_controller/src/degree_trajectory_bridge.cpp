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
    const bool include_gripper = declare_parameter<bool>("include_gripper", true);
    std::vector<std::string> default_joint_names{"joint1", "joint2", "joint3"};
    std::vector<double> default_min_positions{-98.456875, -113.134766, -105.055};
    std::vector<double> default_max_positions{98.543125, 113.135234, 93.535};
    if (include_gripper) {
      default_joint_names.push_back("gripper_joint");
      default_min_positions.push_back(-5.0);
      default_max_positions.push_back(5.0);
    }
    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names", default_joint_names);
    min_positions_deg_ = declare_parameter<std::vector<double>>(
      "min_position_deg", default_min_positions);
    max_positions_deg_ = declare_parameter<std::vector<double>>(
      "max_position_deg", default_max_positions);
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/arm_trajectory_controller/joint_trajectory");

    if (joint_names_.empty() || min_positions_deg_.size() != joint_names_.size() ||
      max_positions_deg_.size() != joint_names_.size())
    {
      throw std::invalid_argument("degree bridge parameter array lengths must match");
    }
    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
      joint_index_.emplace(joint_names_[i], i);
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
    if (!std::isfinite(msg->duration_sec) || msg->duration_sec <= 0.0) {
      RCLCPP_ERROR(get_logger(), "duration_sec must be positive");
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
      if (!std::isfinite(degrees) || degrees < min_positions_deg_[joint] ||
        degrees > max_positions_deg_[joint])
      {
        RCLCPP_ERROR(
          get_logger(), "%s command %.3f deg outside [%.3f, %.3f]",
          msg->joint_names[i].c_str(), degrees, min_positions_deg_[joint],
          max_positions_deg_[joint]);
        return;
      }
      trajectory.joint_names.push_back(msg->joint_names[i]);
      point.positions.push_back(
        robot_arm_controller::angle_utils::degrees_to_radians(degrees));
      point.velocities.push_back(0.0);
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
