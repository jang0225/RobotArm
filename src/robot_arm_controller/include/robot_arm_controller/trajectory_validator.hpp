#ifndef ROBOT_ARM_CONTROLLER__TRAJECTORY_VALIDATOR_HPP_
#define ROBOT_ARM_CONTROLLER__TRAJECTORY_VALIDATOR_HPP_

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace robot_arm_controller
{

struct JointLimit
{
  double min_position;
  double max_position;
  double max_velocity;
};

struct TrajectoryValidationOptions
{
  std::size_t max_points{1000};
  double max_duration_sec{120.0};
  bool check_average_velocity{true};
};

struct TrajectoryValidationResult
{
  bool valid{false};
  std::string error;
  std::vector<std::string> warnings;
  trajectory_msgs::msg::JointTrajectory trajectory;
};

TrajectoryValidationResult validate_and_sanitize_trajectory(
  const trajectory_msgs::msg::JointTrajectory & input,
  const std::unordered_map<std::string, JointLimit> & limits,
  const TrajectoryValidationOptions & options,
  const std::unordered_map<std::string, double> & current_positions = {});

}  // namespace robot_arm_controller

#endif  // ROBOT_ARM_CONTROLLER__TRAJECTORY_VALIDATOR_HPP_
