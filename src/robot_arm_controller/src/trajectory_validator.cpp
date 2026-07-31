#include "robot_arm_controller/trajectory_validator.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <unordered_set>
#include <utility>

namespace robot_arm_controller
{
namespace
{

double duration_seconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) +
         static_cast<double>(duration.nanosec) * 1e-9;
}

bool vector_is_finite(const std::vector<double> & values)
{
  return std::all_of(values.begin(), values.end(), [](double value) {
    return std::isfinite(value);
  });
}

std::string point_field_error(
  std::size_t point_index, const char * field, std::size_t actual, std::size_t expected)
{
  std::ostringstream stream;
  stream << "point " << point_index << " " << field << " size " << actual
         << " does not match joint count " << expected;
  return stream.str();
}

}  // namespace

TrajectoryValidationResult validate_and_sanitize_trajectory(
  const trajectory_msgs::msg::JointTrajectory & input,
  const std::unordered_map<std::string, JointLimit> & limits,
  const TrajectoryValidationOptions & options,
  const std::unordered_map<std::string, double> & current_positions)
{
  TrajectoryValidationResult result;
  result.trajectory = input;

  if (limits.empty()) {
    result.error = "no configured joint limits";
    return result;
  }
  if (input.joint_names.empty() || input.points.empty()) {
    result.error = "joint_names and points must be non-empty";
    return result;
  }
  if (input.points.size() > options.max_points) {
    result.error = "trajectory exceeds the configured point count";
    return result;
  }
  if (!std::isfinite(options.max_duration_sec) || options.max_duration_sec <= 0.0) {
    result.error = "invalid trajectory validation duration";
    return result;
  }

  std::unordered_set<std::string> commanded;
  std::vector<const JointLimit *> ordered_limits;
  ordered_limits.reserve(input.joint_names.size());
  for (const auto & name : input.joint_names) {
    const auto found = limits.find(name);
    if (name.empty() || found == limits.end()) {
      result.error = "unknown joint: " + name;
      return result;
    }
    if (!commanded.insert(name).second) {
      result.error = "duplicate joint: " + name;
      return result;
    }
    ordered_limits.push_back(&found->second);
  }

  const auto joint_count = input.joint_names.size();
  double previous_time = -1.0;
  for (std::size_t point_index = 0; point_index < result.trajectory.points.size(); ++point_index) {
    auto & point = result.trajectory.points[point_index];
    if (point.positions.size() != joint_count) {
      result.error = point_field_error(
        point_index, "positions", point.positions.size(), joint_count);
      return result;
    }
    for (const auto * field : {&point.velocities, &point.accelerations, &point.effort}) {
      if (!field->empty() && field->size() != joint_count) {
        const char * field_name = field == &point.velocities ? "velocities" :
          (field == &point.accelerations ? "accelerations" : "effort");
        result.error = point_field_error(point_index, field_name, field->size(), joint_count);
        return result;
      }
    }
    if (!vector_is_finite(point.positions) ||
      !vector_is_finite(point.velocities) ||
      !vector_is_finite(point.accelerations) ||
      !vector_is_finite(point.effort))
    {
      result.error = "trajectory contains NaN or infinity";
      return result;
    }
    if (point.time_from_start.sec < 0 || point.time_from_start.nanosec >= 1000000000U) {
      result.error = "trajectory contains an invalid time_from_start";
      return result;
    }
    const double current_time = duration_seconds(point.time_from_start);
    if (current_time <= previous_time) {
      result.error = "time_from_start must be strictly increasing";
      return result;
    }
    if (current_time > options.max_duration_sec) {
      result.error = "trajectory exceeds the configured duration";
      return result;
    }

    for (std::size_t joint_index = 0; joint_index < joint_count; ++joint_index) {
      const auto & limit = *ordered_limits[joint_index];
      const double requested_position = point.positions[joint_index];
      point.positions[joint_index] = std::clamp(
        requested_position, limit.min_position, limit.max_position);
      if (point.positions[joint_index] != requested_position) {
        std::ostringstream warning;
        warning << input.joint_names[joint_index] << " point " << point_index
                << " saturated from " << requested_position << " to "
                << point.positions[joint_index] << " rad";
        result.warnings.push_back(warning.str());
      }
      if (!point.velocities.empty() &&
        std::abs(point.velocities[joint_index]) > limit.max_velocity)
      {
        result.error = input.joint_names[joint_index] + " velocity exceeds its limit";
        return result;
      }

      if (!options.check_average_velocity) {
        continue;
      }
      double segment_start_position = point.positions[joint_index];
      double segment_duration = 0.0;
      if (point_index > 0) {
        segment_start_position =
          result.trajectory.points[point_index - 1].positions[joint_index];
        segment_duration = current_time - previous_time;
      } else {
        const auto current = current_positions.find(input.joint_names[joint_index]);
        if (current != current_positions.end() && std::isfinite(current->second)) {
          segment_start_position = current->second;
          segment_duration = current_time;
        }
      }
      const double distance = std::abs(point.positions[joint_index] - segment_start_position);
      if ((segment_duration <= 0.0 && distance > 1e-9) ||
        (segment_duration > 0.0 && distance / segment_duration > limit.max_velocity))
      {
        result.error =
          input.joint_names[joint_index] + " requires motion faster than its configured limit";
        return result;
      }
    }
    previous_time = current_time;
  }

  result.valid = true;
  return result;
}

}  // namespace robot_arm_controller
