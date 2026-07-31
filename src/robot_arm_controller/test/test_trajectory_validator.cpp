#include <cmath>
#include <limits>
#include <string>
#include <unordered_map>

#include "gtest/gtest.h"
#include "robot_arm_controller/trajectory_validator.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace
{

using robot_arm_controller::JointLimit;
using robot_arm_controller::TrajectoryValidationOptions;

trajectory_msgs::msg::JointTrajectory two_joint_trajectory()
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = {"joint1", "joint2"};
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {0.2, -0.2};
  point.time_from_start.sec = 2;
  trajectory.points.push_back(point);
  return trajectory;
}

const std::unordered_map<std::string, JointLimit> kLimits{
  {"joint1", JointLimit{-1.0, 1.0, 1.0}},
  {"joint2", JointLimit{-2.0, 2.0, 2.0}},
};

const TrajectoryValidationOptions kOptions{100, 10.0, true};

TEST(TrajectoryValidator, SaturatesOnlyTheJointOutsideItsLimit)
{
  auto trajectory = two_joint_trajectory();
  trajectory.points[0].positions = {4.0, 1.5};

  const auto result = robot_arm_controller::validate_and_sanitize_trajectory(
    trajectory, kLimits, kOptions);

  ASSERT_TRUE(result.valid) << result.error;
  ASSERT_EQ(result.warnings.size(), 1U);
  EXPECT_DOUBLE_EQ(result.trajectory.points[0].positions[0], 1.0);
  // Regression guard: joint1 saturation must never discard or hold joint2.
  EXPECT_DOUBLE_EQ(result.trajectory.points[0].positions[1], 1.5);
}

TEST(TrajectoryValidator, RejectsUnknownAndDuplicateJoints)
{
  auto unknown = two_joint_trajectory();
  unknown.joint_names[1] = "unknown";
  EXPECT_FALSE(
    robot_arm_controller::validate_and_sanitize_trajectory(
      unknown, kLimits, kOptions).valid);

  auto duplicate = two_joint_trajectory();
  duplicate.joint_names[1] = "joint1";
  EXPECT_FALSE(
    robot_arm_controller::validate_and_sanitize_trajectory(
      duplicate, kLimits, kOptions).valid);
}

TEST(TrajectoryValidator, RejectsMalformedOrNonfinitePoints)
{
  auto wrong_size = two_joint_trajectory();
  wrong_size.points[0].positions.pop_back();
  EXPECT_FALSE(
    robot_arm_controller::validate_and_sanitize_trajectory(
      wrong_size, kLimits, kOptions).valid);

  auto nonfinite = two_joint_trajectory();
  nonfinite.points[0].positions[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(
    robot_arm_controller::validate_and_sanitize_trajectory(
      nonfinite, kLimits, kOptions).valid);
}

TEST(TrajectoryValidator, RequiresIncreasingBoundedTime)
{
  auto trajectory = two_joint_trajectory();
  auto second = trajectory.points.front();
  second.time_from_start.sec = 1;
  trajectory.points.push_back(second);
  EXPECT_FALSE(
    robot_arm_controller::validate_and_sanitize_trajectory(
      trajectory, kLimits, kOptions).valid);

  trajectory = two_joint_trajectory();
  trajectory.points[0].time_from_start.sec = 11;
  EXPECT_FALSE(
    robot_arm_controller::validate_and_sanitize_trajectory(
      trajectory, kLimits, kOptions).valid);
}

TEST(TrajectoryValidator, RejectsMotionFasterThanJointLimit)
{
  auto trajectory = two_joint_trajectory();
  trajectory.points[0].positions[0] = 1.0;
  trajectory.points[0].time_from_start.sec = 0;
  trajectory.points[0].time_from_start.nanosec = 100000000;
  const std::unordered_map<std::string, double> current{{"joint1", 0.0}, {"joint2", 0.0}};

  const auto result = robot_arm_controller::validate_and_sanitize_trajectory(
    trajectory, kLimits, kOptions, current);

  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.error.find("joint1"), std::string::npos);
}

}  // namespace
