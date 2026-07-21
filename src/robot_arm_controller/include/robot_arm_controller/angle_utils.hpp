#ifndef ROBOT_ARM_CONTROLLER__ANGLE_UTILS_HPP_
#define ROBOT_ARM_CONTROLLER__ANGLE_UTILS_HPP_

namespace robot_arm_controller
{
namespace angle_utils
{

constexpr double kPi = 3.14159265358979323846;

constexpr double degrees_to_radians(double degrees)
{
  return degrees * kPi / 180.0;
}

}  // namespace angle_utils
}  // namespace robot_arm_controller

#endif  // ROBOT_ARM_CONTROLLER__ANGLE_UTILS_HPP_
