#ifndef ROBOT_ARM_CONTROLLER__GRIPPER_UTILS_HPP_
#define ROBOT_ARM_CONTROLLER__GRIPPER_UTILS_HPP_

#include <algorithm>
#include <cmath>

namespace robot_arm_controller
{
namespace gripper_utils
{

inline double opening_cm_to_degrees(
  double opening_cm, double max_opening_cm, double max_opening_deg)
{
  return std::clamp(opening_cm, 0.0, max_opening_cm) * max_opening_deg / max_opening_cm;
}

}  // namespace gripper_utils
}  // namespace robot_arm_controller

#endif  // ROBOT_ARM_CONTROLLER__GRIPPER_UTILS_HPP_
