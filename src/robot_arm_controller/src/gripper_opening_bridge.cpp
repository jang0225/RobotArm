#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "robot_arm_controller/angle_utils.hpp"
#include "robot_arm_controller/gripper_utils.hpp"
#include "robot_arm_controller/msg/gripper_command_cm.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

class GripperOpeningBridge : public rclcpp::Node
{
public:
  GripperOpeningBridge()
  : Node("gripper_opening_bridge")
  {
    max_opening_cm_ = declare_parameter<double>("max_opening_cm", 8.65);
    closed_position_deg_ = declare_parameter<double>("closed_position_deg", 311.484375);
    open_position_deg_ = declare_parameter<double>("open_position_deg", 118.212890625);
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "arm_trajectory_controller/joint_trajectory");
    if (!std::isfinite(max_opening_cm_) || !std::isfinite(closed_position_deg_) ||
      !std::isfinite(open_position_deg_) || max_opening_cm_ <= 0.0 ||
      closed_position_deg_ == open_position_deg_)
    {
      throw std::invalid_argument("gripper opening calibration must be finite and non-zero");
    }

    trajectory_publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      output_topic, 10);
    opening_subscription_ = create_subscription<robot_arm_controller::msg::GripperCommandCm>(
      "gripper_opening_cm", 10,
      std::bind(&GripperOpeningBridge::command_callback, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(), "Absolute gripper commands: 0 cm = %.6f deg, %.3f cm = %.6f deg -> '%s'",
      closed_position_deg_, max_opening_cm_, open_position_deg_, output_topic.c_str());
  }

private:
  void command_callback(const robot_arm_controller::msg::GripperCommandCm::SharedPtr message)
  {
    if (!std::isfinite(message->opening_cm) || !std::isfinite(message->duration_sec) ||
      message->duration_sec <= 0.0)
    {
      RCLCPP_ERROR(get_logger(), "gripper opening and duration must be finite; duration > 0");
      return;
    }
    const double bounded_opening = std::clamp(message->opening_cm, 0.0, max_opening_cm_);
    if (bounded_opening != message->opening_cm) {
      RCLCPP_WARN(
        get_logger(), "Gripper command %.3f cm saturated to %.3f cm", message->opening_cm,
        bounded_opening);
    }

    const double target_degrees =
      robot_arm_controller::gripper_utils::opening_cm_to_position_degrees(
      bounded_opening, max_opening_cm_, closed_position_deg_, open_position_deg_);

    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.joint_names = {"gripper_joint"};
    trajectory_msgs::msg::JointTrajectoryPoint target;
    target.positions = {robot_arm_controller::angle_utils::degrees_to_radians(target_degrees)};
    target.velocities = {0.0};
    const auto duration_nanoseconds = static_cast<int64_t>(
      std::llround(message->duration_sec * 1e9));
    target.time_from_start.sec = static_cast<int32_t>(duration_nanoseconds / 1000000000LL);
    target.time_from_start.nanosec = static_cast<uint32_t>(
      duration_nanoseconds % 1000000000LL);
    trajectory.points.push_back(std::move(target));
    trajectory_publisher_->publish(trajectory);
    RCLCPP_INFO(
      get_logger(), "Gripper absolute target: %.3f cm -> %.6f deg",
      bounded_opening, target_degrees);
  }

  double max_opening_cm_{8.65};
  double closed_position_deg_{311.484375};
  double open_position_deg_{118.212890625};
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;
  rclcpp::Subscription<robot_arm_controller::msg::GripperCommandCm>::SharedPtr
    opening_subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<GripperOpeningBridge>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("gripper_opening_bridge"), "%s", exception.what());
  }
  rclcpp::shutdown();
  return 0;
}
