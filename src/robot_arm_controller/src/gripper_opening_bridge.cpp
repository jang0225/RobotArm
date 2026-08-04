#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "robot_arm_controller/gripper_utils.hpp"
#include "robot_arm_controller/msg/gripper_command_cm.hpp"
#include "robot_arm_controller/msg/joint_command_degrees.hpp"

class GripperOpeningBridge : public rclcpp::Node
{
public:
  GripperOpeningBridge()
  : Node("gripper_opening_bridge")
  {
    max_opening_cm_ = declare_parameter<double>("max_opening_cm", 13.0);
    max_opening_deg_ = declare_parameter<double>("max_opening_deg", 278.905469);
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "joint_commands_deg");
    if (!std::isfinite(max_opening_cm_) || !std::isfinite(max_opening_deg_) ||
      max_opening_cm_ <= 0.0 || max_opening_deg_ <= 0.0)
    {
      throw std::invalid_argument("gripper opening limits must be positive and finite");
    }

    degree_publisher_ = create_publisher<robot_arm_controller::msg::JointCommandDegrees>(
      output_topic, 10);
    opening_subscription_ = create_subscription<robot_arm_controller::msg::GripperCommandCm>(
      "gripper_opening_cm", 10,
      std::bind(&GripperOpeningBridge::command_callback, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(), "Gripper opening commands: 'gripper_opening_cm' -> '%s' "
      "(0..%.3f cm)", output_topic.c_str(), max_opening_cm_);
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

    robot_arm_controller::msg::JointCommandDegrees command;
    command.joint_names = {"gripper_joint"};
    command.positions_deg = {robot_arm_controller::gripper_utils::opening_cm_to_degrees(
      bounded_opening, max_opening_cm_, max_opening_deg_)};
    command.duration_sec = message->duration_sec;
    degree_publisher_->publish(command);
  }

  double max_opening_cm_{13.0};
  double max_opening_deg_{278.905469};
  rclcpp::Publisher<robot_arm_controller::msg::JointCommandDegrees>::SharedPtr degree_publisher_;
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
