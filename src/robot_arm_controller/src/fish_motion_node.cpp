#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "builtin_interfaces/msg/duration.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_arm_controller/angle_utils.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;

class FishMotionNode : public rclcpp::Node
{
public:
  FishMotionNode()
  : Node("fish_motion_node")
  {
    joint1_amplitude_deg_ = declare_parameter<double>("joint1_amplitude_deg", 8.0);
    joint2_amplitude_deg_ = declare_parameter<double>("joint2_amplitude_deg", 15.0);
    phase_lag_deg_ = declare_parameter<double>("phase_lag_deg", 60.0);
    period_sec_ = declare_parameter<double>("period_sec", 2.5);
    center_duration_sec_ = declare_parameter<double>("center_duration_sec", 3.0);
    cycles_ = declare_parameter<int>("cycles", 3);
    samples_per_cycle_ = declare_parameter<int>("samples_per_cycle", 20);
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/arm_trajectory_controller/joint_trajectory");

    validate_parameters();
    publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(output_topic, 10);
    publish_timer_ = create_wall_timer(100ms, std::bind(&FishMotionNode::try_publish, this));

    RCLCPP_INFO(
      get_logger(),
      "Preparing fish motion: amplitudes=(%.1f, %.1f) deg, period=%.2f s, cycles=%d",
      joint1_amplitude_deg_, joint2_amplitude_deg_, period_sec_, cycles_);
  }

private:
  static builtin_interfaces::msg::Duration duration_from_seconds(double seconds)
  {
    const auto nanoseconds = static_cast<int64_t>(std::llround(seconds * 1e9));
    builtin_interfaces::msg::Duration duration;
    duration.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
    duration.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
    return duration;
  }

  void validate_parameters() const
  {
    if (!std::isfinite(joint1_amplitude_deg_) || joint1_amplitude_deg_ < 0.0 ||
      joint1_amplitude_deg_ > 98.0)
    {
      throw std::invalid_argument("joint1_amplitude_deg must be within 0..98 degrees");
    }
    if (!std::isfinite(joint2_amplitude_deg_) || joint2_amplitude_deg_ < 0.0 ||
      joint2_amplitude_deg_ > 113.0)
    {
      throw std::invalid_argument("joint2_amplitude_deg must be within 0..113 degrees");
    }
    if (!std::isfinite(phase_lag_deg_) || !std::isfinite(period_sec_) || period_sec_ <= 0.0 ||
      !std::isfinite(center_duration_sec_) || center_duration_sec_ <= 0.0)
    {
      throw std::invalid_argument("phase and duration parameters must be finite and positive");
    }
    if (cycles_ < 1 || cycles_ > 20 || samples_per_cycle_ < 8 || samples_per_cycle_ > 200) {
      throw std::invalid_argument("cycles must be 1..20 and samples_per_cycle must be 8..200");
    }
  }

  void try_publish()
  {
    ++wait_attempts_;
    if (publisher_->get_subscription_count() == 0 && wait_attempts_ < 50) {
      return;
    }
    if (publisher_->get_subscription_count() == 0) {
      RCLCPP_ERROR(get_logger(), "Trajectory controller subscriber was not found");
      rclcpp::shutdown();
      return;
    }

    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.header.stamp = now();
    trajectory.joint_names = {"joint1", "joint2"};

    trajectory_msgs::msg::JointTrajectoryPoint center;
    center.positions = {0.0, 0.0};
    center.time_from_start = duration_from_seconds(center_duration_sec_);
    trajectory.points.push_back(center);

    const int sample_count = cycles_ * samples_per_cycle_;
    const double phase_lag = robot_arm_controller::angle_utils::degrees_to_radians(
      phase_lag_deg_);
    for (int sample = 1; sample <= sample_count; ++sample) {
      const double progress = static_cast<double>(sample) / sample_count;
      const double phase = 2.0 * robot_arm_controller::angle_utils::kPi * cycles_ * progress;
      // The envelope starts and ends at zero to avoid an abrupt kick.
      const double envelope = std::sin(robot_arm_controller::angle_utils::kPi * progress);
      const double joint1_deg = envelope * joint1_amplitude_deg_ * std::sin(phase);
      const double joint2_deg = envelope * joint2_amplitude_deg_ * std::sin(phase - phase_lag);

      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions = {
        robot_arm_controller::angle_utils::degrees_to_radians(joint1_deg),
        robot_arm_controller::angle_utils::degrees_to_radians(joint2_deg)};
      point.time_from_start = duration_from_seconds(
        center_duration_sec_ + progress * cycles_ * period_sec_);
      trajectory.points.push_back(std::move(point));
    }

    publisher_->publish(trajectory);
    publish_timer_->cancel();
    RCLCPP_INFO(
      get_logger(), "Published %zu points; returning both joints to 0 deg",
      trajectory.points.size());
    exit_timer_ = create_wall_timer(500ms, []() {rclcpp::shutdown();});
  }

  double joint1_amplitude_deg_;
  double joint2_amplitude_deg_;
  double phase_lag_deg_;
  double period_sec_;
  double center_duration_sec_;
  int cycles_;
  int samples_per_cycle_;
  int wait_attempts_{0};
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr exit_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<FishMotionNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("fish_motion_node"), "%s", exception.what());
  }
  rclcpp::shutdown();
  return 0;
}
