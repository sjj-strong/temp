#include <cmath>
#include <chrono>
#include <functional>
#include <memory>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{
constexpr double kZOffsetMetres = 0.05;
constexpr double kMinimumMotionSeconds = 1.0;
constexpr double kSettleSeconds = 0.5;
constexpr double kMotionThresholdMetres = 0.0002;
constexpr double kResultTimeoutSeconds = 5.0;

double position_distance(const geometry_msgs::msg::Point& first, const geometry_msgs::msg::Point& second)
{
  const double x = first.x - second.x;
  const double y = first.y - second.y;
  const double z = first.z - second.z;
  return std::sqrt(x * x + y * y + z * z);
}

class BaseZ005Command final : public rclcpp::Node
{
public:
  BaseZ005Command()
  : Node("move_base_z_005")
  {
    target_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "/cartesian_impedance_controller/target_pose", rclcpp::SystemDefaultsQoS());
    error_publisher_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
        "/cartesian_impedance_controller/base_z_005_error", rclcpp::QoS(1).transient_local());
    pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/tcp_pose_broadcaster/pose", rclcpp::SystemDefaultsQoS(),
        std::bind(&BaseZ005Command::pose_callback, this, std::placeholders::_1));
    result_timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&BaseZ005Command::check_result, this));
    RCLCPP_INFO(get_logger(), "等待当前 TCP 位姿；将只发布 base 坐标系下 Z 轴 +0.05 m 的单次目标");
  }

private:
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
  {
    if (pose->header.frame_id != "base") {
      RCLCPP_ERROR(get_logger(), "拒绝发布：当前 TCP 位姿坐标系为 '%s'，预期为 'base'", pose->header.frame_id.c_str());
      rclcpp::shutdown();
      return;
    }

    const rclcpp::Time received_time = now();
    if (!sent_) {
      start_pose_ = *pose;
      latest_pose_ = *pose;
      target_pose_ = *pose;
      target_pose_.header.stamp = received_time;
      target_pose_.pose.position.z += kZOffsetMetres;
      command_time_ = received_time;
      last_motion_time_ = received_time;
      target_publisher_->publish(target_pose_);
      sent_ = true;
      RCLCPP_INFO(get_logger(), "已发布固定目标：base 坐标系 Z 轴 +0.05 m；等待运动稳定后计算误差");
      return;
    }

    if (position_distance(latest_pose_.pose.position, pose->pose.position) > kMotionThresholdMetres) {
      last_motion_time_ = received_time;
    }
    latest_pose_ = *pose;
    has_measurement_after_command_ = true;
  }

  void check_result()
  {
    if (!sent_ || reported_) {
      return;
    }
    const rclcpp::Time current_time = now();
    const double elapsed = (current_time - command_time_).seconds();
    const bool settled = elapsed >= kMinimumMotionSeconds && has_measurement_after_command_ &&
                         (current_time - last_motion_time_).seconds() >= kSettleSeconds;
    if (!settled && elapsed < kResultTimeoutSeconds) {
      return;
    }
    report_error(!settled);
  }

  void report_error(const bool timed_out)
  {
    const double error_x = target_pose_.pose.position.x - latest_pose_.pose.position.x;
    const double error_y = target_pose_.pose.position.y - latest_pose_.pose.position.y;
    const double error_z = target_pose_.pose.position.z - latest_pose_.pose.position.z;
    const double actual_x = latest_pose_.pose.position.x - start_pose_.pose.position.x;
    const double actual_y = latest_pose_.pose.position.y - start_pose_.pose.position.y;
    const double actual_z = latest_pose_.pose.position.z - start_pose_.pose.position.z;
    const double norm = std::sqrt(error_x * error_x + error_y * error_y + error_z * error_z);

    geometry_msgs::msg::Vector3Stamped error_message;
    error_message.header.stamp = now();
    error_message.header.frame_id = "base";
    error_message.vector.x = error_x;
    error_message.vector.y = error_y;
    error_message.vector.z = error_z;
    error_publisher_->publish(error_message);

    if (timed_out) {
      RCLCPP_WARN(get_logger(),
          "Z+0.05 m 测试在 5 秒超时后取样：实际位移 [%.3f, %.3f, %.3f] mm，目标减实际误差 [%.3f, %.3f, %.3f] mm，"
          "位置误差范数 %.3f mm；误差已发布到 /cartesian_impedance_controller/base_z_005_error",
          actual_x * 1000.0, actual_y * 1000.0, actual_z * 1000.0,
          error_x * 1000.0, error_y * 1000.0, error_z * 1000.0, norm * 1000.0);
    } else {
      RCLCPP_INFO(get_logger(),
          "Z+0.05 m 测试完成：实际位移 [%.3f, %.3f, %.3f] mm，目标减实际误差 [%.3f, %.3f, %.3f] mm，"
          "位置误差范数 %.3f mm；误差已发布到 /cartesian_impedance_controller/base_z_005_error",
          actual_x * 1000.0, actual_y * 1000.0, actual_z * 1000.0,
          error_x * 1000.0, error_y * 1000.0, error_z * 1000.0, norm * 1000.0);
    }
    reported_ = true;
    rclcpp::shutdown();
  }

  bool sent_{ false };
  bool reported_{ false };
  bool has_measurement_after_command_{ false };
  rclcpp::Time command_time_{ 0, 0, RCL_ROS_TIME };
  rclcpp::Time last_motion_time_{ 0, 0, RCL_ROS_TIME };
  geometry_msgs::msg::PoseStamped start_pose_;
  geometry_msgs::msg::PoseStamped target_pose_;
  geometry_msgs::msg::PoseStamped latest_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr error_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
  rclcpp::TimerBase::SharedPtr result_timer_;
};
}  // namespace

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BaseZ005Command>());
  rclcpp::shutdown();
  return 0;
}
