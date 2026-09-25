#include <cmath>
#include <chrono>
#include <cctype>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{

constexpr double kMinimumMotionSeconds = 1.0;
constexpr double kSettleSeconds = 0.5;
constexpr double kMotionThresholdMetres = 0.0002;  // 0.2 mm
constexpr double kResultTimeoutSeconds = 5.0;

// 安全限制，可根据需要修改
constexpr double kMaxOffsetMetres = 0.10;  // 最大单次 10 cm

double position_distance(
    const geometry_msgs::msg::Point & first,
    const geometry_msgs::msg::Point & second)
{
  const double x = first.x - second.x;
  const double y = first.y - second.y;
  const double z = first.z - second.z;

  return std::sqrt(x * x + y * y + z * z);
}

struct MotionCommand
{
  char axis;
  double offset;
};

MotionCommand parse_command(const std::vector<std::string> & args)
{
  if (args.size() != 3) {
    throw std::runtime_error(
      "用法:\n"
      "  move_base_xyz x  0.05\n"
      "  move_base_xyz x -0.05\n"
      "  move_base_xyz y  0.05\n"
      "  move_base_xyz y -0.05\n"
      "  move_base_xyz z  0.05\n"
      "  move_base_xyz z -0.05\n"
      "\n"
      "也支持:\n"
      "  move_base_xyz x+ 0.05\n"
      "  move_base_xyz x- 0.05");
  }

  std::string axis_string = args[1];

  for (char & c : axis_string) {
    c = static_cast<char>(
      std::tolower(static_cast<unsigned char>(c)));
  }

  std::size_t parsed_characters = 0;
  double value = 0.0;

  try {
    value = std::stod(args[2], &parsed_characters);
  } catch (...) {
    throw std::runtime_error(
      "位移量必须是数字，例如 0.05 或 -0.05");
  }

  if (parsed_characters != args[2].size()) {
    throw std::runtime_error(
      "位移量格式错误，例如应写成 0.05 或 -0.05");
  }

  if (!std::isfinite(value)) {
    throw std::runtime_error("位移量必须是有限数值");
  }

  char axis = '\0';
  double offset = 0.0;

  // 形式1:
  // x 0.05
  // x -0.05
  if (
    axis_string == "x" ||
    axis_string == "y" ||
    axis_string == "z")
  {
    axis = axis_string[0];
    offset = value;
  }

  // 形式2:
  // x+ 0.05
  // x- 0.05
  // y+ 0.05
  // ...
  else if (
    axis_string.size() == 2 &&
    (axis_string[0] == 'x' ||
    axis_string[0] == 'y' ||
    axis_string[0] == 'z') &&
    (axis_string[1] == '+' ||
    axis_string[1] == '-'))
  {
    axis = axis_string[0];

    const double magnitude = std::abs(value);

    if (axis_string[1] == '+') {
      offset = magnitude;
    } else {
      offset = -magnitude;
    }
  }
  else {
    throw std::runtime_error(
      "轴参数错误，只允许 x / y / z / x+ / x- / y+ / y- / z+ / z-");
  }

  if (std::abs(offset) < 1e-6) {
    throw std::runtime_error("位移量不能为 0");
  }

  if (std::abs(offset) > kMaxOffsetMetres) {
    throw std::runtime_error(
      "拒绝执行：单次位移超过安全限制 0.10 m");
  }

  return MotionCommand{axis, offset};
}


class BaseXYZCommand final : public rclcpp::Node
{
public:
  BaseXYZCommand(
    const char axis,
    const double offset)
  : Node("move_base_xyz"),
    axis_(axis),
    offset_(offset)
  {
    target_publisher_ =
      create_publisher<geometry_msgs::msg::PoseStamped>(
      "/cartesian_impedance_controller/target_pose",
      rclcpp::SystemDefaultsQoS());

    error_publisher_ =
      create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "/cartesian_impedance_controller/base_xyz_error",
      rclcpp::QoS(1).transient_local());

    pose_subscription_ =
      create_subscription<geometry_msgs::msg::PoseStamped>(
      "/cartesian_impedance_controller/current_pose",
      rclcpp::SystemDefaultsQoS(),
      std::bind(
        &BaseXYZCommand::pose_callback,
        this,
        std::placeholders::_1));

    result_timer_ =
      create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(
        &BaseXYZCommand::check_result,
        this));

    RCLCPP_INFO(
      get_logger(),
      "等待当前 TCP 位姿；计划在 base 坐标系 %c 轴运动 %+.3f m",
      axis_,
      offset_);
  }

private:
  void pose_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr pose)
  {
    if (pose->header.frame_id != "base") {
      RCLCPP_ERROR(
        get_logger(),
        "拒绝发布：当前 TCP 位姿坐标系为 '%s'，预期为 'base'",
        pose->header.frame_id.c_str());

      rclcpp::shutdown();
      return;
    }

    const rclcpp::Time received_time = now();

    // 第一次收到 TCP pose 后生成目标
    if (!sent_) {
      start_pose_ = *pose;
      latest_pose_ = *pose;
      target_pose_ = *pose;

      target_pose_.header.stamp = received_time;

      // =====================================================
      // 根据命令行指定的轴添加位移
      // =====================================================
      switch (axis_) {
        case 'x':
          target_pose_.pose.position.x += offset_;
          break;

        case 'y':
          target_pose_.pose.position.y += offset_;
          break;

        case 'z':
          target_pose_.pose.position.z += offset_;
          break;

        default:
          RCLCPP_ERROR(
            get_logger(),
            "内部错误：未知轴 %c",
            axis_);

          rclcpp::shutdown();
          return;
      }

      command_time_ = received_time;
      last_motion_time_ = received_time;

      target_publisher_->publish(target_pose_);

      sent_ = true;

      RCLCPP_INFO(
        get_logger(),
        "已发布固定目标：base-%c 方向 %+.3f m",
        axis_,
        offset_);

      RCLCPP_INFO(
        get_logger(),
        "起始位置: [%.6f, %.6f, %.6f] m",
        start_pose_.pose.position.x,
        start_pose_.pose.position.y,
        start_pose_.pose.position.z);

      RCLCPP_INFO(
        get_logger(),
        "目标位置: [%.6f, %.6f, %.6f] m",
        target_pose_.pose.position.x,
        target_pose_.pose.position.y,
        target_pose_.pose.position.z);

      return;
    }

    // 判断 TCP 是否还在明显运动
    if (
      position_distance(
        latest_pose_.pose.position,
        pose->pose.position) > kMotionThresholdMetres)
    {
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

    const double elapsed =
      (current_time - command_time_).seconds();

    const bool settled =
      elapsed >= kMinimumMotionSeconds &&
      has_measurement_after_command_ &&
      (current_time - last_motion_time_).seconds() >= kSettleSeconds;

    if (!settled && elapsed < kResultTimeoutSeconds) {
      return;
    }

    report_error(!settled);
  }


  void report_error(const bool timed_out)
  {
    const double error_x =
      target_pose_.pose.position.x -
      latest_pose_.pose.position.x;

    const double error_y =
      target_pose_.pose.position.y -
      latest_pose_.pose.position.y;

    const double error_z =
      target_pose_.pose.position.z -
      latest_pose_.pose.position.z;

    const double actual_x =
      latest_pose_.pose.position.x -
      start_pose_.pose.position.x;

    const double actual_y =
      latest_pose_.pose.position.y -
      start_pose_.pose.position.y;

    const double actual_z =
      latest_pose_.pose.position.z -
      start_pose_.pose.position.z;

    const double norm =
      std::sqrt(
      error_x * error_x +
      error_y * error_y +
      error_z * error_z);

    geometry_msgs::msg::Vector3Stamped error_message;

    error_message.header.stamp = now();
    error_message.header.frame_id = "base";

    error_message.vector.x = error_x;
    error_message.vector.y = error_y;
    error_message.vector.z = error_z;

    error_publisher_->publish(error_message);

    if (timed_out) {
      RCLCPP_WARN(
        get_logger(),
        "%c %+.3f m 测试超时：\n"
        "实际位移 = [%.3f, %.3f, %.3f] mm\n"
        "目标误差 = [%.3f, %.3f, %.3f] mm\n"
        "误差范数 = %.3f mm",
        axis_,
        offset_,
        actual_x * 1000.0,
        actual_y * 1000.0,
        actual_z * 1000.0,
        error_x * 1000.0,
        error_y * 1000.0,
        error_z * 1000.0,
        norm * 1000.0);
    } else {
      RCLCPP_INFO(
        get_logger(),
        "%c %+.3f m 测试完成：\n"
        "实际位移 = [%.3f, %.3f, %.3f] mm\n"
        "目标误差 = [%.3f, %.3f, %.3f] mm\n"
        "误差范数 = %.3f mm",
        axis_,
        offset_,
        actual_x * 1000.0,
        actual_y * 1000.0,
        actual_z * 1000.0,
        error_x * 1000.0,
        error_y * 1000.0,
        error_z * 1000.0,
        norm * 1000.0);
    }

    RCLCPP_INFO(
      get_logger(),
      "误差已发布到 "
      "/cartesian_impedance_controller/base_xyz_error");

    reported_ = true;

    rclcpp::shutdown();
  }

  // 运动参数
  char axis_;
  double offset_;

  // 状态
  bool sent_{false};
  bool reported_{false};
  bool has_measurement_after_command_{false};

  rclcpp::Time command_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_motion_time_{0, 0, RCL_ROS_TIME};

  geometry_msgs::msg::PoseStamped start_pose_;
  geometry_msgs::msg::PoseStamped target_pose_;
  geometry_msgs::msg::PoseStamped latest_pose_;

  rclcpp::Publisher<
    geometry_msgs::msg::PoseStamped>::SharedPtr
    target_publisher_;

  rclcpp::Publisher<
    geometry_msgs::msg::Vector3Stamped>::SharedPtr
    error_publisher_;

  rclcpp::Subscription<
    geometry_msgs::msg::PoseStamped>::SharedPtr
    pose_subscription_;

  rclcpp::TimerBase::SharedPtr result_timer_;
};

}  // namespace


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  try {
    // 去掉 --ros-args 等 ROS 自身参数
    const auto args =
      rclcpp::remove_ros_arguments(argc, argv);

    const MotionCommand command =
      parse_command(args);

    RCLCPP_INFO(
      rclcpp::get_logger("move_base_xyz"),
      "命令解析成功：base-%c 位移 %+.4f m (%.1f mm)",
      command.axis,
      command.offset,
      command.offset * 1000.0);

    auto node =
      std::make_shared<BaseXYZCommand>(
      command.axis,
      command.offset);

    rclcpp::spin(node);
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("move_base_xyz"),
      "%s",
      e.what());

    rclcpp::shutdown();
    return 1;
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return 0;
}
