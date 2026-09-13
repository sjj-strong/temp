#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "joint_impedance_controller/impedance_law.hpp"

namespace joint_impedance_controller {
class JointImpedanceController
    : public controller_interface::ControllerInterface {
public:
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time &,
                                           const rclcpp::Duration &) override;

private:
  struct JointTarget {
    JointVector position{};
    JointVector velocity{};
    rclcpp::Time received_time{0, 0, RCL_STEADY_TIME};
    std::uint64_t sequence{0};
    bool valid{false};
  };

  bool read_state(JointVector &position, JointVector &velocity) const;
  bool within_position_limits(const JointVector &position) const;
  bool write_torque(const JointVector &torque);
  void write_zero_torque();
  void target_callback(const sensor_msgs::msg::JointState::SharedPtr message);

  std::string tf_prefix_;
  std::string target_topic_;
  std::vector<std::string> joints_;
  JointVector stiffness_{};
  JointVector damping_{};
  JointVector max_torque_{};
  JointVector max_torque_rate_{};
  JointVector max_position_error_{};
  JointVector joint_position_min_{};
  JointVector joint_position_max_{};
  JointVector reference_position_{};
  // 由受限参考位置差分并低通滤波得到，避免外部消息的速度字段绕过参考限速。
  JointVector reference_velocity_{};
  JointVector previous_torque_{};
  JointVector max_reference_speed_{};
  double reference_velocity_filter_time_constant_{0.02};
  double command_timeout_{0.5};
  bool timed_out_{false};
  std::atomic<std::uint64_t> target_sequence_counter_{0};
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  realtime_tools::RealtimeBuffer<JointTarget> target_buffer_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      target_subscription_;
};
} // namespace joint_impedance_controller
