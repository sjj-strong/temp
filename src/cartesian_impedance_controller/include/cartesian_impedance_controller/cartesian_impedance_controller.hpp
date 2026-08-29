#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>

#include "cartesian_impedance_controller/impedance_law.hpp"
#include "cartesian_impedance_controller/reference_limiter.hpp"

namespace cartesian_impedance_controller
{
class CartesianImpedanceController : public controller_interface::ControllerInterface
{
public:
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time&, const rclcpp::Duration&) override;

private:
  struct PoseTarget
  {
    PoseReference pose;
    rclcpp::Time received_time{ 0, 0, RCL_ROS_TIME };
    std::uint64_t sequence{ 0 };
    bool valid{ false };
  };

  bool configure_kinematics();
  bool read_joint_state(KDL::JntArray& position, KDL::JntArray& velocity) const;
  bool get_current_pose(const KDL::JntArray& position, Eigen::Isometry3d& pose) const;
  bool get_external_wrench_in_base(const KDL::JntArray& position, Vector6& wrench) const;
  bool within_joint_and_workspace_limits(const KDL::JntArray& position, const Eigen::Isometry3d& pose) const;
  void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr message);
  void write_zero_torque();
  bool write_joint_torque(const Vector6& torque);

  std::string tf_prefix_;
  std::string base_frame_;
  std::string tip_frame_;
  std::string ft_frame_;
  std::string ft_sensor_name_;
  std::string target_topic_;
  std::vector<std::string> joints_;

  Vector6 stiffness_{};
  Vector6 damping_{};
  Vector6 integral_gain_{};
  Vector6 integral_limit_{};
  Vector6 max_wrench_{};
  Vector6 max_torque_{};
  Vector6 max_torque_rate_{};
  Vector6 max_measured_wrench_{};
  Vector6 workspace_min_{};
  Vector6 workspace_max_{};
  std::array<double, 6> joint_position_min_{};
  std::array<double, 6> joint_position_max_{};

  bool use_coriolis_{ true };
  bool use_external_ft_{ false };
  double linear_reference_speed_{ 0.05 };
  double angular_reference_speed_{ 0.2 };
  double command_timeout_{ 0.5 };

  KDL::Chain tip_chain_;
  KDL::Chain sensor_chain_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> tip_fk_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> sensor_fk_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_solver_;
  std::unique_ptr<KDL::ChainDynParam> dynamics_solver_;

  PoseReference reference_pose_;
  PoseReference hold_pose_;
  Vector6 integral_error_{};
  Vector6 previous_torque_{};
  std::uint64_t last_target_sequence_{ 0 };
  bool timed_out_{ false };
  std::atomic<std::uint64_t> target_sequence_counter_{ 0 };
  realtime_tools::RealtimeBuffer<PoseTarget> target_buffer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_subscription_;
};
}  // namespace cartesian_impedance_controller
