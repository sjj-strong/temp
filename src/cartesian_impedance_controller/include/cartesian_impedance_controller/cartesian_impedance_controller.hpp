#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>

#include "cartesian_impedance_controller/impedance_law.hpp"

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
    Eigen::Vector3d position{ Eigen::Vector3d::Zero() };
    Eigen::Quaterniond orientation{ Eigen::Quaterniond::Identity() };
    bool valid{ false };
  };

  bool configure_kinematics();
  void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr message);
  Vector6 current_wrench_in_base();
  void write_force_mode(const Vector6& wrench);

  std::string tf_prefix_;
  std::string base_frame_;
  std::string tip_frame_;
  std::string ft_frame_;
  std::string ft_sensor_name_;
  std::string target_topic_;
  std::vector<std::string> joints_;
  Vector6 stiffness_{};
  Vector6 max_wrench_{};
  Vector6 speed_limits_{};
  Vector6 deviation_limits_{};
  std::array<bool, 6> selection_{};
  bool use_external_ft_{ false };
  double force_feedback_gain_{ 0.0 };
  double ur_damping_{ 0.025 };
  double ur_gain_scaling_{ 0.5 };
  int ur_force_mode_type_{ 2 };

  KDL::Chain tip_chain_;
  KDL::Chain sensor_chain_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> tip_fk_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> sensor_fk_;
  realtime_tools::RealtimeBuffer<PoseTarget> target_buffer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_subscription_;
};
}  // namespace cartesian_impedance_controller
