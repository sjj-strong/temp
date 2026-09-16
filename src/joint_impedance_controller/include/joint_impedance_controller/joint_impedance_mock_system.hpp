#pragma once

#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

namespace joint_impedance_controller {
class JointImpedanceMockSystem : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(JointImpedanceMockSystem)

  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareComponentInterfaceParams &params)
      override;
  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::return_type prepare_command_mode_switch(
      const std::vector<std::string> &start_interfaces,
      const std::vector<std::string> &stop_interfaces) override;
  hardware_interface::return_type perform_command_mode_switch(
      const std::vector<std::string> &start_interfaces,
      const std::vector<std::string> &stop_interfaces) override;
  hardware_interface::return_type read(const rclcpp::Time &,
                                       const rclcpp::Duration &period) override;
  hardware_interface::return_type write(const rclcpp::Time &,
                                        const rclcpp::Duration &) override;

private:
  std::vector<double> position_;
  std::vector<double> velocity_;
  std::vector<double> effort_state_;
  std::vector<double> effort_command_;
  std::vector<double> position_command_;
  enum class CommandMode { kNone, kPosition, kEffort };
  CommandMode command_mode_{CommandMode::kNone};
  double inertia_{1.0};
  double viscous_damping_{0.2};
  double maximum_velocity_{2.0};
};
} // namespace joint_impedance_controller
