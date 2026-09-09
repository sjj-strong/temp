#include "joint_impedance_controller/joint_impedance_mock_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace joint_impedance_controller {
hardware_interface::CallbackReturn JointImpedanceMockSystem::on_init(
    const hardware_interface::HardwareComponentInterfaceParams &params) {
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (info_.joints.size() != 6) {
    RCLCPP_ERROR(get_logger(), "模拟硬件必须恰好包含六个关节");
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    inertia_ = std::stod(info_.hardware_parameters.at("inertia"));
    viscous_damping_ =
        std::stod(info_.hardware_parameters.at("viscous_damping"));
    maximum_velocity_ =
        std::stod(info_.hardware_parameters.at("maximum_velocity"));
  } catch (const std::exception &exception) {
    RCLCPP_ERROR(get_logger(), "读取模拟硬件参数失败：%s", exception.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!std::isfinite(inertia_) || inertia_ <= 0.0 ||
      !std::isfinite(viscous_damping_) || viscous_damping_ < 0.0 ||
      !std::isfinite(maximum_velocity_) || maximum_velocity_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), "模拟惯量、阻尼或最大速度参数无效");
    return hardware_interface::CallbackReturn::ERROR;
  }

  position_.resize(info_.joints.size(), 0.0);
  velocity_.resize(info_.joints.size(), 0.0);
  effort_state_.resize(info_.joints.size(), 0.0);
  effort_command_.resize(info_.joints.size(), 0.0);
  for (std::size_t index = 0; index < info_.joints.size(); ++index) {
    const auto &joint = info_.joints[index];
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_EFFORT ||
        joint.state_interfaces.size() != 3 ||
        joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
        joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY ||
        joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_ERROR(
          get_logger(),
          "关节 %s 的接口契约不是 effort 命令及 position/velocity/effort 状态",
          joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    const auto initial_value = joint.state_interfaces[0].initial_value;
    if (!initial_value.empty()) {
      position_[index] = std::stod(initial_value);
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
JointImpedanceMockSystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> interfaces;
  for (std::size_t index = 0; index < info_.joints.size(); ++index) {
    interfaces.emplace_back(info_.joints[index].name,
                            hardware_interface::HW_IF_POSITION,
                            &position_[index]);
    interfaces.emplace_back(info_.joints[index].name,
                            hardware_interface::HW_IF_VELOCITY,
                            &velocity_[index]);
    interfaces.emplace_back(info_.joints[index].name,
                            hardware_interface::HW_IF_EFFORT,
                            &effort_state_[index]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
JointImpedanceMockSystem::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (std::size_t index = 0; index < info_.joints.size(); ++index) {
    interfaces.emplace_back(info_.joints[index].name,
                            hardware_interface::HW_IF_EFFORT,
                            &effort_command_[index]);
  }
  return interfaces;
}

hardware_interface::CallbackReturn
JointImpedanceMockSystem::on_activate(const rclcpp_lifecycle::State &) {
  std::fill(velocity_.begin(), velocity_.end(), 0.0);
  std::fill(effort_state_.begin(), effort_state_.end(), 0.0);
  std::fill(effort_command_.begin(), effort_command_.end(), 0.0);
  RCLCPP_INFO(get_logger(), "关节阻抗模拟硬件已激活");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
JointImpedanceMockSystem::on_deactivate(const rclcpp_lifecycle::State &) {
  std::fill(effort_command_.begin(), effort_command_.end(), 0.0);
  std::fill(effort_state_.begin(), effort_state_.end(), 0.0);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
JointImpedanceMockSystem::read(const rclcpp::Time &,
                               const rclcpp::Duration &period) {
  const double seconds = period.seconds();
  if (!std::isfinite(seconds) || seconds <= 0.0) {
    return hardware_interface::return_type::OK;
  }
  for (std::size_t index = 0; index < position_.size(); ++index) {
    effort_state_[index] = effort_command_[index];
    const double acceleration =
        (effort_state_[index] - viscous_damping_ * velocity_[index]) / inertia_;
    velocity_[index] = std::clamp(velocity_[index] + acceleration * seconds,
                                  -maximum_velocity_, maximum_velocity_);
    position_[index] += velocity_[index] * seconds;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
JointImpedanceMockSystem::write(const rclcpp::Time &,
                                const rclcpp::Duration &) {
  if (!std::all_of(effort_command_.begin(), effort_command_.end(),
                   [](const double value) { return std::isfinite(value); })) {
    std::fill(effort_command_.begin(), effort_command_.end(), 0.0);
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}
} // namespace joint_impedance_controller

PLUGINLIB_EXPORT_CLASS(joint_impedance_controller::JointImpedanceMockSystem,
                       hardware_interface::SystemInterface)
