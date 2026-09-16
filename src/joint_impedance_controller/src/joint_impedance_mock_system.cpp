#include "joint_impedance_controller/joint_impedance_mock_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace joint_impedance_controller {
namespace {
bool has_interface_suffix(const std::string &name, const std::string &suffix) {
  return name.size() >= suffix.size() &&
         name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0;
}
} // namespace

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
  position_command_.resize(info_.joints.size(), 0.0);
  for (std::size_t index = 0; index < info_.joints.size(); ++index) {
    const auto &joint = info_.joints[index];
    if (joint.command_interfaces.size() != 2 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
        joint.command_interfaces[1].name != hardware_interface::HW_IF_EFFORT ||
        joint.state_interfaces.size() != 3 ||
        joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
        joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY ||
        joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_ERROR(
          get_logger(),
          "关节 %s 的接口契约不是 position/effort 命令及 position/velocity/effort 状态",
          joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    const auto initial_value = joint.state_interfaces[0].initial_value;
    if (!initial_value.empty()) {
      position_[index] = std::stod(initial_value);
    }
    position_command_[index] = position_[index];
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
JointImpedanceMockSystem::prepare_command_mode_switch(
    const std::vector<std::string> &start_interfaces,
    const std::vector<std::string> &) {
  bool starts_position = false;
  bool starts_effort = false;
  for (const auto &interface : start_interfaces) {
    starts_position = starts_position || has_interface_suffix(
        interface, "/" + std::string(hardware_interface::HW_IF_POSITION));
    starts_effort = starts_effort || has_interface_suffix(
        interface, "/" + std::string(hardware_interface::HW_IF_EFFORT));
  }
  if (starts_position && starts_effort) {
    RCLCPP_ERROR(get_logger(), "模拟硬件不允许同时启用 position 与 effort 命令接口");
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
JointImpedanceMockSystem::perform_command_mode_switch(
    const std::vector<std::string> &start_interfaces,
    const std::vector<std::string> &) {
  for (const auto &interface : start_interfaces) {
    if (has_interface_suffix(
            interface, "/" + std::string(hardware_interface::HW_IF_POSITION))) {
      command_mode_ = CommandMode::kPosition;
      position_command_ = position_;
      for (std::size_t index = 0; index < info_.joints.size(); ++index) {
        set_command(info_.joints[index].name + "/" + hardware_interface::HW_IF_POSITION,
                    position_[index]);
      }
      std::fill(effort_command_.begin(), effort_command_.end(), 0.0);
    }
    if (has_interface_suffix(
            interface, "/" + std::string(hardware_interface::HW_IF_EFFORT))) {
      command_mode_ = CommandMode::kEffort;
      std::fill(effort_command_.begin(), effort_command_.end(), 0.0);
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn
JointImpedanceMockSystem::on_activate(const rclcpp_lifecycle::State &) {
  std::fill(velocity_.begin(), velocity_.end(), 0.0);
  std::fill(effort_state_.begin(), effort_state_.end(), 0.0);
  std::fill(effort_command_.begin(), effort_command_.end(), 0.0);
  position_command_ = position_;
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
    const auto &joint_name = info_.joints[index].name;
    if (command_mode_ == CommandMode::kPosition) {
      position_command_[index] = get_command<double>(
          joint_name + "/" + hardware_interface::HW_IF_POSITION);
      velocity_[index] = (position_command_[index] - position_[index]) / seconds;
      position_[index] = position_command_[index];
      effort_state_[index] = 0.0;
    } else {
      if (command_mode_ == CommandMode::kEffort) {
        effort_command_[index] = get_command<double>(
            joint_name + "/" + hardware_interface::HW_IF_EFFORT);
      } else {
        effort_command_[index] = 0.0;
      }
      effort_state_[index] = effort_command_[index];
      const double acceleration =
          (effort_state_[index] - viscous_damping_ * velocity_[index]) / inertia_;
      velocity_[index] = std::clamp(velocity_[index] + acceleration * seconds,
                                    -maximum_velocity_, maximum_velocity_);
      position_[index] += velocity_[index] * seconds;
    }
    set_state(joint_name + "/" + hardware_interface::HW_IF_POSITION,
              position_[index]);
    set_state(joint_name + "/" + hardware_interface::HW_IF_VELOCITY,
              velocity_[index]);
    set_state(joint_name + "/" + hardware_interface::HW_IF_EFFORT,
              effort_state_[index]);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
JointImpedanceMockSystem::write(const rclcpp::Time &,
                                const rclcpp::Duration &) {
  // 命令在 read() 中按当前已声明模式读取；未 claim 的接口不参与校验。
  return hardware_interface::return_type::OK;
}
} // namespace joint_impedance_controller

PLUGINLIB_EXPORT_CLASS(joint_impedance_controller::JointImpedanceMockSystem,
                       hardware_interface::SystemInterface)
