#include "joint_impedance_controller/joint_impedance_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_map>

#include <pluginlib/class_list_macros.hpp>

namespace joint_impedance_controller {
namespace {
JointVector vector_parameter(const std::vector<double> &values,
                             const std::string &name) {
  if (values.size() != 6) {
    throw std::runtime_error(name + " 必须恰好包含六个数值");
  }
  JointVector result{};
  std::copy(values.begin(), values.end(), result.begin());
  return result;
}

bool finite_vector(const JointVector &values) {
  return std::all_of(values.begin(), values.end(),
                     [](const double value) { return std::isfinite(value); });
}

bool finite_nonnegative_vector(const JointVector &values) {
  return std::all_of(values.begin(), values.end(), [](const double value) {
    return std::isfinite(value) && value >= 0.0;
  });
}
} // namespace

controller_interface::CallbackReturn JointImpedanceController::on_init() {
  try {
    auto &node = *get_node();
    node.declare_parameter("tf_prefix", "");
    node.declare_parameter("target_topic", "~/target_joint_state");
    node.declare_parameter(
        "joints",
        std::vector<std::string>{"shoulder_pan_joint", "shoulder_lift_joint",
                                 "elbow_joint", "wrist_1_joint",
                                 "wrist_2_joint", "wrist_3_joint"});
    node.declare_parameter(
        "stiffness", std::vector<double>{80.0, 80.0, 60.0, 20.0, 15.0, 10.0});
    node.declare_parameter(
        "damping", std::vector<double>{18.0, 18.0, 14.0, 5.0, 4.0, 3.0});
    node.declare_parameter(
        "max_torque", std::vector<double>{40.0, 40.0, 30.0, 12.0, 10.0, 8.0});
    node.declare_parameter(
        "max_torque_rate",
        std::vector<double>{100.0, 100.0, 80.0, 40.0, 30.0, 20.0});
    node.declare_parameter("max_position_error",
                           std::vector<double>{0.5, 0.5, 0.5, 0.5, 0.5, 0.5});
    node.declare_parameter("joint_position_min",
                           std::vector<double>{-2.0 * M_PI, -2.0 * M_PI,
                                               -2.0 * M_PI, -2.0 * M_PI,
                                               -2.0 * M_PI, -2.0 * M_PI});
    node.declare_parameter("joint_position_max",
                           std::vector<double>{2.0 * M_PI, 2.0 * M_PI,
                                               2.0 * M_PI, 2.0 * M_PI,
                                               2.0 * M_PI, 2.0 * M_PI});
    node.declare_parameter("max_reference_speed",
                           std::vector<double>{0.2, 0.2, 0.2, 0.2, 0.2,
                                               0.2});
    node.declare_parameter("reference_velocity_filter_time_constant", 0.02);
    node.declare_parameter("command_timeout", 0.5);
  } catch (const std::exception &exception) {
    RCLCPP_ERROR(get_node()->get_logger(), "参数声明失败：%s",
                 exception.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
JointImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration configuration;
  configuration.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint : joints_) {
    configuration.names.emplace_back(tf_prefix_ + joint + "/effort");
  }
  return configuration;
}

controller_interface::InterfaceConfiguration
JointImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration configuration;
  configuration.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint : joints_) {
    configuration.names.emplace_back(tf_prefix_ + joint + "/position");
  }
  for (const auto &joint : joints_) {
    configuration.names.emplace_back(tf_prefix_ + joint + "/velocity");
  }
  return configuration;
}

controller_interface::CallbackReturn
JointImpedanceController::on_configure(const rclcpp_lifecycle::State &) {
  try {
    auto &node = *get_node();
    tf_prefix_ = node.get_parameter("tf_prefix").as_string();
    target_topic_ = node.get_parameter("target_topic").as_string();
    joints_ = node.get_parameter("joints").as_string_array();
    if (joints_.size() != 6) {
      throw std::runtime_error("joints 必须恰好包含六个关节名");
    }
    stiffness_ = vector_parameter(
        node.get_parameter("stiffness").as_double_array(), "stiffness");
    damping_ = vector_parameter(node.get_parameter("damping").as_double_array(),
                                "damping");
    max_torque_ = vector_parameter(
        node.get_parameter("max_torque").as_double_array(), "max_torque");
    max_torque_rate_ = vector_parameter(
        node.get_parameter("max_torque_rate").as_double_array(),
        "max_torque_rate");
    max_position_error_ = vector_parameter(
        node.get_parameter("max_position_error").as_double_array(),
        "max_position_error");
    joint_position_min_ = vector_parameter(
        node.get_parameter("joint_position_min").as_double_array(),
        "joint_position_min");
    joint_position_max_ = vector_parameter(
        node.get_parameter("joint_position_max").as_double_array(),
        "joint_position_max");
    max_reference_speed_ = vector_parameter(
        node.get_parameter("max_reference_speed").as_double_array(),
        "max_reference_speed");
    reference_velocity_filter_time_constant_ = node
        .get_parameter("reference_velocity_filter_time_constant")
        .as_double();
    command_timeout_ = node.get_parameter("command_timeout").as_double();

    if (!finite_nonnegative_vector(stiffness_) ||
        !finite_nonnegative_vector(damping_) ||
        !finite_nonnegative_vector(max_torque_) ||
        !finite_nonnegative_vector(max_torque_rate_) ||
        !finite_nonnegative_vector(max_position_error_) ||
        !finite_nonnegative_vector(max_reference_speed_) ||
        !finite_vector(joint_position_min_) ||
        !finite_vector(joint_position_max_) ||
        !std::isfinite(reference_velocity_filter_time_constant_) ||
        reference_velocity_filter_time_constant_ < 0.0 ||
        !std::isfinite(command_timeout_) || command_timeout_ < 0.0) {
      throw std::runtime_error(
          "增益、限幅、参考速度、滤波时间常数和超时参数必须为有限且合法的数值");
    }
    for (std::size_t index = 0; index < joints_.size(); ++index) {
      if (joint_position_min_[index] >= joint_position_max_[index]) {
        throw std::runtime_error("关节位置下限必须小于上限");
      }
    }

    target_subscription_ =
        node.create_subscription<sensor_msgs::msg::JointState>(
            target_topic_, rclcpp::SystemDefaultsQoS(),
            std::bind(&JointImpedanceController::target_callback, this,
                      std::placeholders::_1));
  } catch (const std::exception &exception) {
    RCLCPP_ERROR(get_node()->get_logger(), "控制器配置失败：%s",
                 exception.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

bool JointImpedanceController::read_state(JointVector &position,
                                          JointVector &velocity) const {
  if (state_interfaces_.size() != 12) {
    return false;
  }
  for (std::size_t index = 0; index < joints_.size(); ++index) {
    const auto current_position = state_interfaces_[index].get_optional();
    const auto current_velocity =
        state_interfaces_[joints_.size() + index].get_optional();
    if (!current_position || !current_velocity ||
        !std::isfinite(*current_position) ||
        !std::isfinite(*current_velocity)) {
      return false;
    }
    position[index] = *current_position;
    velocity[index] = *current_velocity;
  }
  return true;
}

bool JointImpedanceController::within_position_limits(
    const JointVector &position) const {
  for (std::size_t index = 0; index < position.size(); ++index) {
    if (position[index] < joint_position_min_[index] ||
        position[index] > joint_position_max_[index]) {
      return false;
    }
  }
  return true;
}

controller_interface::CallbackReturn
JointImpedanceController::on_activate(const rclcpp_lifecycle::State &) {
  JointVector velocity{};
  if (!read_state(reference_position_, velocity) ||
      !within_position_limits(reference_position_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "激活时无法读取有效的六关节状态");
    return CallbackReturn::ERROR;
  }
  reference_velocity_.fill(0.0);
  previous_torque_.fill(0.0);
  timed_out_ = false;
  target_buffer_.writeFromNonRT(JointTarget{});
  write_zero_torque();
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
JointImpedanceController::on_deactivate(const rclcpp_lifecycle::State &) {
  write_zero_torque();
  return CallbackReturn::SUCCESS;
}

void JointImpedanceController::target_callback(
    const sensor_msgs::msg::JointState::SharedPtr message) {
  if (message->name.size() != joints_.size() ||
      message->position.size() != joints_.size() ||
      (!message->velocity.empty() &&
       message->velocity.size() != joints_.size())) {
    RCLCPP_WARN(
        get_node()->get_logger(),
        "忽略目标：name、position 必须含六项，velocity 必须为空或含六项");
    return;
  }

  std::unordered_map<std::string, std::size_t> message_index;
  for (std::size_t index = 0; index < message->name.size(); ++index) {
    if (!message_index.emplace(message->name[index], index).second) {
      RCLCPP_WARN(get_node()->get_logger(), "忽略目标：关节名存在重复项");
      return;
    }
  }

  JointTarget target;
  for (std::size_t index = 0; index < joints_.size(); ++index) {
    const auto iterator = message_index.find(tf_prefix_ + joints_[index]);
    if (iterator == message_index.end()) {
      RCLCPP_WARN(get_node()->get_logger(), "忽略目标：缺少关节 %s",
                  (tf_prefix_ + joints_[index]).c_str());
      return;
    }
    target.position[index] = message->position[iterator->second];
    target.velocity[index] =
        message->velocity.empty() ? 0.0 : message->velocity[iterator->second];
  }
  if (!finite_vector(target.position) || !finite_vector(target.velocity) ||
      !within_position_limits(target.position)) {
    RCLCPP_WARN(get_node()->get_logger(),
                "忽略目标：目标含非有限值或超出关节限位");
    return;
  }
  // controller_manager 的更新时钟可能是 Steady
  // Clock，不能与节点的系统时间直接相减。
  target.received_time = steady_clock_.now();
  target.sequence = target_sequence_counter_.fetch_add(1) + 1;
  target.valid = true;
  target_buffer_.writeFromNonRT(target);
}

controller_interface::return_type
JointImpedanceController::update(const rclcpp::Time &,
                                 const rclcpp::Duration &period) {
  JointVector position{};
  JointVector velocity{};
  if (!read_state(position, velocity) || !within_position_limits(position)) {
    write_zero_torque();
    return controller_interface::return_type::ERROR;
  }

  const JointTarget *target = target_buffer_.readFromRT();
  const double target_age =
      target != nullptr && target->valid
          ? (steady_clock_.now() - target->received_time).seconds()
          : -1.0;
  const bool target_fresh =
      target != nullptr && target->valid && target_age >= 0.0 &&
      (command_timeout_ == 0.0 || target_age <= command_timeout_);
  JointVector requested_position = reference_position_;
  if (target_fresh) {
    requested_position = target->position;
    timed_out_ = false;
  } else if (!timed_out_) {
    reference_position_ = position;
    requested_position = position;
    reference_velocity_.fill(0.0);
    timed_out_ = true;
  }

  const double period_seconds = period.seconds();
  const JointVector previous_reference = reference_position_;
  reference_position_ = limit_reference_step(
      reference_position_, requested_position, max_reference_speed_,
      period_seconds);
  JointVector raw_reference_velocity{};
  if (std::isfinite(period_seconds) && period_seconds > 0.0) {
    for (std::size_t index = 0; index < joints_.size(); ++index) {
      raw_reference_velocity[index] =
          (reference_position_[index] - previous_reference[index]) /
          period_seconds;
    }
  }
  reference_velocity_ = filter_reference_velocity(
      reference_velocity_, raw_reference_velocity,
      reference_velocity_filter_time_constant_, period_seconds);

  JointVector position_error{};
  JointVector velocity_error{};
  for (std::size_t index = 0; index < joints_.size(); ++index) {
    position_error[index] = reference_position_[index] - position[index];
    velocity_error[index] = reference_velocity_[index] - velocity[index];
    if (!std::isfinite(position_error[index]) ||
        std::abs(position_error[index]) > max_position_error_[index]) {
      write_zero_torque();
      return controller_interface::return_type::ERROR;
    }
  }

  const JointVector desired_torque =
      impedance_torque(position_error, velocity_error, stiffness_, damping_);
  if (!finite_vector(desired_torque)) {
    write_zero_torque();
    return controller_interface::return_type::ERROR;
  }
  const JointVector bounded_torque =
      limit_torque(desired_torque, previous_torque_, max_torque_,
                   max_torque_rate_, period_seconds);
  if (!write_torque(bounded_torque)) {
    write_zero_torque();
    return controller_interface::return_type::ERROR;
  }
  previous_torque_ = bounded_torque;
  return controller_interface::return_type::OK;
}

bool JointImpedanceController::write_torque(const JointVector &torque) {
  if (command_interfaces_.size() != torque.size()) {
    return false;
  }
  bool success = true;
  for (std::size_t index = 0; index < torque.size(); ++index) {
    success &= command_interfaces_[index].set_value(torque[index]);
  }
  return success;
}

void JointImpedanceController::write_zero_torque() {
  for (auto &interface : command_interfaces_) {
    static_cast<void>(interface.set_value(0.0));
  }
  previous_torque_.fill(0.0);
}
} // namespace joint_impedance_controller

PLUGINLIB_EXPORT_CLASS(joint_impedance_controller::JointImpedanceController,
                       controller_interface::ControllerInterface)
