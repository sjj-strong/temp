#include "cartesian_impedance_controller/cartesian_impedance_controller.hpp"

#include <algorithm>
#include <cmath>

#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "cartesian_impedance_controller/wrench_transform.hpp"

namespace cartesian_impedance_controller
{
namespace
{
constexpr std::size_t kForceModeTaskFrame = 0;
constexpr std::size_t kForceModeSelection = 6;
constexpr std::size_t kForceModeWrench = 12;
constexpr std::size_t kForceModeType = 18;
constexpr std::size_t kForceModeLimits = 19;
constexpr std::size_t kForceModeAsync = 25;
constexpr std::size_t kForceModeDisable = 26;
constexpr std::size_t kForceModeDamping = 27;
constexpr std::size_t kForceModeGain = 28;

Vector6 vector_parameter(const std::vector<double>& value, const std::string& name)
{
  if (value.size() != 6) {
    throw std::runtime_error(name + " must contain exactly six values");
  }
  Vector6 result{};
  std::copy(value.begin(), value.end(), result.begin());
  return result;
}

Eigen::Isometry3d eigen_frame(const KDL::Frame& frame)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      result.matrix()(row, column) = frame.M(row, column);
    }
  }
  result.translation() = Eigen::Vector3d(frame.p.x(), frame.p.y(), frame.p.z());
  return result;
}
}  // namespace

controller_interface::CallbackReturn CartesianImpedanceController::on_init()
{
  try {
    auto& node = *get_node();
    node.declare_parameter("tf_prefix", "");
    node.declare_parameter("base_frame", "base_link");
    node.declare_parameter("tip_frame", "gripper_tcp");
    node.declare_parameter("target_topic", "~/target_pose");
    node.declare_parameter("joints", std::vector<std::string>{ "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                                                 "wrist_1_joint", "wrist_2_joint", "wrist_3_joint" });
    node.declare_parameter("stiffness", std::vector<double>{ 100.0, 100.0, 100.0, 10.0, 10.0, 10.0 });
    node.declare_parameter("max_wrench", std::vector<double>{ 40.0, 40.0, 40.0, 4.0, 4.0, 4.0 });
    node.declare_parameter("selection_vector", std::vector<bool>{ true, true, true, true, true, true });
    node.declare_parameter("speed_limits", std::vector<double>{ 0.05, 0.05, 0.05, 0.2, 0.2, 0.2 });
    node.declare_parameter("deviation_limits", std::vector<double>{ 0.01, 0.01, 0.01, 0.1, 0.1, 0.1 });
    node.declare_parameter("ur_damping", 0.025);
    node.declare_parameter("ur_gain_scaling", 0.5);
    node.declare_parameter("use_external_ft", false);
    node.declare_parameter("ft_sensor_name", "robotiq_ft_sensor");
    node.declare_parameter("ft_frame", "robotiq_ft_frame_id");
    node.declare_parameter("force_feedback_gain", 0.0);
  } catch (const std::exception& exception) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter initialization failed: %s", exception.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration CartesianImpedanceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  const std::array<std::string, 29> names{ "task_frame_x", "task_frame_y", "task_frame_z", "task_frame_rx", "task_frame_ry", "task_frame_rz",
    "selection_vector_x", "selection_vector_y", "selection_vector_z", "selection_vector_rx", "selection_vector_ry", "selection_vector_rz",
    "wrench_x", "wrench_y", "wrench_z", "wrench_rx", "wrench_ry", "wrench_rz", "type", "limits_x", "limits_y", "limits_z", "limits_rx",
    "limits_ry", "limits_rz", "force_mode_async_success", "disable_cmd", "damping", "gain_scaling" };
  for (const auto& name : names) {
    config.names.emplace_back(tf_prefix_ + "force_mode/" + name);
  }
  return config;
}

controller_interface::InterfaceConfiguration CartesianImpedanceController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joints_) {
    config.names.emplace_back(tf_prefix_ + joint + "/position");
  }
  if (use_external_ft_) {
    for (const auto* name : { "force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z" }) {
      config.names.emplace_back(tf_prefix_ + ft_sensor_name_ + "/" + name);
    }
  }
  return config;
}

controller_interface::CallbackReturn CartesianImpedanceController::on_configure(const rclcpp_lifecycle::State&)
{
  try {
    auto& node = *get_node();
    tf_prefix_ = node.get_parameter("tf_prefix").as_string();
    base_frame_ = node.get_parameter("base_frame").as_string();
    tip_frame_ = node.get_parameter("tip_frame").as_string();
    target_topic_ = node.get_parameter("target_topic").as_string();
    joints_ = node.get_parameter("joints").as_string_array();
    stiffness_ = vector_parameter(node.get_parameter("stiffness").as_double_array(), "stiffness");
    max_wrench_ = vector_parameter(node.get_parameter("max_wrench").as_double_array(), "max_wrench");
    speed_limits_ = vector_parameter(node.get_parameter("speed_limits").as_double_array(), "speed_limits");
    deviation_limits_ = vector_parameter(node.get_parameter("deviation_limits").as_double_array(), "deviation_limits");
    const auto selection = node.get_parameter("selection_vector").as_bool_array();
    if (selection.size() != selection_.size() || joints_.size() != 6) {
      throw std::runtime_error("selection_vector and joints must contain exactly six values");
    }
    std::copy(selection.begin(), selection.end(), selection_.begin());
    ur_damping_ = node.get_parameter("ur_damping").as_double();
    ur_gain_scaling_ = node.get_parameter("ur_gain_scaling").as_double();
    use_external_ft_ = node.get_parameter("use_external_ft").as_bool();
    ft_sensor_name_ = node.get_parameter("ft_sensor_name").as_string();
    ft_frame_ = node.get_parameter("ft_frame").as_string();
    force_feedback_gain_ = node.get_parameter("force_feedback_gain").as_double();
    if (!configure_kinematics()) {
      return CallbackReturn::ERROR;
    }
    target_subscription_ = node.create_subscription<geometry_msgs::msg::PoseStamped>(
        target_topic_, rclcpp::SystemDefaultsQoS(), std::bind(&CartesianImpedanceController::target_callback, this, std::placeholders::_1));
  } catch (const std::exception& exception) {
    RCLCPP_ERROR(get_node()->get_logger(), "Configuration failed: %s", exception.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

bool CartesianImpedanceController::configure_kinematics()
{
  std::string description;
  if (!get_node()->get_parameter("robot_description", description)) {
    RCLCPP_ERROR(get_node()->get_logger(), "robot_description parameter is required");
    return false;
  }
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(description, tree) || !tree.getChain(tf_prefix_ + base_frame_, tf_prefix_ + tip_frame_, tip_chain_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Cannot create KDL chain from %s to %s", base_frame_.c_str(), tip_frame_.c_str());
    return false;
  }
  tip_fk_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(tip_chain_);
  if (use_external_ft_) {
    if (!tree.getChain(tf_prefix_ + base_frame_, tf_prefix_ + ft_frame_, sensor_chain_)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Cannot create KDL chain from %s to %s", base_frame_.c_str(), ft_frame_.c_str());
      return false;
    }
    sensor_fk_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(sensor_chain_);
  }
  return true;
}

void CartesianImpedanceController::target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr message)
{
  PoseTarget target;
  target.position = Eigen::Vector3d(message->pose.position.x, message->pose.position.y, message->pose.position.z);
  target.orientation = Eigen::Quaterniond(message->pose.orientation.w, message->pose.orientation.x, message->pose.orientation.y,
                                          message->pose.orientation.z);
  if (target.orientation.norm() < 1e-8 || (!message->header.frame_id.empty() && message->header.frame_id != tf_prefix_ + base_frame_)) {
    RCLCPP_WARN(get_node()->get_logger(), "Ignoring target_pose outside base frame or with invalid quaternion");
    return;
  }
  target.orientation.normalize();
  target.valid = true;
  target_buffer_.writeFromNonRT(target);
}

controller_interface::CallbackReturn CartesianImpedanceController::on_activate(const rclcpp_lifecycle::State&)
{
  target_buffer_.writeFromNonRT(PoseTarget{});
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianImpedanceController::on_deactivate(const rclcpp_lifecycle::State&)
{
  static_cast<void>(command_interfaces_[kForceModeDisable].set_value(1.0));
  static_cast<void>(command_interfaces_[kForceModeAsync].set_value(2.0));
  return CallbackReturn::SUCCESS;
}

Vector6 CartesianImpedanceController::current_wrench_in_base()
{
  Vector6 result{};
  if (!use_external_ft_) {
    return result;
  }
  KDL::JntArray joints(joints_.size());
  for (std::size_t index = 0; index < joints_.size(); ++index) {
    joints(index) = state_interfaces_[index].get_optional().value_or(0.0);
  }
  KDL::Frame base_from_sensor;
  if (sensor_fk_->JntToCart(joints, base_from_sensor) < 0) {
    return result;
  }
  Wrench sensor_wrench{ Eigen::Vector3d(state_interfaces_[joints_.size() + 0].get_optional().value_or(0.0),
                                         state_interfaces_[joints_.size() + 1].get_optional().value_or(0.0),
                                         state_interfaces_[joints_.size() + 2].get_optional().value_or(0.0)),
                         Eigen::Vector3d(state_interfaces_[joints_.size() + 3].get_optional().value_or(0.0),
                                         state_interfaces_[joints_.size() + 4].get_optional().value_or(0.0),
                                         state_interfaces_[joints_.size() + 5].get_optional().value_or(0.0)) };
  const Wrench base_wrench = transform_wrench(eigen_frame(base_from_sensor), sensor_wrench);
  result = { base_wrench.force.x(), base_wrench.force.y(), base_wrench.force.z(), base_wrench.torque.x(), base_wrench.torque.y(), base_wrench.torque.z() };
  return result;
}

controller_interface::return_type CartesianImpedanceController::update(const rclcpp::Time&, const rclcpp::Duration&)
{
  const PoseTarget* target = target_buffer_.readFromRT();
  if (target == nullptr || !target->valid) {
    return controller_interface::return_type::OK;
  }
  KDL::JntArray joints(joints_.size());
  for (std::size_t index = 0; index < joints_.size(); ++index) {
    joints(index) = state_interfaces_[index].get_optional().value_or(0.0);
  }
  KDL::Frame base_from_tip;
  if (tip_fk_->JntToCart(joints, base_from_tip) < 0) {
    return controller_interface::return_type::ERROR;
  }
  const Eigen::Isometry3d actual = eigen_frame(base_from_tip);
  const Eigen::Quaterniond actual_orientation(actual.rotation());
  const Eigen::AngleAxisd rotation_error(target->orientation * actual_orientation.conjugate());
  Vector6 error{ target->position.x() - actual.translation().x(), target->position.y() - actual.translation().y(),
                 target->position.z() - actual.translation().z(), rotation_error.axis().x() * rotation_error.angle(),
                 rotation_error.axis().y() * rotation_error.angle(), rotation_error.axis().z() * rotation_error.angle() };
  Vector6 command = spring_wrench(error, stiffness_, max_wrench_);
  const Vector6 measured = current_wrench_in_base();
  for (std::size_t index = 0; index < command.size(); ++index) {
    command[index] = std::clamp(command[index] - force_feedback_gain_ * measured[index], -max_wrench_[index], max_wrench_[index]);
  }
  write_force_mode(command);
  return controller_interface::return_type::OK;
}

void CartesianImpedanceController::write_force_mode(const Vector6& wrench)
{
  for (std::size_t index = 0; index < 6; ++index) {
    static_cast<void>(command_interfaces_[kForceModeTaskFrame + index].set_value(0.0));
    static_cast<void>(command_interfaces_[kForceModeSelection + index].set_value(selection_[index] ? 1.0 : 0.0));
    static_cast<void>(command_interfaces_[kForceModeWrench + index].set_value(wrench[index]));
    static_cast<void>(command_interfaces_[kForceModeLimits + index].set_value(selection_[index] ? speed_limits_[index] : deviation_limits_[index]));
  }
  static_cast<void>(command_interfaces_[kForceModeType].set_value(static_cast<double>(ur_force_mode_type_)));
  static_cast<void>(command_interfaces_[kForceModeDamping].set_value(ur_damping_));
  static_cast<void>(command_interfaces_[kForceModeGain].set_value(ur_gain_scaling_));
  static_cast<void>(command_interfaces_[kForceModeAsync].set_value(2.0));
}
}  // namespace cartesian_impedance_controller

PLUGINLIB_EXPORT_CLASS(cartesian_impedance_controller::CartesianImpedanceController,
                       controller_interface::ControllerInterface)
