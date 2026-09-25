#include "cartesian_impedance_controller/cartesian_impedance_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <Eigen/Core>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "cartesian_impedance_controller/cartesian_pose.hpp"
#include "cartesian_impedance_controller/wrench_safety.hpp"
#include "cartesian_impedance_controller/wrench_transform.hpp"

namespace cartesian_impedance_controller
{
namespace
{
Vector6 vector_parameter(const std::vector<double>& value, const std::string& name)
{
  if (value.size() != 6) {
    throw std::runtime_error(name + " must contain exactly six values");
  }
  Vector6 result{};
  std::copy(value.begin(), value.end(), result.begin());
  return result;
}

std::array<double, 6> array_parameter(const std::vector<double>& value, const std::string& name)
{
  if (value.size() != 6) {
    throw std::runtime_error(name + " must contain exactly six values");
  }
  std::array<double, 6> result{};
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

PoseReference pose_reference(const Eigen::Isometry3d& pose)
{
  return { pose.translation(), Eigen::Quaterniond(pose.rotation()) };
}

bool finite_and_nonnegative(const Vector6& values)
{
  return std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value) && value >= 0.0; });
}

bool finite_vector(const Vector6& values)
{
  return std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); });
}

double orientation_distance(const Eigen::Quaterniond& first, const Eigen::Quaterniond& second)
{
  return 2.0 * std::acos(std::clamp(std::abs(first.normalized().dot(second.normalized())), 0.0, 1.0));
}

bool chain_joint_order_matches(const KDL::Chain& chain, const std::vector<std::string>& joints, const std::string& tf_prefix)
{
  std::size_t joint_index = 0;
  for (unsigned int segment_index = 0; segment_index < chain.getNrOfSegments(); ++segment_index) {
    const KDL::Joint& joint = chain.getSegment(segment_index).getJoint();
    if (joint.getType() == KDL::Joint::None) {
      continue;
    }
    if (joint_index >= joints.size() || joint.getName() != tf_prefix + joints[joint_index]) {
      return false;
    }
    ++joint_index;
  }
  return joint_index == joints.size();
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
    node.declare_parameter("damping", std::vector<double>{ 20.0, 20.0, 20.0, 2.0, 2.0, 2.0 });
    node.declare_parameter("integral_gain", std::vector<double>{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
    node.declare_parameter("integral_limit", std::vector<double>{ 0.05, 0.05, 0.05, 0.2, 0.2, 0.2 });
    node.declare_parameter("max_wrench", std::vector<double>{ 40.0, 40.0, 40.0, 4.0, 4.0, 4.0 });
    node.declare_parameter("max_torque", std::vector<double>{ 60.0, 60.0, 60.0, 30.0, 30.0, 30.0 });
    node.declare_parameter("max_torque_rate", std::vector<double>{ 200.0, 200.0, 200.0, 100.0, 100.0, 100.0 });
    node.declare_parameter("linear_reference_speed", 0.05);
    node.declare_parameter("angular_reference_speed", 0.2);
    node.declare_parameter("reference_filter_alpha", 0.005);
    node.declare_parameter("integral_reset_position_threshold", 0.005);
    node.declare_parameter("integral_reset_orientation_threshold", 0.05);
    node.declare_parameter("use_coriolis", true);
    node.declare_parameter("workspace_min", std::vector<double>{ -1.2, -1.2, -0.1, -M_PI, -M_PI, -M_PI });
    node.declare_parameter("workspace_max", std::vector<double>{ 1.2, 1.2, 1.5, M_PI, M_PI, M_PI });
    node.declare_parameter("joint_position_min", std::vector<double>{ -2.0 * M_PI, -2.0 * M_PI, -2.0 * M_PI,
                                                                          -2.0 * M_PI, -2.0 * M_PI, -2.0 * M_PI });
    node.declare_parameter("joint_position_max", std::vector<double>{ 2.0 * M_PI, 2.0 * M_PI, 2.0 * M_PI,
                                                                          2.0 * M_PI, 2.0 * M_PI, 2.0 * M_PI });
    node.declare_parameter("use_external_ft", false);
    node.declare_parameter("ft_sensor_name", "robotiq_ft_sensor");
    node.declare_parameter("ft_frame", "robotiq_ft_frame_id");
    node.declare_parameter("max_measured_wrench", std::vector<double>{ 80.0, 80.0, 80.0, 8.0, 8.0, 8.0 });
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
  for (const auto& joint : joints_) {
    config.names.emplace_back(tf_prefix_ + joint + "/effort");
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
  for (const auto& joint : joints_) {
    config.names.emplace_back(tf_prefix_ + joint + "/velocity");
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
    if (joints_.size() != 6) {
      throw std::runtime_error("joints must contain exactly six names");
    }
    stiffness_ = vector_parameter(node.get_parameter("stiffness").as_double_array(), "stiffness");
    damping_ = vector_parameter(node.get_parameter("damping").as_double_array(), "damping");
    integral_gain_ = vector_parameter(node.get_parameter("integral_gain").as_double_array(), "integral_gain");
    integral_limit_ = vector_parameter(node.get_parameter("integral_limit").as_double_array(), "integral_limit");
    max_wrench_ = vector_parameter(node.get_parameter("max_wrench").as_double_array(), "max_wrench");
    max_torque_ = vector_parameter(node.get_parameter("max_torque").as_double_array(), "max_torque");
    max_torque_rate_ = vector_parameter(node.get_parameter("max_torque_rate").as_double_array(), "max_torque_rate");
    max_measured_wrench_ = vector_parameter(node.get_parameter("max_measured_wrench").as_double_array(), "max_measured_wrench");
    workspace_min_ = vector_parameter(node.get_parameter("workspace_min").as_double_array(), "workspace_min");
    workspace_max_ = vector_parameter(node.get_parameter("workspace_max").as_double_array(), "workspace_max");
    joint_position_min_ = array_parameter(node.get_parameter("joint_position_min").as_double_array(), "joint_position_min");
    joint_position_max_ = array_parameter(node.get_parameter("joint_position_max").as_double_array(), "joint_position_max");
    linear_reference_speed_ = node.get_parameter("linear_reference_speed").as_double();
    angular_reference_speed_ = node.get_parameter("angular_reference_speed").as_double();
    reference_filter_alpha_ = node.get_parameter("reference_filter_alpha").as_double();
    integral_reset_position_threshold_ = node.get_parameter("integral_reset_position_threshold").as_double();
    integral_reset_orientation_threshold_ = node.get_parameter("integral_reset_orientation_threshold").as_double();
    use_coriolis_ = node.get_parameter("use_coriolis").as_bool();
    use_external_ft_ = node.get_parameter("use_external_ft").as_bool();
    ft_sensor_name_ = node.get_parameter("ft_sensor_name").as_string();
    ft_frame_ = node.get_parameter("ft_frame").as_string();
    if (!finite_and_nonnegative(stiffness_) || !finite_and_nonnegative(damping_) || !finite_and_nonnegative(integral_gain_) ||
        !finite_and_nonnegative(integral_limit_) || !finite_and_nonnegative(max_wrench_) ||
        !finite_and_nonnegative(max_torque_) || !finite_and_nonnegative(max_torque_rate_) ||
        !finite_and_nonnegative(max_measured_wrench_) || !std::isfinite(linear_reference_speed_) ||
        !std::isfinite(angular_reference_speed_) || !std::isfinite(reference_filter_alpha_) ||
        !std::isfinite(integral_reset_position_threshold_) || !std::isfinite(integral_reset_orientation_threshold_) ||
        linear_reference_speed_ < 0.0 || angular_reference_speed_ < 0.0 || reference_filter_alpha_ < 0.0 ||
        reference_filter_alpha_ > 1.0 ||
        integral_reset_position_threshold_ < 0.0 || integral_reset_orientation_threshold_ < 0.0) {
      throw std::runtime_error("limits and reference speeds must be finite and non-negative; reference_filter_alpha must be within [0, 1]");
    }
    for (std::size_t index = 0; index < joints_.size(); ++index) {
      if (!std::isfinite(joint_position_min_[index]) || !std::isfinite(joint_position_max_[index]) ||
          !std::isfinite(workspace_min_[index]) || !std::isfinite(workspace_max_[index]) ||
          joint_position_min_[index] >= joint_position_max_[index] || workspace_min_[index] > workspace_max_[index]) {
        throw std::runtime_error("joint and workspace lower limits must be smaller than upper limits");
      }
    }
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
  const std::string& description = get_robot_description();
  if (description.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "robot_description parameter is required");
    return false;
  }
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(description, tree) || !tree.getChain(tf_prefix_ + base_frame_, tf_prefix_ + tip_frame_, tip_chain_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Cannot create KDL chain from %s to %s", base_frame_.c_str(), tip_frame_.c_str());
    return false;
  }
  if (tip_chain_.getNrOfJoints() != joints_.size()) {
    RCLCPP_ERROR(get_node()->get_logger(), "KDL chain has %u joints, but the controller is configured for %zu", tip_chain_.getNrOfJoints(),
                 joints_.size());
    return false;
  }
  if (!chain_joint_order_matches(tip_chain_, joints_, tf_prefix_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Configured joint names and order must exactly match the KDL chain from %s to %s",
                 base_frame_.c_str(), tip_frame_.c_str());
    return false;
  }
  tip_fk_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(tip_chain_);
  jacobian_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(tip_chain_);
  dynamics_solver_ = std::make_unique<KDL::ChainDynParam>(tip_chain_, KDL::Vector::Zero());
  position_buffer_.resize(joints_.size());
  velocity_buffer_.resize(joints_.size());
  coriolis_buffer_.resize(joints_.size());
  jacobian_buffer_.resize(joints_.size());
  if (use_external_ft_) {
    if (!tree.getChain(tf_prefix_ + base_frame_, tf_prefix_ + ft_frame_, sensor_chain_) ||
        sensor_chain_.getNrOfJoints() != joints_.size() || !chain_joint_order_matches(sensor_chain_, joints_, tf_prefix_)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Cannot create a compatible KDL sensor chain from %s to %s", base_frame_.c_str(),
                   ft_frame_.c_str());
      return false;
    }
    sensor_fk_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(sensor_chain_);
  }
  return true;
}

bool CartesianImpedanceController::read_joint_state(KDL::JntArray& position, KDL::JntArray& velocity) const
{
  if (position.rows() != joints_.size() || velocity.rows() != joints_.size()) {
    return false;
  }
  for (std::size_t index = 0; index < joints_.size(); ++index) {
    const auto q = state_interfaces_[index].get_optional();
    const auto dq = state_interfaces_[joints_.size() + index].get_optional();
    if (!q || !dq || !std::isfinite(*q) || !std::isfinite(*dq)) {
      return false;
    }
    position(index) = *q;
    velocity(index) = *dq;
  }
  return true;
}

bool CartesianImpedanceController::get_current_pose(const KDL::JntArray& position, Eigen::Isometry3d& pose) const
{
  KDL::Frame frame;
  if (tip_fk_->JntToCart(position, frame) < 0) {
    return false;
  }
  pose = eigen_frame(frame);
  return true;
}

bool CartesianImpedanceController::get_external_wrench_in_base(const KDL::JntArray& position, Vector6& wrench) const
{
  if (!use_external_ft_) {
    wrench.fill(0.0);
    return true;
  }
  KDL::Frame base_from_sensor;
  if (sensor_fk_->JntToCart(position, base_from_sensor) < 0) {
    return false;
  }
  Wrench sensor_wrench;
  for (std::size_t index = 0; index < 3; ++index) {
    const auto force = state_interfaces_[2 * joints_.size() + index].get_optional();
    const auto torque = state_interfaces_[2 * joints_.size() + 3 + index].get_optional();
    if (!force || !torque || !std::isfinite(*force) || !std::isfinite(*torque)) {
      return false;
    }
    sensor_wrench.force(index) = *force;
    sensor_wrench.torque(index) = *torque;
  }
  const Wrench base_wrench = transform_wrench(eigen_frame(base_from_sensor), sensor_wrench);
  wrench = { base_wrench.force.x(), base_wrench.force.y(), base_wrench.force.z(), base_wrench.torque.x(), base_wrench.torque.y(),
             base_wrench.torque.z() };
  return true;
}

bool CartesianImpedanceController::within_joint_and_workspace_limits(const KDL::JntArray& position,
                                                                       const Eigen::Isometry3d& pose) const
{
  for (std::size_t index = 0; index < joints_.size(); ++index) {
    if (position(index) < joint_position_min_[index] || position(index) > joint_position_max_[index]) {
      return false;
    }
  }
  return within_workspace_limits(pose.translation());
}

bool CartesianImpedanceController::within_workspace_limits(const Eigen::Vector3d& position) const
{
  for (std::size_t index = 0; index < 3; ++index) {
    if (position(static_cast<Eigen::Index>(index)) < workspace_min_[index] ||
        position(static_cast<Eigen::Index>(index)) > workspace_max_[index]) {
      return false;
    }
  }
  return true;
}

void CartesianImpedanceController::target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr message)
{
  const std::string base_frame = tf_prefix_ + base_frame_;
  if ((!message->header.frame_id.empty() && message->header.frame_id != base_frame) ||
      !std::isfinite(message->pose.position.x) || !std::isfinite(message->pose.position.y) ||
      !std::isfinite(message->pose.position.z) ||
      !within_workspace_limits(Eigen::Vector3d(message->pose.position.x, message->pose.position.y, message->pose.position.z))) {
    RCLCPP_WARN(get_node()->get_logger(), "Ignoring target_pose outside base frame or with non-finite position");
    return;
  }
  Eigen::Quaterniond orientation(message->pose.orientation.w, message->pose.orientation.x, message->pose.orientation.y,
                                 message->pose.orientation.z);
  if (!std::isfinite(orientation.w()) || !std::isfinite(orientation.x()) || !std::isfinite(orientation.y()) ||
      !std::isfinite(orientation.z()) || orientation.norm() < 1e-8) {
    RCLCPP_WARN(get_node()->get_logger(), "Ignoring target_pose with invalid quaternion");
    return;
  }
  PoseTarget target;
  target.pose.position = Eigen::Vector3d(message->pose.position.x, message->pose.position.y, message->pose.position.z);
  target.pose.orientation = orientation.normalized();
  target.sequence = target_sequence_counter_.fetch_add(1) + 1;
  target.valid = true;
  target_buffer_.writeFromNonRT(target);
}

controller_interface::CallbackReturn CartesianImpedanceController::on_activate(const rclcpp_lifecycle::State&)
{
  Eigen::Isometry3d pose;
  if (!read_joint_state(position_buffer_, velocity_buffer_) || !get_current_pose(position_buffer_, pose)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Cannot read a valid state while activating Cartesian impedance control");
    return CallbackReturn::ERROR;
  }
  reference_pose_ = pose_reference(pose);
  reference_target_pose_ = reference_pose_;
  integral_error_.fill(0.0);
  previous_torque_.fill(0.0);
  last_target_sequence_ = 0;
  has_last_target_pose_ = false;
  target_buffer_.writeFromNonRT(PoseTarget{});
  write_zero_torque();
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianImpedanceController::on_deactivate(const rclcpp_lifecycle::State&)
{
  write_zero_torque();
  integral_error_.fill(0.0);
  previous_torque_.fill(0.0);
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianImpedanceController::update(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  Eigen::Isometry3d current_pose;
  if (!read_joint_state(position_buffer_, velocity_buffer_) || !get_current_pose(position_buffer_, current_pose) ||
      !within_joint_and_workspace_limits(position_buffer_, current_pose)) {
    write_zero_torque();
    return controller_interface::return_type::ERROR;
  }

  const PoseTarget* target = target_buffer_.readFromRT();
  if (target != nullptr && target->valid && target->sequence != last_target_sequence_) {
    if (!has_last_target_pose_ || (target->pose.position - last_target_pose_.position).norm() > integral_reset_position_threshold_ ||
        orientation_distance(target->pose.orientation, last_target_pose_.orientation) > integral_reset_orientation_threshold_) {
      integral_error_.fill(0.0);
    }
    reference_target_pose_ = target->pose;
    last_target_pose_ = target->pose;
    has_last_target_pose_ = true;
    last_target_sequence_ = target->sequence;
  }

  const double period_seconds = period.seconds();
  const PoseReference previous_reference = reference_pose_;
  const PoseReference filtered_reference = low_pass_reference(reference_pose_, reference_target_pose_, reference_filter_alpha_);
  reference_pose_ = limit_reference_step(reference_pose_, filtered_reference, linear_reference_speed_, angular_reference_speed_,
                                         period_seconds);
  if (!within_workspace_limits(reference_pose_.position)) {
    write_zero_torque();
    return controller_interface::return_type::ERROR;
  }
  Eigen::Isometry3d target_pose = Eigen::Isometry3d::Identity();
  target_pose.translation() = reference_pose_.position;
  target_pose.linear() = reference_pose_.orientation.toRotationMatrix();
  const Vector6 pose_error = cartesian_pose_error(current_pose, target_pose);
  if (jacobian_solver_->JntToJac(position_buffer_, jacobian_buffer_) < 0) {
    write_zero_torque();
    return controller_interface::return_type::ERROR;
  }
  cartesian_velocity_buffer_.noalias() = jacobian_buffer_.data * velocity_buffer_.data;
  Vector6 measured_twist{};
  for (std::size_t index = 0; index < measured_twist.size(); ++index) {
    measured_twist[index] = cartesian_velocity_buffer_(static_cast<Eigen::Index>(index));
  }
  const Vector6 desired_twist = reference_twist(previous_reference, reference_pose_, period_seconds);
  const Vector6 zero_integral{};
  const Vector6 wrench_without_integral = cartesian_impedance_wrench_unbounded(
      pose_error, measured_twist, desired_twist, zero_integral, stiffness_, damping_, zero_integral);
  integral_error_ = integrate_error_with_antiwindup(integral_error_, pose_error, period_seconds, integral_limit_,
                                                     wrench_without_integral, integral_gain_, max_wrench_);
  const Vector6 task_wrench = cartesian_impedance_wrench(pose_error, measured_twist, desired_twist, integral_error_, stiffness_,
                                                          damping_, integral_gain_, max_wrench_);
  if (!finite_vector(task_wrench)) {
    write_zero_torque();
    return controller_interface::return_type::ERROR;
  }

  Vector6 external_wrench{};
  if (!get_external_wrench_in_base(position_buffer_, external_wrench) ||
      (use_external_ft_ && !wrench_within_limits(external_wrench, max_measured_wrench_))) {
    write_zero_torque();
    return controller_interface::return_type::ERROR;
  }

  const Eigen::Map<const Eigen::Matrix<double, 6, 1>> task_wrench_eigen(task_wrench.data());
  desired_torque_buffer_.noalias() = jacobian_buffer_.data.transpose() * task_wrench_eigen;
  if (use_coriolis_) {
    if (dynamics_solver_->JntToCoriolis(position_buffer_, velocity_buffer_, coriolis_buffer_) < 0) {
      write_zero_torque();
      return controller_interface::return_type::ERROR;
    }
    desired_torque_buffer_ += coriolis_buffer_.data;
  }
  Vector6 desired_torque_array{};
  for (std::size_t index = 0; index < desired_torque_array.size(); ++index) {
    desired_torque_array[index] = desired_torque_buffer_(static_cast<Eigen::Index>(index));
  }
  if (!finite_vector(desired_torque_array)) {
    write_zero_torque();
    return controller_interface::return_type::ERROR;
  }
  const Vector6 bounded_torque = limit_joint_torque(desired_torque_array, previous_torque_, max_torque_, max_torque_rate_, period_seconds);
  if (!write_joint_torque(bounded_torque)) {
    write_zero_torque();
    return controller_interface::return_type::ERROR;
  }
  previous_torque_ = bounded_torque;
  return controller_interface::return_type::OK;
}

void CartesianImpedanceController::write_zero_torque()
{
  for (auto& command_interface : command_interfaces_) {
    static_cast<void>(command_interface.set_value(0.0));
  }
  previous_torque_.fill(0.0);
}

bool CartesianImpedanceController::write_joint_torque(const Vector6& torque)
{
  if (command_interfaces_.size() != torque.size()) {
    return false;
  }
  bool success = true;
  for (std::size_t index = 0; index < torque.size(); ++index) {
    success &= command_interfaces_[index].set_value(torque[index]);
  }
  return success;
}
}  // namespace cartesian_impedance_controller

PLUGINLIB_EXPORT_CLASS(cartesian_impedance_controller::CartesianImpedanceController,
                       controller_interface::ControllerInterface)
