#include "cartesian_impedance_controller/cartesian_impedance_controller.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <filesystem>
#include <sstream>
#include <stdexcept>

#include <Eigen/Core>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "cartesian_impedance_controller/cartesian_pose.hpp"

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
    node.declare_parameter("tip_frame", "tool0");
    node.declare_parameter("target_topic", "~/target_pose");
    node.declare_parameter("joints", std::vector<std::string>{ "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                                                  "wrist_1_joint", "wrist_2_joint", "wrist_3_joint" });
    node.declare_parameter("stiffness", std::vector<double>{ 100.0, 100.0, 100.0, 10.0, 10.0, 10.0 });
    node.declare_parameter("damping", std::vector<double>{ 20.0, 20.0, 20.0, 2.0, 2.0, 2.0 });
    node.declare_parameter("max_pose_error", std::vector<double>{ 0.02, 0.02, 0.02, 0.10, 0.10, 0.10 });
    node.declare_parameter("max_wrench", std::vector<double>{ 40.0, 40.0, 40.0, 4.0, 4.0, 4.0 });
    node.declare_parameter("max_torque", std::vector<double>{ 60.0, 60.0, 60.0, 30.0, 30.0, 30.0 });
    node.declare_parameter("reference_filter_alpha", 0.1);
    node.declare_parameter("save_debug_log", false);
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
    max_pose_error_ = vector_parameter(node.get_parameter("max_pose_error").as_double_array(), "max_pose_error");
    max_wrench_ = vector_parameter(node.get_parameter("max_wrench").as_double_array(), "max_wrench");
    max_torque_ = vector_parameter(node.get_parameter("max_torque").as_double_array(), "max_torque");
    reference_filter_alpha_ = node.get_parameter("reference_filter_alpha").as_double();
    if (!finite_and_nonnegative(stiffness_) || !finite_and_nonnegative(damping_) ||
        !finite_and_nonnegative(max_pose_error_) || !finite_and_nonnegative(max_wrench_) ||
        !finite_and_nonnegative(max_torque_) || !std::isfinite(reference_filter_alpha_) ||
        reference_filter_alpha_ < 0.0 || reference_filter_alpha_ > 1.0) {
      throw std::runtime_error("增益和限幅必须为有限非负数，reference_filter_alpha 必须位于 [0, 1]");
    }
    if (!configure_kinematics()) {
      return CallbackReturn::ERROR;
    }
    // 目标是离散的控制命令，必须可靠送达；不要使用 SystemDefaultsQoS，
    // 否则在部分 DDS 配置中会退化成 BEST_EFFORT，导致单次 ros2 topic pub 丢失。
    target_subscription_ = node.create_subscription<geometry_msgs::msg::PoseStamped>(
        target_topic_, rclcpp::QoS(10).reliable(),
        std::bind(&CartesianImpedanceController::target_callback, this, std::placeholders::_1));
    current_pose_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(
        node.create_publisher<geometry_msgs::msg::PoseStamped>("~/current_pose", rclcpp::SystemDefaultsQoS()));
    debug_timer_.reset();
    debug_csv_.reset();
    if (node.get_parameter("save_debug_log").as_bool()) {
      const std::filesystem::path directory("/ros2_ws/log/cartesian_impedance_controller");
      std::filesystem::create_directories(directory);
      const auto stamp = std::chrono::system_clock::now().time_since_epoch();
      const auto filename = "debug-" + std::to_string(
          std::chrono::duration_cast<std::chrono::nanoseconds>(stamp).count()) + ".csv";
      const auto path = directory / filename;
      std::ostringstream metadata;
      metadata << "# frame=" << tf_prefix_ + base_frame_ << ",tip=" << tf_prefix_ + tip_frame_
               << ",reference_filter_alpha=" << reference_filter_alpha_;
      for (const auto& entry : { std::make_pair("stiffness", stiffness_), std::make_pair("damping", damping_),
                                std::make_pair("max_pose_error", max_pose_error_),
                                std::make_pair("max_wrench", max_wrench_), std::make_pair("max_torque", max_torque_) }) {
        metadata << "\n# " << entry.first;
        for (double value : entry.second) { metadata << ',' << value; }
      }
      debug_csv_ = std::make_unique<DebugCsv>(path.string(), metadata.str());
      debug_timer_ = node.create_wall_timer(std::chrono::milliseconds(200), [this]() {
        try {
          debug_csv_->flush_latest();
        } catch (const std::exception& exception) {
          RCLCPP_ERROR(get_node()->get_logger(), "%s；停止保存调试日志", exception.what());
          debug_timer_->cancel();
        }
      });
      RCLCPP_WARN(node.get_logger(), "阻抗调试日志保存路径：%s（5 Hz）", path.c_str());
    }
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

void CartesianImpedanceController::target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr message)
{
  const std::string base_frame = tf_prefix_ + base_frame_;
  if ((!message->header.frame_id.empty() && message->header.frame_id != base_frame) ||
      !std::isfinite(message->pose.position.x) || !std::isfinite(message->pose.position.y) ||
      !std::isfinite(message->pose.position.z)) {
    RCLCPP_WARN(get_node()->get_logger(), "忽略坐标系不符或位置非有限的目标位姿");
    return;
  }
  Eigen::Quaterniond orientation(message->pose.orientation.w, message->pose.orientation.x, message->pose.orientation.y,
                                 message->pose.orientation.z);
  if (!std::isfinite(orientation.w()) || !std::isfinite(orientation.x()) || !std::isfinite(orientation.y()) ||
      !std::isfinite(orientation.z()) || !std::isfinite(orientation.norm()) || orientation.norm() < 1e-8) {
    RCLCPP_WARN(get_node()->get_logger(), "Ignoring target_pose with invalid quaternion");
    return;
  }
  PoseTarget target;
  target.pose.position = Eigen::Vector3d(message->pose.position.x, message->pose.position.y, message->pose.position.z);
  target.pose.orientation = orientation.normalized();
  target.sequence = target_sequence_counter_.fetch_add(1) + 1;
  target.valid = true;
  target_buffer_.writeFromNonRT(target);
  // 订阅回调不在实时 update 环内；限频记录真正通过校验并写入目标缓冲区的位姿。
  RCLCPP_INFO_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 200,
      "已接收目标 pose 坐标系=%s xyz=(%.4f,%.4f,%.4f) xyzw=(%.4f,%.4f,%.4f,%.4f) 序号=%llu",
      base_frame.c_str(), target.pose.position.x(), target.pose.position.y(), target.pose.position.z(),
      target.pose.orientation.x(), target.pose.orientation.y(), target.pose.orientation.z(),
      target.pose.orientation.w(), static_cast<unsigned long long>(target.sequence));
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
  last_target_sequence_ = 0;
  target_buffer_.writeFromNonRT(PoseTarget{});
  write_zero_torque();
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianImpedanceController::on_deactivate(const rclcpp_lifecycle::State&)
{
  write_zero_torque();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianImpedanceController::update(const rclcpp::Time& time, const rclcpp::Duration&)
{
  Eigen::Isometry3d current_pose;
  if (!read_joint_state(position_buffer_, velocity_buffer_) || !get_current_pose(position_buffer_, current_pose)) {
    write_zero_torque();
    return controller_interface::return_type::ERROR;
  }

  if (current_pose_publisher_ && current_pose_publisher_->trylock()) {
    auto& message = current_pose_publisher_->msg_;
    const Eigen::Quaterniond orientation(current_pose.rotation());
    message.header.stamp = time;
    message.header.frame_id = tf_prefix_ + base_frame_;
    message.pose.position.x = current_pose.translation().x();
    message.pose.position.y = current_pose.translation().y();
    message.pose.position.z = current_pose.translation().z();
    message.pose.orientation.x = orientation.x();
    message.pose.orientation.y = orientation.y();
    message.pose.orientation.z = orientation.z();
    message.pose.orientation.w = orientation.w();
    current_pose_publisher_->unlockAndPublish();
  }

  const PoseTarget* target = target_buffer_.readFromRT();
  if (target != nullptr && target->valid && target->sequence != last_target_sequence_) {
    reference_target_pose_ = target->pose;
    last_target_sequence_ = target->sequence;
  }

  reference_pose_ = low_pass_reference(reference_pose_, reference_target_pose_, reference_filter_alpha_);
  Eigen::Isometry3d target_pose = Eigen::Isometry3d::Identity();
  target_pose.translation() = reference_pose_.position;
  target_pose.linear() = reference_pose_.orientation.toRotationMatrix();
  const Vector6 raw_pose_error = cartesian_pose_error(current_pose, target_pose);
  Vector6 pose_error = raw_pose_error;
  for (std::size_t index = 0; index < pose_error.size(); ++index) {
    pose_error[index] = std::clamp(pose_error[index], -max_pose_error_[index], max_pose_error_[index]);
  }
  if (jacobian_solver_->JntToJac(position_buffer_, jacobian_buffer_) < 0) {
    write_zero_torque();
    return controller_interface::return_type::ERROR;
  }
  cartesian_velocity_buffer_.noalias() = jacobian_buffer_.data * velocity_buffer_.data;
  Vector6 measured_twist{};
  for (std::size_t index = 0; index < measured_twist.size(); ++index) {
    measured_twist[index] = cartesian_velocity_buffer_(static_cast<Eigen::Index>(index));
  }
  Vector6 spring{}, damper{}, raw_task_wrench{}, task_wrench{};
  for (std::size_t index = 0; index < task_wrench.size(); ++index) {
    spring[index] = stiffness_[index] * pose_error[index];
    damper[index] = -damping_[index] * measured_twist[index];
    const double raw_wrench = spring[index] + damper[index];
    raw_task_wrench[index] = raw_wrench;
    task_wrench[index] = std::clamp(raw_wrench, -max_wrench_[index], max_wrench_[index]);
  }
  if (!finite_vector(task_wrench)) {
    write_zero_torque();
    return controller_interface::return_type::ERROR;
  }

  const Eigen::Map<const Eigen::Matrix<double, 6, 1>> task_wrench_eigen(task_wrench.data());
  desired_torque_buffer_.noalias() = jacobian_buffer_.data.transpose() * task_wrench_eigen;
  if (dynamics_solver_->JntToCoriolis(position_buffer_, velocity_buffer_, coriolis_buffer_) < 0) {
    write_zero_torque();
    return controller_interface::return_type::ERROR;
  }
  desired_torque_buffer_ += coriolis_buffer_.data;
  Vector6 desired_torque_array{};
  for (std::size_t index = 0; index < desired_torque_array.size(); ++index) {
    desired_torque_array[index] = desired_torque_buffer_(static_cast<Eigen::Index>(index));
  }
  if (!finite_vector(desired_torque_array)) {
    write_zero_torque();
    return controller_interface::return_type::ERROR;
  }
  Vector6 bounded_torque{};
  for (std::size_t index = 0; index < bounded_torque.size(); ++index) {
    bounded_torque[index] = std::clamp(desired_torque_array[index], -max_torque_[index], max_torque_[index]);
  }
  const bool write_ok = write_joint_torque(bounded_torque);
  if (debug_csv_) {
    DebugSample sample;
    sample.time_ns = time.nanoseconds();
    sample.target_sequence = last_target_sequence_;
    const auto copy_pose = [](const PoseReference& pose) {
      return std::array<double, 7>{ pose.position.x(), pose.position.y(), pose.position.z(),
          pose.orientation.x(), pose.orientation.y(), pose.orientation.z(), pose.orientation.w() };
    };
    sample.current = copy_pose(pose_reference(current_pose));
    sample.target = copy_pose(reference_target_pose_);
    sample.reference = copy_pose(reference_pose_);
    sample.error_raw = raw_pose_error;
    sample.error = pose_error;
    sample.twist = measured_twist;
    sample.spring = spring;
    sample.damper = damper;
    sample.wrench_raw = raw_task_wrench;
    sample.wrench = task_wrench;
    for (std::size_t i = 0; i < 6; ++i) {
      sample.coriolis[i] = coriolis_buffer_(i);
      sample.torque_task[i] = desired_torque_array[i] - sample.coriolis[i];
    }
    sample.torque_raw = desired_torque_array;
    sample.torque = bounded_torque;
    sample.write_ok = write_ok;
    debug_csv_->capture(sample);
  }
  if (!write_ok) {
    write_zero_torque();
    return controller_interface::return_type::ERROR;
  }
  return controller_interface::return_type::OK;
}

void CartesianImpedanceController::write_zero_torque()
{
  for (auto& command_interface : command_interfaces_) {
    static_cast<void>(command_interface.set_value(0.0));
  }
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
