#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

#include "cartesian_impedance_controller/impedance_law.hpp"

namespace cartesian_impedance_controller
{
struct PoseReference
{
  Eigen::Vector3d position{ Eigen::Vector3d::Zero() };
  Eigen::Quaterniond orientation{ Eigen::Quaterniond::Identity() };
};

inline PoseReference limit_reference_step(const PoseReference& current, const PoseReference& target,
                                          const double linear_speed_limit, const double angular_speed_limit,
                                          const double period_seconds)
{
  PoseReference result = current;
  const double period = std::max(0.0, std::isfinite(period_seconds) ? period_seconds : 0.0);
  const double linear_step = std::abs(linear_speed_limit) * period;
  const double angular_step = std::abs(angular_speed_limit) * period;

  const Eigen::Vector3d displacement = target.position - current.position;
  const double displacement_norm = displacement.norm();
  result.position = displacement_norm > linear_step && displacement_norm > 0.0
                        ? current.position + displacement * (linear_step / displacement_norm)
                        : target.position;

  Eigen::Quaterniond current_orientation = current.orientation.normalized();
  Eigen::Quaterniond target_orientation = target.orientation.normalized();
  if (current_orientation.dot(target_orientation) < 0.0) {
    target_orientation.coeffs() *= -1.0;
  }
  const double cosine = std::clamp(current_orientation.dot(target_orientation), -1.0, 1.0);
  const double angular_distance = 2.0 * std::acos(cosine);
  const double interpolation = angular_distance > angular_step && angular_distance > 0.0
                                 ? angular_step / angular_distance
                                 : 1.0;
  result.orientation = current_orientation.slerp(interpolation, target_orientation).normalized();
  return result;
}

inline PoseReference low_pass_reference(const PoseReference& current, const PoseReference& target, const double alpha)
{
  PoseReference filtered;
  filtered.position = current.position + alpha * (target.position - current.position);
  filtered.orientation = current.orientation.slerp(alpha, target.orientation).normalized();
  return filtered;
}

inline Vector6 reference_twist(const PoseReference& previous, const PoseReference& current, const double period_seconds)
{
  Vector6 result{};
  if (!std::isfinite(period_seconds) || period_seconds <= 0.0) {
    return result;
  }
  const Eigen::Vector3d linear_velocity = (current.position - previous.position) / period_seconds;
  Eigen::Quaterniond previous_orientation = previous.orientation.normalized();
  Eigen::Quaterniond current_orientation = current.orientation.normalized();
  if (previous_orientation.dot(current_orientation) < 0.0) {
    current_orientation.coeffs() *= -1.0;
  }
  Eigen::Quaterniond delta = current_orientation * previous_orientation.conjugate();
  delta.normalize();
  if (delta.w() < 0.0) {
    delta.coeffs() *= -1.0;
  }
  const Eigen::AngleAxisd angular_delta(delta);
  const Eigen::Vector3d angular_velocity = angular_delta.axis() * angular_delta.angle() / period_seconds;
  return { linear_velocity.x(), linear_velocity.y(), linear_velocity.z(), angular_velocity.x(), angular_velocity.y(),
           angular_velocity.z() };
}
}  // namespace cartesian_impedance_controller
