#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

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
}  // namespace cartesian_impedance_controller
