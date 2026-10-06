#pragma once

#include <Eigen/Geometry>

namespace cartesian_impedance_controller
{
struct PoseReference
{
  Eigen::Vector3d position{ Eigen::Vector3d::Zero() };
  Eigen::Quaterniond orientation{ Eigen::Quaterniond::Identity() };
};

inline PoseReference low_pass_reference(const PoseReference& current, const PoseReference& target, const double alpha)
{
  PoseReference filtered;
  filtered.position = current.position + alpha * (target.position - current.position);
  filtered.orientation = current.orientation.slerp(alpha, target.orientation).normalized();
  return filtered;
}

}  // namespace cartesian_impedance_controller
