#pragma once

#include <cmath>

#include <Eigen/Geometry>

#include "cartesian_impedance_controller/impedance_law.hpp"

namespace cartesian_impedance_controller
{
inline Vector6 cartesian_pose_error(const Eigen::Isometry3d& current, const Eigen::Isometry3d& target)
{
  const Eigen::Vector3d translation_error = target.translation() - current.translation();
  Eigen::Quaterniond rotation_error(target.rotation() * current.rotation().transpose());
  rotation_error.normalize();
  if (rotation_error.w() < 0.0) {
    rotation_error.coeffs() *= -1.0;
  }
  const Eigen::AngleAxisd angle_axis(rotation_error);
  const Eigen::Vector3d orientation_error = angle_axis.axis() * angle_axis.angle();
  return { translation_error.x(), translation_error.y(), translation_error.z(), orientation_error.x(),
           orientation_error.y(), orientation_error.z() };
}
}  // namespace cartesian_impedance_controller
