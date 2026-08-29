#pragma once

#include <Eigen/Geometry>

namespace cartesian_impedance_controller
{
struct Wrench
{
  Eigen::Vector3d force;
  Eigen::Vector3d torque;
};

/** Transform a wrench from the sensor origin into the target-frame origin. */
inline Wrench transform_wrench(const Eigen::Isometry3d& target_from_sensor, const Wrench& sensor_wrench)
{
  const Eigen::Vector3d force = target_from_sensor.rotation() * sensor_wrench.force;
  const Eigen::Vector3d torque = target_from_sensor.rotation() * sensor_wrench.torque +
                                 target_from_sensor.translation().cross(force);
  return { force, torque };
}
}  // namespace cartesian_impedance_controller
