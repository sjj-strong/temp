#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include "cartesian_impedance_controller/wrench_transform.hpp"

namespace cartesian_impedance_controller
{
TEST(WrenchTransform, RotatesForceAndTorque)
{
  const Eigen::Isometry3d target_from_sensor =
      Eigen::Translation3d::Identity() * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ());
  const Wrench sensor_wrench{ Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY() };

  const Wrench result = transform_wrench(target_from_sensor, sensor_wrench);

  EXPECT_TRUE(result.force.isApprox(Eigen::Vector3d::UnitY()));
  EXPECT_TRUE(result.torque.isApprox(-Eigen::Vector3d::UnitX()));
}

TEST(WrenchTransform, TranslatesTorqueReferencePoint)
{
  Eigen::Isometry3d target_from_sensor = Eigen::Isometry3d::Identity();
  target_from_sensor.translation() = Eigen::Vector3d(0.0, 0.1, 0.0);
  const Wrench sensor_wrench{ Eigen::Vector3d(10.0, 0.0, 0.0), Eigen::Vector3d::Zero() };

  const Wrench result = transform_wrench(target_from_sensor, sensor_wrench);

  EXPECT_TRUE(result.force.isApprox(sensor_wrench.force));
  EXPECT_TRUE(result.torque.isApprox(Eigen::Vector3d(0.0, 0.0, -1.0)));
}
}  // namespace cartesian_impedance_controller
