#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include "cartesian_impedance_controller/reference_limiter.hpp"

namespace cartesian_impedance_controller
{
TEST(ReferenceLimiter, LowPassFiltersPositionAndOrientation)
{
  PoseReference current;
  current.position = Eigen::Vector3d::Zero();
  current.orientation = Eigen::Quaterniond::Identity();

  PoseReference target;
  target.position = Eigen::Vector3d(1.0, -2.0, 3.0);
  target.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));

  const PoseReference filtered = low_pass_reference(current, target, 0.25);

  EXPECT_TRUE(filtered.position.isApprox(Eigen::Vector3d(0.25, -0.5, 0.75)));
  const Eigen::AngleAxisd filtered_orientation(filtered.orientation);
  EXPECT_NEAR(filtered_orientation.angle(), M_PI / 4.0, 1e-12);
  EXPECT_TRUE(filtered_orientation.axis().isApprox(Eigen::Vector3d::UnitZ()));
}
}  // namespace cartesian_impedance_controller
