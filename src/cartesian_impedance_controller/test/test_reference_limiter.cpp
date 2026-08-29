#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include "cartesian_impedance_controller/reference_limiter.hpp"

namespace cartesian_impedance_controller
{
TEST(ReferenceLimiter, BoundsTranslationAndOrientationStep)
{
  PoseReference current;
  current.position = Eigen::Vector3d::Zero();
  current.orientation = Eigen::Quaterniond::Identity();

  PoseReference target;
  target.position = Eigen::Vector3d(1.0, 0.0, 0.0);
  target.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));

  const PoseReference limited = limit_reference_step(current, target, 0.1, 0.5, 0.2);

  EXPECT_TRUE(limited.position.isApprox(Eigen::Vector3d(0.02, 0.0, 0.0)));
  const Eigen::AngleAxisd orientation_step(limited.orientation);
  EXPECT_NEAR(orientation_step.angle(), 0.1, 1e-12);
  EXPECT_TRUE(orientation_step.axis().isApprox(Eigen::Vector3d::UnitZ()));
}

TEST(ReferenceLimiter, DerivesDesiredTwistFromSuccessiveReferences)
{
  PoseReference previous;
  previous.position = Eigen::Vector3d::Zero();
  previous.orientation = Eigen::Quaterniond::Identity();

  PoseReference current;
  current.position = Eigen::Vector3d(0.02, 0.0, 0.0);
  current.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ()));

  const Vector6 twist = reference_twist(previous, current, 0.2);

  EXPECT_DOUBLE_EQ(twist[0], 0.1);
  EXPECT_DOUBLE_EQ(twist[1], 0.0);
  EXPECT_DOUBLE_EQ(twist[2], 0.0);
  EXPECT_NEAR(twist[3], 0.0, 1e-12);
  EXPECT_NEAR(twist[4], 0.0, 1e-12);
  EXPECT_NEAR(twist[5], 0.5, 1e-12);
}
}  // namespace cartesian_impedance_controller
