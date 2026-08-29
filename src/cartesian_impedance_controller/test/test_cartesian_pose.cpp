#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include "cartesian_impedance_controller/cartesian_pose.hpp"

namespace cartesian_impedance_controller
{
TEST(CartesianPose, ComputesBaseFrameTranslationAndLogOrientationError)
{
  const Eigen::Isometry3d current = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  target.translation() = Eigen::Vector3d(0.1, -0.2, 0.3);
  target.linear() = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  const Vector6 error = cartesian_pose_error(current, target);

  EXPECT_DOUBLE_EQ(error[0], 0.1);
  EXPECT_DOUBLE_EQ(error[1], -0.2);
  EXPECT_DOUBLE_EQ(error[2], 0.3);
  EXPECT_NEAR(error[3], 0.0, 1e-12);
  EXPECT_NEAR(error[4], 0.0, 1e-12);
  EXPECT_NEAR(error[5], M_PI_2, 1e-12);
}
}  // namespace cartesian_impedance_controller
