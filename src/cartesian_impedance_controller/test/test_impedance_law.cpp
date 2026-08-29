#include <gtest/gtest.h>

#include "cartesian_impedance_controller/impedance_law.hpp"

namespace cartesian_impedance_controller
{
TEST(ImpedanceLaw, LimitsPerAxisCommand)
{
  const Vector6 stiffness{ 100.0, 100.0, 100.0, 10.0, 10.0, 10.0 };
  const Vector6 max_wrench{ 50.0, 50.0, 50.0, 5.0, 5.0, 5.0 };
  const Vector6 error{ 1.0, -0.1, 0.2, 1.0, -1.0, 0.1 };

  const Vector6 command = spring_wrench(error, stiffness, max_wrench);

  EXPECT_DOUBLE_EQ(command[0], 50.0);
  EXPECT_DOUBLE_EQ(command[1], -10.0);
  EXPECT_DOUBLE_EQ(command[2], 20.0);
  EXPECT_DOUBLE_EQ(command[3], 5.0);
  EXPECT_DOUBLE_EQ(command[4], -5.0);
  EXPECT_DOUBLE_EQ(command[5], 1.0);
}

TEST(ImpedanceLaw, CombinesStiffnessDampingAndIntegralPerAxis)
{
  const Vector6 pose_error{ 0.10, -0.05, 0.00, 0.20, 0.00, -0.10 };
  const Vector6 measured_twist{ 0.20, -0.10, 0.00, 0.10, 0.00, -0.20 };
  const Vector6 desired_twist{ 0.00, 0.30, 0.00, 0.00, 0.00, 0.10 };
  const Vector6 integral_error{ 0.02, 0.00, 0.00, 0.00, 0.00, 0.00 };
  const Vector6 stiffness{ 100.0, 100.0, 100.0, 10.0, 10.0, 10.0 };
  const Vector6 damping{ 20.0, 10.0, 10.0, 2.0, 2.0, 2.0 };
  const Vector6 integral_gain{ 50.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  const Vector6 wrench_limit{ 100.0, 100.0, 100.0, 100.0, 100.0, 100.0 };

  const Vector6 wrench = cartesian_impedance_wrench(
      pose_error, measured_twist, desired_twist, integral_error, stiffness, damping, integral_gain, wrench_limit);

  EXPECT_DOUBLE_EQ(wrench[0], 7.0);
  EXPECT_DOUBLE_EQ(wrench[1], -1.0);
  EXPECT_DOUBLE_EQ(wrench[2], 0.0);
  EXPECT_DOUBLE_EQ(wrench[3], 1.8);
  EXPECT_DOUBLE_EQ(wrench[4], 0.0);
  EXPECT_DOUBLE_EQ(wrench[5], -0.4);
}

TEST(ImpedanceLaw, ClampsIntegralErrorPerAxis)
{
  const Vector6 previous{ 0.9, -0.9, 0.0, 0.0, 0.0, 0.0 };
  const Vector6 pose_error{ 2.0, -2.0, 0.5, 0.0, 0.0, 0.0 };
  const Vector6 integral_limit{ 1.0, 1.0, 0.2, 1.0, 1.0, 1.0 };

  const Vector6 integrated = integrate_error(previous, pose_error, 0.1, integral_limit);

  EXPECT_DOUBLE_EQ(integrated[0], 1.0);
  EXPECT_DOUBLE_EQ(integrated[1], -1.0);
  EXPECT_DOUBLE_EQ(integrated[2], 0.05);
}

TEST(ImpedanceLaw, AppliesAbsoluteAndRateTorqueLimits)
{
  const Vector6 desired{ 30.0, -30.0, 3.0, 0.0, 0.0, 0.0 };
  const Vector6 previous{ 8.0, -8.0, 0.0, 0.0, 0.0, 0.0 };
  const Vector6 absolute_limit{ 10.0, 10.0, 10.0, 10.0, 10.0, 10.0 };
  const Vector6 rate_limit{ 20.0, 20.0, 20.0, 20.0, 20.0, 20.0 };

  const Vector6 limited = limit_joint_torque(desired, previous, absolute_limit, rate_limit, 0.1);

  EXPECT_DOUBLE_EQ(limited[0], 10.0);
  EXPECT_DOUBLE_EQ(limited[1], -10.0);
  EXPECT_DOUBLE_EQ(limited[2], 2.0);
}
}  // namespace cartesian_impedance_controller
