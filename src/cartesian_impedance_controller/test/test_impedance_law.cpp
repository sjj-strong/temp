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
}  // namespace cartesian_impedance_controller
