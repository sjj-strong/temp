#include <gtest/gtest.h>

#include "cartesian_impedance_controller/wrench_safety.hpp"

namespace cartesian_impedance_controller
{
TEST(WrenchSafety, RejectsAForceAboveItsConfiguredLimit)
{
  const Vector6 measured{ 41.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  const Vector6 limit{ 40.0, 40.0, 40.0, 4.0, 4.0, 4.0 };

  EXPECT_FALSE(wrench_within_limits(measured, limit));
}
}  // namespace cartesian_impedance_controller
