#pragma once

#include <cmath>

#include "cartesian_impedance_controller/impedance_law.hpp"

namespace cartesian_impedance_controller
{
inline bool wrench_within_limits(const Vector6& wrench, const Vector6& limit)
{
  for (std::size_t index = 0; index < wrench.size(); ++index) {
    if (!std::isfinite(wrench[index]) || std::abs(wrench[index]) > limit[index]) {
      return false;
    }
  }
  return true;
}
}  // namespace cartesian_impedance_controller
