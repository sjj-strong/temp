#pragma once

#include <algorithm>
#include <array>

namespace cartesian_impedance_controller
{
using Vector6 = std::array<double, 6>;

inline Vector6 spring_wrench(const Vector6& error, const Vector6& stiffness, const Vector6& max_wrench)
{
  Vector6 command{};
  for (std::size_t index = 0; index < command.size(); ++index) {
    command[index] = std::clamp(stiffness[index] * error[index], -max_wrench[index], max_wrench[index]);
  }
  return command;
}
}  // namespace cartesian_impedance_controller
