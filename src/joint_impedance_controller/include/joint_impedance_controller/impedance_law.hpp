#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace joint_impedance_controller {
using JointVector = std::array<double, 6>;

inline JointVector impedance_torque(const JointVector &position_error,
                                    const JointVector &velocity_error,
                                    const JointVector &stiffness,
                                    const JointVector &damping) {
  JointVector torque{};
  for (std::size_t index = 0; index < torque.size(); ++index) {
    torque[index] = stiffness[index] * position_error[index] +
                    damping[index] * velocity_error[index];
  }
  return torque;
}

inline JointVector limit_torque(const JointVector &desired,
                                const JointVector &previous,
                                const JointVector &maximum,
                                const JointVector &maximum_rate,
                                const double period_seconds) {
  JointVector bounded{};
  const double safe_period = std::max(0.0, period_seconds);
  for (std::size_t index = 0; index < bounded.size(); ++index) {
    const double absolute_limited =
        std::clamp(desired[index], -maximum[index], maximum[index]);
    const double maximum_step = maximum_rate[index] * safe_period;
    bounded[index] =
        previous[index] + std::clamp(absolute_limited - previous[index],
                                     -maximum_step, maximum_step);
  }
  return bounded;
}

inline double limit_reference(const double current, const double requested,
                              const double maximum_speed,
                              const double period_seconds) {
  const double maximum_step =
      std::max(0.0, maximum_speed) * std::max(0.0, period_seconds);
  return current + std::clamp(requested - current, -maximum_step, maximum_step);
}
} // namespace joint_impedance_controller
