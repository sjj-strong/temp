#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace cartesian_impedance_controller
{
using Vector6 = std::array<double, 6>;

inline Vector6 cartesian_impedance_wrench_unbounded(const Vector6& pose_error, const Vector6& measured_twist,
                                                    const Vector6& desired_twist, const Vector6& integral_error,
                                                    const Vector6& stiffness, const Vector6& damping,
                                                    const Vector6& integral_gain)
{
  Vector6 command{};
  for (std::size_t index = 0; index < command.size(); ++index) {
    command[index] = stiffness[index] * pose_error[index] + damping[index] * (desired_twist[index] - measured_twist[index]) +
                     integral_gain[index] * integral_error[index];
  }
  return command;
}

inline Vector6 cartesian_impedance_wrench(const Vector6& pose_error, const Vector6& measured_twist,
                                          const Vector6& desired_twist, const Vector6& integral_error,
                                          const Vector6& stiffness, const Vector6& damping,
                                          const Vector6& integral_gain, const Vector6& wrench_limit)
{
  Vector6 command = cartesian_impedance_wrench_unbounded(pose_error, measured_twist, desired_twist, integral_error,
                                                          stiffness, damping, integral_gain);
  for (std::size_t index = 0; index < command.size(); ++index) {
    const double limit = std::abs(wrench_limit[index]);
    command[index] = std::clamp(command[index], -limit, limit);
  }
  return command;
}

inline Vector6 integrate_error(const Vector6& previous, const Vector6& pose_error, const double period_seconds,
                               const Vector6& integral_limit)
{
  Vector6 result = previous;
  if (!std::isfinite(period_seconds) || period_seconds <= 0.0) {
    return result;
  }
  for (std::size_t index = 0; index < result.size(); ++index) {
    const double limit = std::abs(integral_limit[index]);
    result[index] = std::clamp(previous[index] + pose_error[index] * period_seconds, -limit, limit);
  }
  return result;
}

inline Vector6 integrate_error_with_antiwindup(const Vector6& previous, const Vector6& pose_error,
                                               const double period_seconds, const Vector6& integral_limit,
                                               const Vector6& wrench_without_integral, const Vector6& integral_gain,
                                               const Vector6& wrench_limit)
{
  Vector6 candidate = integrate_error(previous, pose_error, period_seconds, integral_limit);
  for (std::size_t index = 0; index < candidate.size(); ++index) {
    const double candidate_wrench = wrench_without_integral[index] + integral_gain[index] * candidate[index];
    const double limit = std::abs(wrench_limit[index]);
    const bool deepens_positive_saturation = candidate_wrench > limit && pose_error[index] > 0.0;
    const bool deepens_negative_saturation = candidate_wrench < -limit && pose_error[index] < 0.0;
    if (deepens_positive_saturation || deepens_negative_saturation) {
      candidate[index] = previous[index];
    }
  }
  return candidate;
}

inline Vector6 limit_joint_torque(const Vector6& desired, const Vector6& previous, const Vector6& absolute_limit,
                                  const Vector6& rate_limit, const double period_seconds)
{
  Vector6 result{};
  const double bounded_period = std::max(0.0, std::isfinite(period_seconds) ? period_seconds : 0.0);
  for (std::size_t index = 0; index < result.size(); ++index) {
    const double absolute = std::abs(absolute_limit[index]);
    const double rate_step = std::abs(rate_limit[index]) * bounded_period;
    const double bounded_desired = std::clamp(desired[index], -absolute, absolute);
    result[index] = std::clamp(bounded_desired, previous[index] - rate_step, previous[index] + rate_step);
    result[index] = std::clamp(result[index], -absolute, absolute);
  }
  return result;
}

inline Vector6 spring_wrench(const Vector6& error, const Vector6& stiffness, const Vector6& max_wrench)
{
  const Vector6 zero{};
  return cartesian_impedance_wrench(error, zero, zero, zero, stiffness, zero, zero, max_wrench);
}
}  // namespace cartesian_impedance_controller
