#include <gtest/gtest.h>

#include "joint_impedance_controller/impedance_law.hpp"

using joint_impedance_controller::JointVector;

TEST(JointImpedanceLaw, ComputesSpringDamperTorque) {
  const JointVector position_error{0.1, -0.2, 0.0, 0.0, 0.0, 0.0};
  const JointVector velocity_error{0.5, 0.25, 0.0, 0.0, 0.0, 0.0};
  const JointVector stiffness{10.0, 20.0, 1.0, 1.0, 1.0, 1.0};
  const JointVector damping{2.0, 4.0, 1.0, 1.0, 1.0, 1.0};

  const auto torque = joint_impedance_controller::impedance_torque(
      position_error, velocity_error, stiffness, damping);
  EXPECT_DOUBLE_EQ(torque[0], 2.0);
  EXPECT_DOUBLE_EQ(torque[1], -3.0);
}

TEST(JointImpedanceLaw, LimitsMagnitudeAndRate) {
  const JointVector desired{100.0, -100.0, 5.0, 5.0, 5.0, 5.0};
  const JointVector previous{};
  const JointVector maximum{10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
  const JointVector rate{20.0, 20.0, 20.0, 20.0, 20.0, 20.0};

  const auto torque = joint_impedance_controller::limit_torque(
      desired, previous, maximum, rate, 0.01);
  EXPECT_DOUBLE_EQ(torque[0], 0.2);
  EXPECT_DOUBLE_EQ(torque[1], -0.2);
  EXPECT_DOUBLE_EQ(torque[2], 0.2);
}

TEST(JointImpedanceLaw, LimitsReferenceSpeed) {
  EXPECT_DOUBLE_EQ(
      joint_impedance_controller::limit_reference(0.0, 1.0, 0.2, 0.01), 0.002);
  EXPECT_DOUBLE_EQ(
      joint_impedance_controller::limit_reference(0.0, -1.0, 0.2, 0.01),
      -0.002);
}
