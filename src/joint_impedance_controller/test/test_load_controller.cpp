#include <gmock/gmock.h>

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <controller_manager/controller_manager.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros2_control_test_assets/descriptions.hpp>

#include "joint_impedance_controller/joint_impedance_controller.hpp"

namespace {
const std::array<std::string, 6> kJointNames{
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint"};
} // namespace

TEST(JointImpedanceController, LoadsFromPluginRegistry) {
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  controller_manager::ControllerManager manager(
      executor, ros2_control_test_assets::minimal_robot_urdf, true,
      "test_controller_manager");
  manager.set_parameter(
      {"joint_impedance.type",
       "joint_impedance_controller/JointImpedanceController"});
  EXPECT_NE(manager.load_controller("joint_impedance"), nullptr);
}

TEST(JointImpedanceController, DeclaresUrEffortContract) {
  joint_impedance_controller::JointImpedanceController controller;
  ASSERT_EQ(
      controller.init("joint_impedance", "", 500, "", rclcpp::NodeOptions()),
      controller_interface::return_type::OK);
  ASSERT_EQ(controller.configure().id(),
            lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  const auto command = controller.command_interface_configuration();
  EXPECT_THAT(command.names,
              testing::ElementsAre(
                  "shoulder_pan_joint/effort", "shoulder_lift_joint/effort",
                  "elbow_joint/effort", "wrist_1_joint/effort",
                  "wrist_2_joint/effort", "wrist_3_joint/effort"));
  const auto state = controller.state_interface_configuration();
  EXPECT_EQ(state.names.size(), 12U);
  EXPECT_EQ(state.names.front(), "shoulder_pan_joint/position");
  EXPECT_EQ(state.names.back(), "wrist_3_joint/velocity");
}

TEST(JointImpedanceController, ActivationHoldsCurrentPositionWithZeroTorque) {
  joint_impedance_controller::JointImpedanceController controller;
  ASSERT_EQ(
      controller.init("joint_impedance", "", 500, "", rclcpp::NodeOptions()),
      controller_interface::return_type::OK);
  ASSERT_EQ(controller.configure().id(),
            lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  std::vector<hardware_interface::CommandInterface::SharedPtr> commands;
  std::vector<hardware_interface::LoanedCommandInterface> loaned_commands;
  for (const auto &joint : kJointNames) {
    auto interface = std::make_shared<hardware_interface::CommandInterface>(
        joint, "effort", "double", "0.0");
    loaned_commands.emplace_back(interface, []() {});
    commands.push_back(std::move(interface));
  }
  std::vector<hardware_interface::LoanedStateInterface> loaned_states;
  for (const auto *interface_name : {"position", "velocity"}) {
    for (const auto &joint : kJointNames) {
      auto interface = std::make_shared<hardware_interface::StateInterface>(
          joint, interface_name, "double", "0.0");
      loaned_states.emplace_back(interface);
    }
  }
  controller.assign_interfaces(std::move(loaned_commands),
                               std::move(loaned_states));
  ASSERT_EQ(controller.get_node()->activate().id(),
            lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_EQ(controller.update(rclcpp::Time(1, 0, RCL_ROS_TIME),
                              rclcpp::Duration::from_seconds(0.002)),
            controller_interface::return_type::OK);
  for (const auto &command : commands) {
    ASSERT_TRUE(command->get_optional<double>().has_value());
    EXPECT_DOUBLE_EQ(*command->get_optional<double>(), 0.0);
  }
}

int main(int argc, char *argv[]) {
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
