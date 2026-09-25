#include <gmock/gmock.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <controller_manager/controller_manager.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros2_control_test_assets/descriptions.hpp>

#include "cartesian_impedance_controller/cartesian_impedance_controller.hpp"

namespace
{
const char* const kSixAxisUrdf = R"(
<robot name="six_axis_test">
  <link name="base_link"/>
  <link name="link_1"/>
  <link name="link_2"/>
  <link name="link_3"/>
  <link name="link_4"/>
  <link name="link_5"/>
  <link name="tool0"/>
  <joint name="shoulder_pan_joint" type="revolute"><parent link="base_link"/><child link="link_1"/><axis xyz="0 0 1"/><limit lower="-6.2" upper="6.2" effort="100" velocity="1"/></joint>
  <joint name="shoulder_lift_joint" type="revolute"><parent link="link_1"/><child link="link_2"/><origin xyz="0 0 0.1"/><axis xyz="0 1 0"/><limit lower="-6.2" upper="6.2" effort="100" velocity="1"/></joint>
  <joint name="elbow_joint" type="revolute"><parent link="link_2"/><child link="link_3"/><origin xyz="0 0 0.1"/><axis xyz="0 1 0"/><limit lower="-6.2" upper="6.2" effort="100" velocity="1"/></joint>
  <joint name="wrist_1_joint" type="revolute"><parent link="link_3"/><child link="link_4"/><origin xyz="0 0 0.1"/><axis xyz="1 0 0"/><limit lower="-6.2" upper="6.2" effort="100" velocity="1"/></joint>
  <joint name="wrist_2_joint" type="revolute"><parent link="link_4"/><child link="link_5"/><origin xyz="0 0 0.1"/><axis xyz="0 1 0"/><limit lower="-6.2" upper="6.2" effort="100" velocity="1"/></joint>
  <joint name="wrist_3_joint" type="revolute"><parent link="link_5"/><child link="tool0"/><origin xyz="0 0 0.1"/><axis xyz="1 0 0"/><limit lower="-6.2" upper="6.2" effort="100" velocity="1"/></joint>
  <link name="ft_frame"/>
  <joint name="ft_fixed_joint" type="fixed"><parent link="tool0"/><child link="ft_frame"/></joint>
</robot>)";

const std::array<std::string, 6> kJointNames{ "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                              "wrist_1_joint", "wrist_2_joint", "wrist_3_joint" };

class RuntimeController
{
public:
  explicit RuntimeController(const bool use_external_ft = false, const bool restrict_workspace_x = false)
  {
    controller = std::make_unique<cartesian_impedance_controller::CartesianImpedanceController>();
    EXPECT_EQ(controller->init("cartesian_impedance", kSixAxisUrdf, 500, "", rclcpp::NodeOptions()),
              controller_interface::return_type::OK);
    EXPECT_TRUE(controller->get_node()->set_parameter(rclcpp::Parameter("use_coriolis", false)).successful);
    if (use_external_ft) {
      EXPECT_TRUE(controller->get_node()->set_parameter(rclcpp::Parameter("use_external_ft", true)).successful);
      EXPECT_TRUE(controller->get_node()->set_parameter(rclcpp::Parameter("ft_frame", "ft_frame")).successful);
    }
    if (restrict_workspace_x) {
      EXPECT_TRUE(controller->get_node()
                      ->set_parameter(rclcpp::Parameter("workspace_min", std::vector<double>{ -0.1, -1.2, -0.1, -3.2, -3.2, -3.2 }))
                      .successful);
      EXPECT_TRUE(controller->get_node()
                      ->set_parameter(rclcpp::Parameter("workspace_max", std::vector<double>{ 0.1, 1.2, 1.5, 3.2, 3.2, 3.2 }))
                      .successful);
    }
    EXPECT_EQ(controller->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    std::vector<hardware_interface::LoanedCommandInterface> loaned_commands;
    for (const auto& joint : kJointNames) {
      auto interface = std::make_shared<hardware_interface::CommandInterface>(joint, "effort", "double", "0.0");
      loaned_commands.emplace_back(interface, []() {});
      command_interfaces.push_back(std::move(interface));
    }

    std::vector<hardware_interface::LoanedStateInterface> loaned_states;
    for (const auto* interface_name : { "position", "velocity" }) {
      for (const auto& joint : kJointNames) {
        auto interface = std::make_shared<hardware_interface::StateInterface>(joint, interface_name, "double", "0.0");
        loaned_states.emplace_back(interface);
        state_interfaces.push_back(std::move(interface));
      }
    }
    if (use_external_ft) {
      for (const auto* interface_name : { "force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z" }) {
        auto interface =
            std::make_shared<hardware_interface::StateInterface>("robotiq_ft_sensor", interface_name, "double", "0.0");
        loaned_states.emplace_back(interface);
        state_interfaces.push_back(std::move(interface));
      }
    }
    controller->assign_interfaces(std::move(loaned_commands), std::move(loaned_states));
    EXPECT_EQ(controller->get_node()->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }

  void expect_zero_torque() const
  {
    for (const auto& interface : command_interfaces) {
      const auto value = interface->get_optional<double>();
      ASSERT_TRUE(value.has_value());
      EXPECT_DOUBLE_EQ(*value, 0.0);
    }
  }

  std::vector<hardware_interface::StateInterface::SharedPtr> state_interfaces;
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;
  std::unique_ptr<cartesian_impedance_controller::CartesianImpedanceController> controller;
};
}  // namespace

TEST(CartesianImpedanceController, LoadsFromPluginRegistry)
{
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  controller_manager::ControllerManager manager(
      executor, ros2_control_test_assets::minimal_robot_urdf, true, "test_controller_manager");
  manager.set_parameter({ "cartesian_impedance.type",
                          "cartesian_impedance_controller/CartesianImpedanceController" });

  EXPECT_NE(manager.load_controller("cartesian_impedance"), nullptr);
}

TEST(CartesianImpedanceController, ConfiguresSixJointEffortContract)
{
  cartesian_impedance_controller::CartesianImpedanceController controller;
  ASSERT_EQ(controller.init("cartesian_impedance", kSixAxisUrdf, 500, "", rclcpp::NodeOptions()),
            controller_interface::return_type::OK);
  ASSERT_EQ(controller.configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  const auto command = controller.command_interface_configuration();
  ASSERT_EQ(command.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  EXPECT_THAT(command.names,
              testing::ElementsAre("shoulder_pan_joint/effort", "shoulder_lift_joint/effort", "elbow_joint/effort",
                                   "wrist_1_joint/effort", "wrist_2_joint/effort", "wrist_3_joint/effort"));

  const auto state = controller.state_interface_configuration();
  ASSERT_EQ(state.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  EXPECT_THAT(state.names,
              testing::ElementsAre("shoulder_pan_joint/position", "shoulder_lift_joint/position", "elbow_joint/position",
                                   "wrist_1_joint/position", "wrist_2_joint/position", "wrist_3_joint/position",
                                   "shoulder_pan_joint/velocity", "shoulder_lift_joint/velocity", "elbow_joint/velocity",
                                   "wrist_1_joint/velocity", "wrist_2_joint/velocity", "wrist_3_joint/velocity"));
}

TEST(CartesianImpedanceController, RejectsJointOrderDifferentFromKdlChain)
{
  cartesian_impedance_controller::CartesianImpedanceController controller;
  ASSERT_EQ(controller.init("cartesian_impedance", kSixAxisUrdf, 500, "", rclcpp::NodeOptions()),
            controller_interface::return_type::OK);
  ASSERT_TRUE(controller.get_node()
                  ->set_parameter(rclcpp::Parameter("joints", std::vector<std::string>{
                                                               "shoulder_lift_joint", "shoulder_pan_joint", "elbow_joint",
                                                               "wrist_1_joint", "wrist_2_joint", "wrist_3_joint" }))
                  .successful);

  EXPECT_NE(controller.configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

TEST(CartesianImpedanceController, UpdateWritesEffortFromCartesianDamping)
{
  RuntimeController fixture;
  ASSERT_TRUE(fixture.state_interfaces[6]->set_value(0.2));

  EXPECT_EQ(fixture.controller->update(rclcpp::Time(1, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::OK);
  EXPECT_TRUE(std::any_of(fixture.command_interfaces.begin(), fixture.command_interfaces.end(), [](const auto& interface) {
    const auto value = interface->template get_optional<double>();
    return value.has_value() && std::abs(*value) > 1e-9;
  }));
}

TEST(CartesianImpedanceController, InvalidJointStateWritesZeroEffort)
{
  RuntimeController fixture;
  for (const auto& interface : fixture.command_interfaces) {
    ASSERT_TRUE(interface->set_value(5.0));
  }
  ASSERT_TRUE(fixture.state_interfaces[0]->set_value(std::numeric_limits<double>::quiet_NaN()));

  EXPECT_EQ(fixture.controller->update(rclcpp::Time(1, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::ERROR);
  fixture.expect_zero_torque();
}

TEST(CartesianImpedanceController, WorkspaceViolationWritesZeroEffort)
{
  RuntimeController fixture(false, true);
  for (const auto& interface : fixture.command_interfaces) {
    ASSERT_TRUE(interface->set_value(5.0));
  }
  ASSERT_TRUE(fixture.state_interfaces[1]->set_value(std::acos(-1.0) / 2.0));

  EXPECT_EQ(fixture.controller->update(rclcpp::Time(1, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::ERROR);
  fixture.expect_zero_torque();
}

TEST(CartesianImpedanceController, ExternalWrenchLimitWritesZeroEffort)
{
  RuntimeController fixture(true);
  for (const auto& interface : fixture.command_interfaces) {
    ASSERT_TRUE(interface->set_value(5.0));
  }
  ASSERT_TRUE(fixture.state_interfaces[12]->set_value(81.0));

  EXPECT_EQ(fixture.controller->update(rclcpp::Time(1, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::ERROR);
  fixture.expect_zero_torque();
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
