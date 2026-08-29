#include <gmock/gmock.h>

#include <array>

#include <controller_manager/controller_manager.hpp>
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
  <link name="gripper_tcp"/>
  <joint name="shoulder_pan_joint" type="revolute"><parent link="base_link"/><child link="link_1"/><axis xyz="0 0 1"/><limit lower="-6.2" upper="6.2" effort="100" velocity="1"/></joint>
  <joint name="shoulder_lift_joint" type="revolute"><parent link="link_1"/><child link="link_2"/><origin xyz="0 0 0.1"/><axis xyz="0 1 0"/><limit lower="-6.2" upper="6.2" effort="100" velocity="1"/></joint>
  <joint name="elbow_joint" type="revolute"><parent link="link_2"/><child link="link_3"/><origin xyz="0 0 0.1"/><axis xyz="0 1 0"/><limit lower="-6.2" upper="6.2" effort="100" velocity="1"/></joint>
  <joint name="wrist_1_joint" type="revolute"><parent link="link_3"/><child link="link_4"/><origin xyz="0 0 0.1"/><axis xyz="1 0 0"/><limit lower="-6.2" upper="6.2" effort="100" velocity="1"/></joint>
  <joint name="wrist_2_joint" type="revolute"><parent link="link_4"/><child link="link_5"/><origin xyz="0 0 0.1"/><axis xyz="0 1 0"/><limit lower="-6.2" upper="6.2" effort="100" velocity="1"/></joint>
  <joint name="wrist_3_joint" type="revolute"><parent link="link_5"/><child link="gripper_tcp"/><origin xyz="0 0 0.1"/><axis xyz="1 0 0"/><limit lower="-6.2" upper="6.2" effort="100" velocity="1"/></joint>
</robot>)";
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

int main(int argc, char* argv[])
{
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
