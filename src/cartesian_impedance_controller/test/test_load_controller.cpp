#include <gmock/gmock.h>

#include <controller_manager/controller_manager.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros2_control_test_assets/descriptions.hpp>

TEST(CartesianImpedanceController, LoadsFromPluginRegistry)
{
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  controller_manager::ControllerManager manager(
      executor, ros2_control_test_assets::minimal_robot_urdf, true, "test_controller_manager");
  manager.set_parameter({ "cartesian_impedance.type",
                          "cartesian_impedance_controller/CartesianImpedanceController" });

  EXPECT_NE(manager.load_controller("cartesian_impedance"), nullptr);
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
