cmake_minimum_required(VERSION 3.22)
project(joint_impedance_controller)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/joint_impedance_controller.cpp
  src/joint_impedance_mock_system.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
ament_target_dependencies(${PROJECT_NAME}
  controller_interface hardware_interface pluginlib rclcpp realtime_tools sensor_msgs)
pluginlib_export_plugin_description_file(controller_interface controller_plugins.xml)
pluginlib_export_plugin_description_file(hardware_interface hardware_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(lifecycle_msgs REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gtest(test_impedance_law test/test_impedance_law.cpp)
  target_include_directories(test_impedance_law PRIVATE include)

  ament_add_gmock(test_load_controller test/test_load_controller.cpp)
  target_link_libraries(test_load_controller ${PROJECT_NAME})
  target_include_directories(test_load_controller PRIVATE include)
  ament_target_dependencies(test_load_controller
    controller_manager lifecycle_msgs ros2_control_test_assets)
  set_tests_properties(test_load_controller PROPERTIES
    ENVIRONMENT "AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}:$ENV{AMENT_PREFIX_PATH}")
endif()

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(FILES controller_plugins.xml hardware_plugins.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config launch docs urdf DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/run_rviz_test.sh DESTINATION lib/${PROJECT_NAME})

ament_package()
