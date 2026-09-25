cmake_minimum_required(VERSION 3.22)
project(cartesian_impedance_controller)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(controller_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)

add_library(${PROJECT_NAME} SHARED src/cartesian_impedance_controller.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
ament_target_dependencies(${PROJECT_NAME}
  controller_interface geometry_msgs kdl_parser orocos_kdl pluginlib rclcpp realtime_tools)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
pluginlib_export_plugin_description_file(controller_interface controller_plugins.xml)

add_executable(move_base_xyz src/move_base_xyz.cpp)
ament_target_dependencies(move_base_xyz geometry_msgs rclcpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  ament_add_gtest(test_wrench_transform test/test_wrench_transform.cpp)
  target_link_libraries(test_wrench_transform Eigen3::Eigen)
  target_include_directories(test_wrench_transform PRIVATE include)

  ament_add_gtest(test_impedance_law test/test_impedance_law.cpp)
  target_link_libraries(test_impedance_law Eigen3::Eigen)
  target_include_directories(test_impedance_law PRIVATE include)

  ament_add_gtest(test_reference_limiter test/test_reference_limiter.cpp)
  target_link_libraries(test_reference_limiter Eigen3::Eigen)
  target_include_directories(test_reference_limiter PRIVATE include)

  ament_add_gtest(test_cartesian_pose test/test_cartesian_pose.cpp)
  target_link_libraries(test_cartesian_pose Eigen3::Eigen)
  target_include_directories(test_cartesian_pose PRIVATE include)

  ament_add_gtest(test_wrench_safety test/test_wrench_safety.cpp)
  target_include_directories(test_wrench_safety PRIVATE include)

  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)
  ament_add_gmock(test_load_controller test/test_load_controller.cpp)
  target_link_libraries(test_load_controller ${PROJECT_NAME})
  ament_target_dependencies(test_load_controller controller_manager ros2_control_test_assets)
  set_tests_properties(test_load_controller PROPERTIES
    ENVIRONMENT "AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}:$ENV{AMENT_PREFIX_PATH}")
endif()

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
install(TARGETS move_base_xyz DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES controller_plugins.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
ament_package()
