cmake_minimum_required(VERSION 3.22)
project(cartesian_impedance_controller)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_wrench_transform test/test_wrench_transform.cpp)
  target_link_libraries(test_wrench_transform Eigen3::Eigen)
  target_include_directories(test_wrench_transform PRIVATE include)
endif()

ament_package()
