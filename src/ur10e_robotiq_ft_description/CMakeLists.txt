cmake_minimum_required(VERSION 3.22)
project(ur10e_robotiq_ft_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
