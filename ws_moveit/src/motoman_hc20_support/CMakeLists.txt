cmake_minimum_required(VERSION 3.12)

project(motoman_hc20_support)

find_package(ament_cmake REQUIRED)
ament_package()

install(DIRECTORY meshes urdf launch config
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/joint_state_relay.py
  DESTINATION lib/${PROJECT_NAME})
