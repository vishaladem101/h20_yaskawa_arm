cmake_minimum_required(VERSION 3.8)
project(hc30pl_pick_place)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(pick_place_coordinator src/pick_place_coordinator.cpp)
target_include_directories(pick_place_coordinator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  pick_place_coordinator
  "rclcpp"
  "moveit_ros_planning_interface"
  "geometry_msgs"
  "tf2_geometry_msgs"
)

install(TARGETS pick_place_coordinator
  DESTINATION lib/${PROJECT_NAME})

ament_package()
