cmake_minimum_required(VERSION 3.8)
project(mission_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mission_sequencer REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(mission_planner_node
  src/mission_planner.cpp
  src/mission_planner_config.cpp
)

ament_target_dependencies(mission_planner_node
  rclcpp
  mavros_msgs
  geometry_msgs
  std_msgs
  mission_sequencer
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(mission_planner_node Eigen3::Eigen)


install(TARGETS mission_planner_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include/
)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
