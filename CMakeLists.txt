cmake_minimum_required(VERSION 3.8)
project(trajectory_tracking)

# Enable C++14 or later
set(CMAKE_CXX_STANDARD 14)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include headers
include_directories(
  include
  /usr/include/eigen3
)

# Add executable
add_executable(trajectory_tracking_node
  src/main.cpp
  src/path_planner_node.cpp
  src/pure_pursuit_controller.cpp
)

# Link dependencies
ament_target_dependencies(trajectory_tracking_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

# Link Eigen explicitly if needed
target_link_libraries(trajectory_tracking_node
  Eigen3::Eigen
)

# Install binary
install(TARGETS trajectory_tracking_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files if available
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
