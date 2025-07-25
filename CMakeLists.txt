cmake_minimum_required(VERSION 3.8)
project(trajectory_tracking)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Add executable
add_executable(trajectory_tracking_node
  src/main.cpp
  src/path_smoother.cpp
  src/trajectory_generator.cpp
  src/trajectory_visualizer.cpp
)

# Link libraries to the executable
target_link_libraries(trajectory_tracking_node
  Eigen3::Eigen
)

# Link dependencies
ament_target_dependencies(trajectory_tracking_node
  rclcpp
  nav_msgs
  geometry_msgs
)

# Install
install(TARGETS trajectory_tracking_node
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
