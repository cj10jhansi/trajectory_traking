#include "trajectory_tracking/path_planner_node.hpp"
#include "trajectory_tracking/pure_pursuit_controller.hpp"
#include "trajectory_tracking/pose2d.hpp"
#include <rclcpp/rclcpp.hpp>
#include <thread>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto planner = PathPlannerNode::getInstance();
  auto controller = PurePursuitController::getInstance();

  std::thread planner_thread([&]() { planner->run(); });

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(planner);
  executor.add_node(controller);
  executor.spin();

  planner_thread.join();

  rclcpp::shutdown();
  return 0;
}
