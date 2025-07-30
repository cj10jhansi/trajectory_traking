#ifndef PATH_PLANNER_NODE_HPP_
#define PATH_PLANNER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mutex>
#include <thread>
#include "trajectory_tracking/pose2d.hpp"
using trajectory_tracking::Pose2D;

// struct Pose2D {
//   double x;
//   double y;
//   double yaw;
// };

class PathPlannerNode : public rclcpp::Node {
public:
  static std::shared_ptr<PathPlannerNode> getInstance();
  void run();

private:
  PathPlannerNode();
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  std::vector<Pose2D> smoothTrajectory(const Pose2D &start, const std::vector<Pose2D> &goals);

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr waypoints_pub_;

  std::vector<Pose2D> goal_queue_;
  Pose2D current_pose_;
  bool has_pose_ = false;
  std::mutex mutex_;
};

#endif  // PATH_PLANNER_NODE_HPP_
