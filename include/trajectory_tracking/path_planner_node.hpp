#ifndef PATH_PLANNER_NODE_HPP
#define PATH_PLANNER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include "trajectory_tracking/pose2d.hpp"

#include <vector>
#include <mutex>
#include <memory>

using trajectory_tracking::Pose2D;

class PathPlannerNode : public rclcpp::Node {
public:
  static std::shared_ptr<PathPlannerNode> getInstance();

  void run();

private:
  PathPlannerNode();

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  std::vector<Pose2D> smoothTrajectory(const Pose2D &start, const Pose2D &goal, double resolution);

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr waypoints_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  Pose2D current_pose_;
  Pose2D goal_pose_;
  bool has_pose_ = false;
  bool has_goal_ = false;

  std::mutex mutex_;
};

#endif // PATH_PLANNER_NODE_HPP
