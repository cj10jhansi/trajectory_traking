#include "trajectory_tracking/path_planner_node.hpp"
#include "trajectory_tracking/pose2d.hpp"

#include <cmath>

std::shared_ptr<PathPlannerNode> PathPlannerNode::getInstance() {
  static std::shared_ptr<PathPlannerNode> instance(new PathPlannerNode());
  return instance;
}

PathPlannerNode::PathPlannerNode() : Node("path_planner_node") {
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&PathPlannerNode::goalCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&PathPlannerNode::odomCallback, this, std::placeholders::_1));

  waypoints_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/waypoints", 10);
}

void PathPlannerNode::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (goal_queue_.size() >= 3) goal_queue_.erase(goal_queue_.begin());
  goal_queue_.emplace_back(Pose2D{msg->pose.position.x, msg->pose.position.y, 0.0});
}

void PathPlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  current_pose_.x = msg->pose.pose.position.x;
  current_pose_.y = msg->pose.pose.position.y;
  has_pose_ = true;
}

std::vector<Pose2D> PathPlannerNode::smoothTrajectory(const Pose2D &start, const std::vector<Pose2D> &goals) {
  std::vector<Pose2D> smoothed;
  Pose2D prev = start;

  for (const auto &goal : goals) {
    double dx = goal.x - prev.x;
    double dy = goal.y - prev.y;
    double dist = std::hypot(dx, dy);
    int steps = static_cast<int>(dist / 0.5);

    for (int i = 1; i <= steps; ++i) {
      double ratio = static_cast<double>(i) / steps;
      Pose2D interp;
      interp.x = prev.x + ratio * dx;
      interp.y = prev.y + ratio * dy;
      interp.yaw = 0.0;
      smoothed.push_back(interp);
    }
    prev = goal;
  }

  return smoothed;
}

void PathPlannerNode::run() {
  rclcpp::Rate rate(2);
  while (rclcpp::ok()) {
    std::vector<Pose2D> goals;
    Pose2D pose;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!has_pose_ || goal_queue_.size() < 3) {
        rate.sleep();
        continue;
      }
      pose = current_pose_;
      goals = goal_queue_;
    }

    std::vector<Pose2D> path = smoothTrajectory(pose, goals);

    geometry_msgs::msg::PoseArray msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";
    for (const auto &pt : path) {
      geometry_msgs::msg::Pose pose_msg;
      pose_msg.position.x = pt.x;
      pose_msg.position.y = pt.y;
      pose_msg.orientation.w = 1.0;
      msg.poses.push_back(pose_msg);
    }

    waypoints_pub_->publish(msg);
    rate.sleep();
  }
}
