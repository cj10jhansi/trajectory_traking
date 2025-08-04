#ifndef PURE_PURSUIT_CONTROLLER_HPP
#define PURE_PURSUIT_CONTROLLER_HPP

#include "trajectory_tracking/pose2d.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <vector>
#include <mutex>
#include <thread>
#include <memory>
using trajectory_tracking::Pose2D;

class PurePursuitController : public rclcpp::Node {
public:
  static std::shared_ptr<PurePursuitController> getInstance();

private:
  PurePursuitController();

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void waypointsCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void controlLoop();
  Pose2D getLookaheadPoint(const Pose2D &current_pose);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr waypoints_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  Pose2D current_pose_;
  std::vector<Pose2D> waypoints_;

  double lookahead_distance_;
  double wheel_base_;
  double linear_velocity_;
  bool has_pose_ = false;

  std::mutex mutex_;
  std::thread control_thread_;
};

#endif // PURE_PURSUIT_CONTROLLER_HPP
