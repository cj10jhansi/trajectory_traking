#ifndef PURE_PURSUIT_CONTROLLER_HPP_
#define PURE_PURSUIT_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <thread>
#include <mutex>
#include "trajectory_tracking/pose2d.hpp"
using trajectory_tracking::Pose2D;

// struct Pose2D {
//   double x;
//   double y;
//   double yaw;
// };

class PurePursuitController : public rclcpp::Node {
public:
  static std::shared_ptr<PurePursuitController> getInstance();
  void run() {}

private:
  PurePursuitController();
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void waypointsCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void controlLoop();
  Pose2D getLookaheadPoint(const Pose2D &current_pose);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr waypoints_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steer_pub_;

  std::mutex mutex_;
  Pose2D current_pose_;
  std::vector<Pose2D> waypoints_;
  bool has_pose_ = false;

  double lookahead_distance_ = 0.5;
  std::thread control_thread_;
};

#endif  // PURE_PURSUIT_CONTROLLER_HPP_
