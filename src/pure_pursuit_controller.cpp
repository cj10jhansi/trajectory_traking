#include "trajectory_tracking/pure_pursuit_controller.hpp"
#include "trajectory_tracking/pose2d.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <thread>

std::shared_ptr<PurePursuitController> PurePursuitController::getInstance() {
  static std::shared_ptr<PurePursuitController> instance(new PurePursuitController());
  return instance;
}

PurePursuitController::PurePursuitController()
    : Node("pure_pursuit_controller_node"),
      lookahead_distance_(0.4),   // tune as needed
      wheel_base_(0.2),           // TurtleBot3 wheelbase (approx)
      linear_velocity_(0.2)       // constant speed
{
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&PurePursuitController::odomCallback, this, std::placeholders::_1));

  waypoints_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/waypoints_ppc", 10,
      std::bind(&PurePursuitController::waypointsCallback, this, std::placeholders::_1));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  control_thread_ = std::thread(&PurePursuitController::controlLoop, this);
}

void PurePursuitController::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  current_pose_.x = msg->pose.pose.position.x;
  current_pose_.y = msg->pose.pose.position.y;

  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  current_pose_.yaw = tf2::getYaw(q);
  has_pose_ = true;
}

void PurePursuitController::waypointsCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  waypoints_.clear();
  for (const auto &pose : msg->poses) {
    double yaw = tf2::getYaw(pose.orientation);
    waypoints_.emplace_back(Pose2D{pose.position.x, pose.position.y, yaw});
  }
}

Pose2D PurePursuitController::getLookaheadPoint(const Pose2D &current_pose) {
  for (const auto &pt : waypoints_) {
    double dx = pt.x - current_pose.x;
    double dy = pt.y - current_pose.y;
    if (std::hypot(dx, dy) >= lookahead_distance_) {
      return pt;
    }
  }
  return waypoints_.back();  // final point fallback
}

void PurePursuitController::controlLoop() {
  rclcpp::Rate rate(10);
  while (rclcpp::ok()) {
    Pose2D pose;
    std::vector<Pose2D> wps;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!has_pose_ || waypoints_.empty()) {
        rate.sleep();
        continue;
      }
      pose = current_pose_;
      wps = waypoints_;
    }

    Pose2D lookahead = getLookaheadPoint(pose);
    double dx = lookahead.x - pose.x;
    double dy = lookahead.y - pose.y;
    double Ld = std::hypot(dx, dy);

    if (Ld < 0.05) {
      RCLCPP_INFO(this->get_logger(), "Goal reached.");
      geometry_msgs::msg::Twist stop_msg;
      stop_msg.linear.x = 0.0;
      stop_msg.angular.z = 0.0;
      cmd_vel_pub_->publish(stop_msg);
      continue;
    }

    double angle_to_goal = std::atan2(dy, dx);
    double alpha = angle_to_goal - pose.yaw;

    double angular_velocity = (2.0 * linear_velocity_ * std::sin(alpha)) / Ld;

    geometry_msgs::msg::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = linear_velocity_;
    cmd_vel_msg.angular.z = angular_velocity;
    cmd_vel_pub_->publish(cmd_vel_msg);

    rate.sleep();
  }
}
