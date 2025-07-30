#include "trajectory_tracking/pure_pursuit_controller.hpp"
#include "trajectory_tracking/path_planner_node.hpp"
#include <cmath>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "trajectory_tracking/pose2d.hpp"

std::shared_ptr<PurePursuitController> PurePursuitController::getInstance() {
  static std::shared_ptr<PurePursuitController> instance(new PurePursuitController());
  return instance;
}

PurePursuitController::PurePursuitController()
    : Node("pure_pursuit_controller") {
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&PurePursuitController::odomCallback, this, std::placeholders::_1));

  waypoints_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/waypoints", 10,
      std::bind(&PurePursuitController::waypointsCallback, this, std::placeholders::_1));

  steer_pub_ = this->create_publisher<std_msgs::msg::Float64>("/steering_angle", 10);

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
    waypoints_.emplace_back(Pose2D{pose.position.x, pose.position.y, 0.0});
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
  return current_pose;
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
    double angle_to_goal = std::atan2(dy, dx);
    double alpha = angle_to_goal - pose.yaw;
    double steering_angle = std::atan2(2.0 * 0.2 * std::sin(alpha), Ld);  // wheelbase = 0.2

    std_msgs::msg::Float64 msg;
    msg.data = steering_angle;
    steer_pub_->publish(msg);
    rate.sleep();
  }
}
