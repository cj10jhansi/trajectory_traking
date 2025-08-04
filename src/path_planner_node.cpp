#include "trajectory_tracking/path_planner_node.hpp"
#include <cmath>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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

  waypoints_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/waypoints_ppc", 10);
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
}

void PathPlannerNode::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  goal_pose_ = Pose2D{msg->pose.position.x, msg->pose.position.y, 0.0};
  has_goal_ = true;
  RCLCPP_INFO(this->get_logger(), "Received goal: x=%.2f, y=%.2f", goal_pose_.x, goal_pose_.y);
}

void PathPlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  current_pose_.x = msg->pose.pose.position.x;
  current_pose_.y = msg->pose.pose.position.y;
  has_pose_ = true;
}


std::vector<Pose2D> PathPlannerNode::smoothTrajectory(const Pose2D &start, const Pose2D &goal, double resolution) {
  std::vector<Pose2D> trajectory;

  // Generate control points
  int num_points = 5;
  std::vector<double> x_points, y_points, t_points;

  for (int i = 0; i < num_points; ++i) {
    double t = static_cast<double>(i) / (num_points - 1);
    t_points.push_back(t);
    x_points.push_back(start.x + t * (goal.x - start.x));
    y_points.push_back(start.y + t * (goal.y - start.y));
  }


  auto interpolate = [](const std::vector<double> &t, const std::vector<double> &values, double dt) {
    int n = t.size();
    Eigen::VectorXd T(n), Y(n);
    for (int i = 0; i < n; ++i) {
      T(i) = t[i];
      Y(i) = values[i];
    }

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
    Eigen::VectorXd B = Eigen::VectorXd::Zero(n);

    A(0, 0) = 1.0;
    A(n - 1, n - 1) = 1.0;

    for (int i = 1; i < n - 1; ++i) {
      double dt1 = T(i) - T(i - 1);
      double dt2 = T(i + 1) - T(i);
      A(i, i - 1) = dt1 / 6.0;
      A(i, i) = (dt1 + dt2) / 3.0;
      A(i, i + 1) = dt2 / 6.0;
      B(i) = (Y(i + 1) - Y(i)) / dt2 - (Y(i) - Y(i - 1)) / dt1;
    }

    Eigen::VectorXd M = A.colPivHouseholderQr().solve(B);

    std::vector<double> result;
    for (double tval = 0.0; tval <= 1.0; tval += dt) {
      int i = 1;
      while (i < n - 1 && T(i + 1) < tval) i++;
      double h = T(i + 1) - T(i);
      double a = (T(i + 1) - tval) / h;
      double b = (tval - T(i)) / h;

      double val = a * values[i] + b * values[i + 1]
        + ((a * a * a - a) * M(i) + (b * b * b - b) * M(i + 1)) * (h * h) / 6.0;
      result.push_back(val);
    }

    return result;
  };

  double path_length = std::hypot(goal.x - start.x, goal.y - start.y);
  int interp_points = std::max(2, static_cast<int>(path_length / resolution));
  double dt = 1.0 / interp_points;

  std::vector<double> x_spline = interpolate(t_points, x_points, dt);
  std::vector<double> y_spline = interpolate(t_points, y_points, dt);

  for (size_t i = 0; i < x_spline.size(); ++i) {
    Pose2D pt;
    pt.x = x_spline[i];
    pt.y = y_spline[i];

    if (i < x_spline.size() - 1) {
      double dx = x_spline[i + 1] - x_spline[i];
      double dy = y_spline[i + 1] - y_spline[i];
      pt.yaw = std::atan2(dy, dx);
    } else if (i > 0) {
      double dx = x_spline[i] - x_spline[i - 1];
      double dy = y_spline[i] - y_spline[i - 1];
      pt.yaw = std::atan2(dy, dx);
    } else {
      pt.yaw = 0.0;
    }

    trajectory.push_back(pt);
  }

  return trajectory;
}


void PathPlannerNode::run() {
  rclcpp::Rate rate(10);
  while (rclcpp::ok()) {
    Pose2D pose, goal;
    bool ready = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (has_pose_ && has_goal_) {
        pose = current_pose_;
        goal = goal_pose_;
        ready = true;
      }
    }

    if (!ready) {
      rate.sleep();
      continue;
    }

    double dx = goal.x - pose.x;
    double dy = goal.y - pose.y;
    double dist = std::hypot(dx, dy);

    if (dist < 0.05) {
      RCLCPP_INFO(this->get_logger(), "Reached goal. Waiting for new goal.");
      std::lock_guard<std::mutex> lock(mutex_);
      has_goal_ = false;
      rate.sleep();
      continue;
    }

    std::vector<Pose2D> path = smoothTrajectory(pose, goal, 0.4);

    // Publish PoseArray waypoints
    geometry_msgs::msg::PoseArray pose_array_msg;
    pose_array_msg.header.stamp = this->now();
    pose_array_msg.header.frame_id = "map";

    // Publish Path
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = "map";

    for (const auto &pt : path) {
      geometry_msgs::msg::Pose pose_msg;
      pose_msg.position.x = pt.x;
      pose_msg.position.y = pt.y;
      pose_msg.orientation.w = pt.yaw;
      pose_array_msg.poses.push_back(pose_msg);

      geometry_msgs::msg::PoseStamped ps;
      ps.header.stamp = this->now();
      ps.header.frame_id = "map";
      ps.pose = pose_msg;
      path_msg.poses.push_back(ps);
    }

    waypoints_pub_->publish(pose_array_msg);
    path_pub_->publish(path_msg);
    rate.sleep();
  }
}
