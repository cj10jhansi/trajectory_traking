#include "trajectory_tracking/trajectory_visualizer.hpp"

TrajectoryVisualizer::TrajectoryVisualizer(rclcpp::Node::SharedPtr node) {
    path_pub_ = node->create_publisher<nav_msgs::msg::Path>("smoothed_path", 10);
    traj_pub_ = node->create_publisher<nav_msgs::msg::Path>("trajectory_path", 10);
}

void TrajectoryVisualizer::publishPath(const std::vector<std::pair<double, double>>& path, const std::string& frame_id) {
    nav_msgs::msg::Path msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp = rclcpp::Clock().now();

    for (const auto& [x, y] : path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg.header;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;  // no rotation
        msg.poses.push_back(pose);
    }

    path_pub_->publish(msg);
}

void TrajectoryVisualizer::publishTrajectory(const std::vector<TrajectoryPoint>& traj, const std::string& frame_id) {
    nav_msgs::msg::Path msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp = rclcpp::Clock().now();

    for (const auto& pt : traj) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg.header;
        pose.pose.position.x = pt.x;
        pose.pose.position.y = pt.y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;
        msg.poses.push_back(pose);
    }

    traj_pub_->publish(msg);
}
