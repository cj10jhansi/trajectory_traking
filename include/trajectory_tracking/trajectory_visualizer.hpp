#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "trajectory_tracking/trajectory_generator.hpp"

/**
 * @class TrajectoryVisualizer
 * @brief Publishes smoothed paths and trajectories as ROS 2 messages for RViz visualization.
 */
class TrajectoryVisualizer {
public:
    /**
     * @brief Constructor.
     * @param node The ROS 2 node.
     */
    TrajectoryVisualizer(rclcpp::Node::SharedPtr node);

    /**
     * @brief Publish a path to RViz.
     * @param path The vector of (x, y) points.
     * @param frame_id The coordinate frame to use (default: "map").
     */
    void publishPath(const std::vector<std::pair<double, double>>& path, const std::string& frame_id = "map");

    /**
     * @brief Publish a trajectory to RViz.
     * @param traj The time-stamped trajectory.
     * @param frame_id The coordinate frame to use (default: "map").
     */
    void publishTrajectory(const std::vector<TrajectoryPoint>& traj, const std::string& frame_id = "map");

private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_pub_;
};
