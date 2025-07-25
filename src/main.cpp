#include "rclcpp/rclcpp.hpp"
#include "trajectory_tracking/path_smoother.hpp"
#include "trajectory_tracking/trajectory_generator.hpp"
#include "trajectory_tracking/trajectory_visualizer.hpp"
#include "trajectory_tracking/utils.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

/**
 * @brief Load 2D waypoints from a CSV file
 * Format: x,y (without header)
 */
std::vector<std::pair<double, double>> loadWaypointsFromCSV(const std::string& filename) {
    std::vector<std::pair<double, double>> waypoints;
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open CSV file: " + filename);
    }

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string x_str, y_str;
        std::getline(ss, x_str, ',');
        std::getline(ss, y_str, ',');

        try {
            double x = std::stod(x_str);
            double y = std::stod(y_str);
            waypoints.emplace_back(x, y);
        } catch (...) {
            RCLCPP_WARN(rclcpp::get_logger("CSVParser"), "Skipping invalid line: %s", line.c_str());
        }
    }

    return waypoints;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("trajectory_tracking_node");

    RCLCPP_INFO(node->get_logger(), "Trajectory tracking node started.");

    PathSmoother smoother;
    TrajectoryGenerator generator;
    TrajectoryVisualizer visualizer(node);
    // Controller controller;
    // RobotInterface robot(node);

    std::string csv_file = "/home/jhansi/assignment/src/trajectory_tracking/scripts/waypoints.csv";  // Adjust or load from param
    std::vector<std::pair<double, double>> waypoints;

    try {
        waypoints = loadWaypointsFromCSV(csv_file);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Error loading waypoints: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    // Smooth and generate
    auto smooth_path = smoother.smooth(waypoints);
    auto trajectory = generator.generate(smooth_path);

    // Visualize and Export
    visualizer.publishPath(smooth_path, "map");
    visualizer.publishTrajectory(trajectory, "map");

    exportPathToCSV(smooth_path, "smooth_path.csv");
    exportTrajectoryToCSV(trajectory, "trajectory.csv");

    RCLCPP_INFO(node->get_logger(), "Trajectory published and saved to CSV.");

    rclcpp::spin_some(node);  // Optional spin (for visualization)
    rclcpp::shutdown();
    return 0;
}
