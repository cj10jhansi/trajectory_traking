#pragma once
#include <vector>
#include <utility>
#include <string>
#include <fstream>
#include "trajectory_tracking/trajectory_generator.hpp"

/**
 * @brief Export a path to CSV.
 */
inline void exportPathToCSV(const std::vector<std::pair<double, double>>& path, const std::string& filename) {
    std::ofstream out(filename);
    out << "x,y\n";
    for (const auto& [x, y] : path) {
        out << x << "," << y << "\n";
    }
}

/**
 * @brief Export a trajectory to CSV.
 */
inline void exportTrajectoryToCSV(const std::vector<TrajectoryPoint>& traj, const std::string& filename) {
    std::ofstream out(filename);
    out << "x,y,time\n";
    for (const auto& pt : traj) {
        out << pt.x << "," << pt.y << "," << pt.time << "\n";
    }
}
