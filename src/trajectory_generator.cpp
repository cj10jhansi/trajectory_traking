#include "trajectory_tracking/trajectory_generator.hpp"
#include <cmath>

using std::pair;
using std::vector;

/**
 * @brief Generate time-stamped trajectory from a 2D path.
 *
 * Time is calculated based on constant linear velocity between consecutive points.
 *
 * @param path Smoothed (x, y) path
 * @return Time-parameterized trajectory
 */
vector<TrajectoryPoint> TrajectoryGenerator::generate(const vector<pair<double, double>>& path) {
    vector<TrajectoryPoint> trajectory;
    if (path.size() < 2) return trajectory;

    double current_time = 0.0;
    double speed = 0.5; // meters/second

    for (size_t i = 0; i < path.size(); ++i) {
        if (i > 0) {
            double dx = path[i].first - path[i - 1].first;
            double dy = path[i].second - path[i - 1].second;
            double dist = std::sqrt(dx * dx + dy * dy);
            current_time += dist / speed;
        }

        trajectory.push_back({path[i].first, path[i].second, current_time});
    }

    return trajectory;
}
