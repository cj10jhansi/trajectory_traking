#pragma once
#include <vector>
#include <utility>

/**
 * @struct TrajectoryPoint
 * @brief A time-parameterized trajectory point in 2D space.
 */
struct TrajectoryPoint {
    double x;     ///< X position
    double y;     ///< Y position
    double time;  ///< Time stamp in seconds
};

/**
 * @class TrajectoryGenerator
 * @brief Generates a time-parameterized trajectory from a smoothed path.
 */
class TrajectoryGenerator {
public:
    /**
     * @brief Generate a trajectory from a given path.
     *
     * This function assumes constant speed and assigns time stamps to each point.
     *
     * @param path A vector of smoothed (x, y) coordinates.
     * @return A vector of trajectory points with time information.
     */
    std::vector<TrajectoryPoint> generate(const std::vector<std::pair<double, double>>& path);
};
