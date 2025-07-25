#pragma once
#include <vector>
#include <utility>

/**
 * @class PathSmoother
 * @brief Performs cubic spline smoothing on a sequence of 2D waypoints.
 */
class PathSmoother {
public:
    /**
     * @brief Smooth a path using cubic spline interpolation.
     *
     * @param waypoints A vector of (x, y) waypoints to smooth.
     * @return A smoothed vector of (x, y) points.
     */
    std::vector<std::pair<double, double>> smooth(const std::vector<std::pair<double, double>>& waypoints);
};
