#include "trajectory_tracking/path_smoother.hpp"
#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>

using Eigen::VectorXd;
using std::pair;
using std::vector;

/**
 * @class PathSmoother
 * @brief Provides a method to smooth a sequence of 2D waypoints using cubic spline interpolation.
 */

/**
 * @brief Smooths a set of 2D waypoints using cubic B-spline interpolation.
 *
 * This function constructs a cubic B-spline in 2D using Eigen's unsupported spline module.
 * It assumes a uniform parameterization over the input waypoints and samples the resulting spline
 * at a fixed resolution to return a denser, smoother trajectory.
 *
 * Steps:
 * 1. Converts the input list of (x, y) waypoints into a 2×N Eigen matrix.
 * 2. Constructs a parameter vector `t` from 0 to N-1.
 * 3. Fits a 2D spline of degree 3 using Eigen's SplineFitting.
 * 4. Evaluates the spline at `resolution` uniformly spaced intervals along `t`.
 *
 * @param waypoints A vector of 2D points represented as pairs (x, y).
 *                  Must contain at least 2 waypoints to perform smoothing.
 * @return A new vector of smoothed 2D points with higher resolution,
 *         maintaining the general path of the input.
 *
 * @note Requires the unsupported Eigen module `SplineFitting`.
 * @note If fewer than 2 points are provided, the input is returned unchanged.
 */
vector<pair<double, double>> PathSmoother::smooth(const vector<pair<double, double>>& waypoints) {
    if (waypoints.size() < 2) return waypoints;

    size_t n = waypoints.size();

    // Create uniform parameter vector t = [0, 1, 2, ..., n-1]
    VectorXd t(n);
    for (size_t i = 0; i < n; ++i)
        t(i) = static_cast<double>(i);

    // Convert waypoints to 2×N Eigen matrix (first row: x, second row: y)
    Eigen::Matrix<double, 2, Eigen::Dynamic> points(2, n);
    for (size_t i = 0; i < n; ++i) {
        points(0, i) = waypoints[i].first;
        points(1, i) = waypoints[i].second;
    }

    // Define 2D spline type and perform cubic spline interpolation
    using Spline2d = Eigen::Spline<double, 2>;
    Spline2d spline = Eigen::SplineFitting<Spline2d>::Interpolate(points, 3, t);

    // Sample the spline at a fixed resolution
    vector<pair<double, double>> result;
    int resolution = 200;
    for (int i = 0; i < resolution; ++i) {
        double ti = (static_cast<double>(i) / (resolution - 1)) * (n - 1);
        Eigen::Vector2d point = spline(ti);
        result.emplace_back(point(0), point(1));
    }

    return result;
}
