#pragma once

#include <Eigen/Dense>
#include <vector>

namespace krakel
{
/**
 * @brief B-spline implementation.
 *
 */
struct BSpline
{
  std::vector<Eigen::VectorXd> control_points;
  std::vector<double> knots;
  int degree = 3;
};

/**
 * @brief Finds the appropriate knot span for the given parameter t.
 *
 * @details It searches through the knots to find the span where t lies, which helps determine the starting control
 * points for De Boor’s algorithm.
 *
 * @return The index of the knot span.
 */
int findKnotSpan(double t, const std::vector<double>& knots, int degree);

/**
 * @brief Computes the value of the B-spline at the parameter tt
 *
 * @details It starts with the control points corresponding to the found knot span and iteratively refines the
 * approximation based on the degree of the spline.
 *
 * @return Value of BSpline at parameter t.
 */
Eigen::VectorXd deBoor(const BSpline& spline, double t);

// Generate Knot Vector for B-spline.
std::vector<double> generateClampedUniformKnotVector(int num_control_points, int degree);

// Interpolate path with B-spline.
BSpline interpolatePath(const std::vector<Eigen::VectorXd>& path, int degree);
}  // namespace krakel
