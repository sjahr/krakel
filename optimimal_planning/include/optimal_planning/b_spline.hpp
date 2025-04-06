#pragma once

#include <Eigen/Dense>
#include <vector>
#include <pinocchio/autodiff/casadi.hpp>
namespace krakel
{
using namespace casadi;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// B-spline implementation using std library and Eigen
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
 * Generates a clamped uniform knot vector for B-splines
 * @param num_control_points Number of control points
 * @param degree B-spline degree
 * @return Knot vector with clamped ends
 */
std::vector<double> generateClampedUniformKnotVector(int num_control_points, int degree);

/**
 * Recursive calculation of B-spline basis function.
 * @param control_point_index Index of control point
 * @param degree B-spline degree
 * @param parameter Parameter value (typically between 0 and 1)
 * @param knot_vector Knot vector
 * @return Value of basis function
 */
casadi::MX bSplineBasis(int control_point_index, int degree, const casadi::MX& parameter,
                        const std::vector<double>& knot_vector);

/**
 * Creates a CasADI function to evaluate all basis functions at a given parameter value
 * @param num_control_points Number of control points
 * @param degree B-spline degree
 * @return CasADI function that maps parameter value to basis function values
 */
casadi::Function createBasisEvaluator(const int num_control_points, const int degree);

/**
 * Calculate control points for B-spline derivative
 * @param control_points Original control points
 * @param degree B-spline degree
 * @param knots Knot vector
 * @return New control points for derivative calculation
 */
casadi::MX deriveControlPoints(const casadi::MX& control_points, int degree, const std::vector<double>& knots);

/**
 * Evaluate a B-spline at a parameter value
 * @param control_points Control points matrix (each row is a control point)
 * @param parameter Parameter value (typically between 0 and 1)
 * @param basis_evaluator Function that evaluates basis functions
 * @return B-spline value at parameter
 */
casadi::MX evaluateBSpline(const casadi::MX& control_points, const casadi::MX& parameter,
                           const casadi::Function& basis_evaluator);

/**
 * Evaluate a B-spline derivative at a parameter value
 * @param control_points Control points matrix (each row is a control point)
 * @param parameter Parameter value (typically between 0 and 1)
 * @param degree B-spline degree
 * @param knots Knot vector
 * @param derivative_order Order of derivative (1 for first derivative, etc.)
 * @param time_scale Time scaling factor (for time domain adjustment)
 * @return B-spline derivative at parameter
 */
casadi::MX evaluateBSplineDerivative(const casadi::MX& control_points, const casadi::MX& parameter, const int degree,
                                     const std::vector<double>& knots, const int derivative_order,
                                     const double time_scale = 1.0);
}  // namespace krakel
