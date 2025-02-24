#pragma once

#include <Eigen/Dense>
#include <vector>
#include <pinocchio/autodiff/casadi.hpp>
namespace krakel
{

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
 * @brief Finds the appropriate knot span for the given parameter t.
 *
 * @details It searches through the knots to find the span where t lies, which helps determine the starting control
 * points for De Boor’s algorithm.
 *
 * @return The index of the knot span.
 */
int findKnotSpan(double t, const std::vector<double>& knots, int degree);

/**
 * @brief Computes the value of the B-spline at the parameter t
 *
 * @details It starts with the control points corresponding to the found knot span and iteratively refines the
 * approximation based on the degree of the spline.
 *
 * @return Value of BSpline at parameter t.
 */
Eigen::VectorXd deBoor(const BSpline& spline, double t);

/**
 * @brief Generated a clamped uniform knot vector for a B-spline.
 *
 * @param num_control_points Number of control points.
 * @param degree Spline degree.
 * @return Konot point vector for BSpline.
 */
std::vector<double> generateClampedUniformKnotVector(int num_control_points, int degree);

/**
 * @brief Interpolates a path using a B-spline.
 *
 * @param path Joint space path to be interpolated with the B-spline.
 * @param degree Degree of the B-spline.
 * @return Spline that represents the interpolated path.
 */
BSpline interpolatePath(const std::vector<Eigen::VectorXd>& path, int degree);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// B-spline cassadi implementations for optimization.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

using namespace casadi;

/**
 * @brief Creates a function that evaluates the B-spline basis functions.
 *
 * @param num_control_points Number of control points.
 * @param degree Degree of the B-spline.
 * @return Function that evaluates the B-spline basis functions.
 */
casadi::Function createBasisEvaluator(const int num_control_points, const int degree);

/**
 * @brief Computes the B-spline basis function value.
 *
 * This function calculates the value of the B-spline basis function for a given control point index, degree, parameter,
 * and knot vector.
 *
 * @param control_point_index The index of the control point.
 * @param degree The degree of the B-spline.
 * @param parameter The parameter value at which to evaluate the basis function.
 * @param knot_vector The knot vector defining the B-spline.
 * @return The value of the B-spline basis function at the given parameter.
 */
casadi::MX bSplineBasis(int control_point_index, int degree, const casadi::MX& parameter,
                        const std::vector<double>& knot_vector);

/**
 * @brief Evaluates the B-spline at a given parameter value.
 *
 * This function evaluates the B-spline at a given parameter value using the control points, basis evaluator function,
 * and knot vector.
 *
 * @param control_points The control points of the B-spline.
 * @param parameter The parameter value at which to evaluate the B-spline.
 * @param basis_evaluator The function that evaluates the B-spline basis functions.
 * @param derivative_order The order of the derivative to evaluate.
 * @param spline_degree The degree of the B-spline.
 * @param start_time The start time of the B-spline.
 * @param end_time The end time of the B-spline.
 * @return The value of the B-spline at the given parameter value.
 */
casadi::MX evaluateBSpline(const casadi::MX& control_points, double parameter, const casadi::Function& basis_evaluator,
                           int derivative_order, const int spline_degree, const double start_time,
                           const double end_time);

}  // namespace krakel
