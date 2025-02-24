#include "optimal_planning/b_spline.hpp"

namespace krakel
{
int findKnotSpan(double parameter, const std::vector<double>& knot_vector, int degree)
{
  int numcontrol_points = knot_vector.size() - degree - 1;
  if (parameter == knot_vector[numcontrol_points])
  {
    return numcontrol_points - 1;  // If parameter is the last knot, return the second-to-last span
  }
  for (int i = degree; i <= numcontrol_points; ++i)
  {
    if (parameter >= knot_vector[i] && parameter < knot_vector[i + 1])
    {
      return i;
    }
  }
  return degree;  // Default case, for boundary conditions
}

Eigen::VectorXd deBoor(const BSpline& spline, double parameter)
{
  int degree = spline.degree;
  int numcontrol_points = spline.control_points.size();

  // Find the knot span
  int knotSpan = findKnotSpan(parameter, spline.knots, degree);

  // Create a copy of the control points to store intermediate results
  std::vector<Eigen::VectorXd> intermediatePoints(degree + 1);

  // Initialize the first set of points with the control points
  for (int i = 0; i <= degree; ++i)
  {
    intermediatePoints[i] = spline.control_points[knotSpan - degree + i];
  }

  // Perform the De Boor algorithm (recursive calculation)
  for (int r = 1; r <= degree; ++r)
  {
    for (int j = degree; j >= r; --j)
    {
      double alpha = (parameter - spline.knots[knotSpan - degree + j]) /
                     (spline.knots[knotSpan - degree + j + 1] - spline.knots[knotSpan - degree + j]);
      intermediatePoints[j] = (1 - alpha) * intermediatePoints[j - 1] + alpha * intermediatePoints[j];
    }
  }

  // Return the final value (at the top level, i.e., intermediatePoints[degree])
  return intermediatePoints[degree];
}

std::vector<double> generateClampedUniformKnotVector(int num_control_points, int degree)
{
  int m = num_control_points + degree + 1;  // Number of knots
  std::vector<double> knots(m);

  // Clamped knots: first and last knots are repeated
  for (int i = 0; i <= degree; ++i)
  {
    knots[i] = 0.0;  // First degree knots are 0
  }
  for (int i = degree + 1; i < m - degree; ++i)
  {
    knots[i] = (i - degree) / static_cast<double>(num_control_points - degree);
  }
  for (int i = m - degree; i < m; ++i)
  {
    knots[i] = 1.0;  // Last degree knots are 1
  }
  return knots;
}

casadi::Function createBasisEvaluator(const int num_control_points, const int degree)
{
  // Symbolic time variable
  MX t = MX::sym("t");

  // Create uniform knot vector
  std::vector<double> knots = generateClampedUniformKnotVector(num_control_points, degree);

  // Create symbolic basis functions
  std::vector<MX> basis_functions;
  for (int i = 0; i < num_control_points; ++i)
  {
    MX basis = bSplineBasis(i, degree, t, knots);
    basis_functions.push_back(basis);
  }

  // Create function
  return Function("basis_eval", { t }, { vertcat(basis_functions) });
}

casadi::MX bSplineBasis(int control_point_index, int degree, const casadi::MX& parameter,
                        const std::vector<double>& knot_vector)
{
  if (degree == 0)
  {
    return (parameter >= knot_vector[control_point_index] && parameter < knot_vector[control_point_index + 1]);
  }

  MX firstTerm = 0, secondTerm = 0;

  // First term
  if (knot_vector[control_point_index + degree] - knot_vector[control_point_index] != 0)
  {
    firstTerm = (parameter - knot_vector[control_point_index]) /
                (knot_vector[control_point_index + degree] - knot_vector[control_point_index]) *
                bSplineBasis(control_point_index, degree - 1, parameter, knot_vector);
  }

  // Second term
  if (knot_vector[control_point_index + degree + 1] - knot_vector[control_point_index + 1] != 0)
  {
    secondTerm = (knot_vector[control_point_index + degree + 1] - parameter) /
                 (knot_vector[control_point_index + degree + 1] - knot_vector[control_point_index + 1]) *
                 bSplineBasis(control_point_index + 1, degree - 1, parameter, knot_vector);
  }

  return firstTerm + secondTerm;
}

casadi::MX evaluateBSpline(const casadi::MX& control_points, double parameter, const casadi::Function& basis_evaluator,
                           int derivative_order, const int degree, const double start_time, const double end_time)
{
  // Evaluate basis functions
  std::vector<MX> basisFunctions = basis_evaluator(std::vector<MX>{ parameter });
  MX basisMatrix = basisFunctions[0];

  // For derivatives, we need to implement B-spline derivative computation
  if (derivative_order > 0)
  {
    // This is a simplified version - in practice, you'd implement proper B-spline derivatives
    double scale = std::pow(degree / (end_time - start_time), derivative_order);
    basisMatrix = scale * basisMatrix;  // This is an approximation
  }

  // Multiply with control points
  return mtimes(control_points, basisMatrix);
}
}  // namespace krakel
