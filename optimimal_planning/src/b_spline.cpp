#include "optimal_planning/b_spline.hpp"

namespace krakel
{
std::vector<double> generateClampedUniformKnotVector(int num_control_points, int degree)
{
  assert(num_control_points > degree && "Number of control points must be greater than degree");

  int m = num_control_points + degree + 1;  // Total number of knots
  std::vector<double> knots(m);

  // Clamped knots: first and last knots are repeated (degree+1) times
  for (int i = 0; i <= degree; ++i)
  {
    knots[i] = 0.0;  // First degree+1 knots are 0
  }

  // Internal knots are uniformly distributed
  for (int i = degree + 1; i < m - degree; ++i)
  {
    knots[i] = (i - degree) / static_cast<double>(num_control_points - degree);
  }

  // Last degree+1 knots are 1
  for (int i = m - degree; i < m; ++i)
  {
    knots[i] = 1.0;
  }

  return knots;
}

casadi::MX bSplineBasis(int control_point_index, int degree, const casadi::MX& parameter,
                        const std::vector<double>& knot_vector)
{
  // Base case: degree 0
  if (degree == 0)
  {
    // Standard interval check
    MX condition1 = parameter >= knot_vector[control_point_index] && parameter < knot_vector[control_point_index + 1];

    // Special case for the end point
    MX condition2 = parameter == knot_vector.back() && control_point_index == static_cast<int>(knot_vector.size()) - 2;

    return if_else(condition1 || condition2, 1.0, 0.0);
  }

  // Recursive case
  MX firstTerm = 0, secondTerm = 0;

  // First term
  double denom1 = knot_vector[control_point_index + degree] - knot_vector[control_point_index];
  if (denom1 > 0)
  {
    MX num1 = parameter - knot_vector[control_point_index];
    MX term1 = num1 / denom1 * bSplineBasis(control_point_index, degree - 1, parameter, knot_vector);
    firstTerm = term1;
  }

  // Second term
  double denom2 = knot_vector[control_point_index + degree + 1] - knot_vector[control_point_index + 1];
  if (denom2 > 0)
  {
    MX num2 = knot_vector[control_point_index + degree + 1] - parameter;
    MX term2 = num2 / denom2 * bSplineBasis(control_point_index + 1, degree - 1, parameter, knot_vector);
    secondTerm = term2;
  }

  return firstTerm + secondTerm;
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

  // Create function that returns a column vector of basis functions
  return Function("basis_eval", { t }, { vertcat(basis_functions) });
}

casadi::MX deriveControlPoints(const casadi::MX& control_points, int degree, const std::vector<double>& knots)
{
  int n = control_points.size1();
  MX new_control_points = MX::zeros(n - 1, control_points.size2());

  for (int i = 0; i < n - 1; ++i)
  {
    double denom = knots[i + degree + 1] - knots[i + 1];
    if (denom > 0)
    {
      double factor = degree / denom;
      new_control_points(i, Slice()) = factor * (control_points(i + 1, Slice()) - control_points(i, Slice()));
    }
  }

  return new_control_points;
}

casadi::MX evaluateBSpline(const casadi::MX& control_points, const casadi::MX& parameter,
                           const casadi::Function& basis_evaluator)
{
  // Evaluate basis functions at parameter
  std::vector<MX> result = basis_evaluator({ parameter });
  MX basis_values = result[0];

  // Multiply control points by basis functions
  return mtimes(transpose(basis_values), control_points);
}

casadi::MX evaluateBSplineDerivative(const casadi::MX& control_points, const casadi::MX& parameter, const int degree,
                                     const std::vector<double>& knots, const int derivative_order,
                                     const double time_scale)
{
  // Base case: no derivative
  if (derivative_order == 0)
  {
    Function basis_eval = createBasisEvaluator(control_points.size1(), degree);
    return evaluateBSpline(control_points, parameter, basis_eval);
  }

  // Compute new control points for the derivative
  MX new_control_points = deriveControlPoints(control_points, degree, knots);

  // For time scaling, we need to apply the chain rule (dt/ds)
  double scale_factor = pow(time_scale, derivative_order);

  // Recursively compute higher-order derivatives
  if (derivative_order > 1)
  {
    // Reduced degree for the derivative
    return scale_factor * evaluateBSplineDerivative(new_control_points, parameter, degree - 1, knots,
                                                    derivative_order - 1, time_scale);
  }
  else
  {
    // First derivative
    Function basis_eval = createBasisEvaluator(new_control_points.size1(), degree - 1);
    return scale_factor * evaluateBSpline(new_control_points, parameter, basis_eval);
  }
}

casadi::MX evaluateBSplineTrajectory(const casadi::MX& control_points, const casadi::MX& t, int degree,
                                     double start_time, double end_time, int derivative_order)
{
  // Generate knot vector
  std::vector<double> knots = generateClampedUniformKnotVector(control_points.size1(), degree);

  // Map time parameter to [0, 1]
  casadi::MX normalized_t = (t - start_time) / (end_time - start_time);

  // Time scaling factor for derivatives (dt/ds = end_time - start_time)
  double time_scale = 1.0 / (end_time - start_time);

  // Evaluate at the normalized parameter
  return evaluateBSplineDerivative(control_points, normalized_t, degree, knots, derivative_order, time_scale);
}
}  // namespace krakel
