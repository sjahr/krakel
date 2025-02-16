#include "optimal_planning/b_spline_parameterization.hpp"

namespace krakel
{

Eigen::VectorXd deBoor(const BSpline& spline, double t)
{
  // Find the index of the first control point that is greater than t
  int controlPointIndex = 0;
  while (controlPointIndex < spline.control_points.size() - 1 && spline.knots[controlPointIndex + 1] < t)
  {
    controlPointIndex++;
  }

  // Initialize the basis functions
  std::vector<Eigen::VectorXd> basisFunctions(spline.control_points.size());
  for (int i = 0; i < spline.control_points.size(); ++i)
  {
    basisFunctions[i] = Eigen::VectorXd::Zero(spline.control_points.size());
    basisFunctions[i][i] = 1.0;
  }

  // Compute the basis functions
  for (int degree = 1; degree <= spline.degree; ++degree)
  {
    for (int j = controlPointIndex; j >= controlPointIndex - degree + 1; --j)
    {
      double alpha = (t - spline.knots[j]) / (spline.knots[j + degree] - spline.knots[j]);
      basisFunctions[j] = (1 - alpha) * basisFunctions[j - 1] + alpha * basisFunctions[j];
    }
  }

  return basisFunctions[controlPointIndex];
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

}  // namespace krakel
