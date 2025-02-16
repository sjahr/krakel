#include "optimal_planning/b_spline.hpp"

namespace krakel
{
int findKnotSpan(double parameter, const std::vector<double>& knotVector, int degree)
{
  int numControlPoints = knotVector.size() - degree - 1;
  if (parameter == knotVector[numControlPoints])
  {
    return numControlPoints - 1;  // If parameter is the last knot, return the second-to-last span
  }
  for (int i = degree; i <= numControlPoints; ++i)
  {
    if (parameter >= knotVector[i] && parameter < knotVector[i + 1])
    {
      return i;
    }
  }
  return degree;  // Default case, for boundary conditions
}

Eigen::VectorXd deBoor(const BSpline& spline, double parameter)
{
  int degree = spline.degree;
  int numControlPoints = spline.control_points.size();

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
}  // namespace krakel
