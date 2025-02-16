#include "optimal_planning/b_spline_parameterization.hpp"

namespace krakel
{
namespace
{
// Function to compute B-spline basis function (Cox-de Boor)
double bSplineBasis(int i, int degree, double t, const std::vector<double>& knots)
{
  if (degree == 0)
  {
    if (t >= knots[i] && t < knots[i + 1])
    {
      return 1.0;
    }
    else
    {
      return 0.0;
    }
  }
  else
  {
    double left = 0.0, right = 0.0;
    if (knots[i + degree] - knots[i] != 0)
    {
      left = (t - knots[i]) / (knots[i + degree] - knots[i]) * bSplineBasis(i, degree - 1, t, knots);
    }
    if (knots[i + degree + 1] - knots[i + 1] != 0)
    {
      right = (knots[i + degree + 1] - t) / (knots[i + degree + 1] - knots[i + 1]) *
              bSplineBasis(i + 1, degree - 1, t, knots);
    }
    return left + right;
  }
}
}  // namespace
// Calculate time between two waypoints using velocity and acceleration constraints
double calculateTimeStep(double distance, double max_velocity, double max_acceleration)
{
  // Time to traverse with constant velocity
  double time_v = distance / max_velocity;

  // Estimate time considering acceleration and deceleration phases
  double time_a = std::sqrt(2 * distance / max_acceleration);

  // Choose the maximum of both times (conservative estimate)
  return std::max(time_v, time_a);
}

std::vector<double> createNonUniformTimeVector(const std::vector<Eigen::VectorXd>& path, double max_velocity,
                                               double max_acceleration)
{
  std::vector<double> time_vector;
  time_vector.push_back(0.0);  // Start at time 0

  for (size_t i = 1; i < path.size(); ++i)
  {
    // Calculate the Euclidean distance between consecutive points
    double distance = (path[i] - path[i - 1]).norm();

    // Calculate the time it takes to travel this distance based on velocity and acceleration limits
    double time = calculateTimeStep(distance, max_velocity, max_acceleration);

    // Accumulate the time for each segment
    time_vector.push_back(time_vector.back() + time);
  }

  return time_vector;
}

// Create the B-spline from the path, now with time as a parameter
std::vector<Eigen::VectorXd> createTimeParameterizedBSpline(const std::vector<Eigen::VectorXd>& path, int degree,
                                                            double total_time, int num_samples,
                                                            const std::vector<double>& time_vector)
{
  int num_control_points = path.size();
  int num_dofs = path[0].size();  // Dimension of the trajectory (e.g., 7 for a 7-DoF robot)

  // Create the knot vector, scaled by the time vector
  std::vector<double> knots;
  for (int i = 0; i <= num_control_points + degree; ++i)
  {
    knots.push_back(i / static_cast<double>(num_control_points + degree));
  }

  // Sample the B-spline at different time intervals, adjusted by the time_vector
  std::vector<Eigen::VectorXd> trajectory(num_samples);

  // Iterate over samples
  for (int i = 0; i < num_samples; ++i)
  {
    // Map time t from 0 to total_time
    double t = i * total_time / (num_samples - 1);

    // Find the corresponding control point time
    int segment = 0;
    while (segment < time_vector.size() - 1 && t > time_vector[segment + 1])
    {
      segment++;
    }

    // Map time t to the local segment parameter for the B-spline evaluation
    double time = (t - time_vector[segment]) / (time_vector[segment + 1] - time_vector[segment]);

    // Evaluate the B-spline at this time
    Eigen::VectorXd spline_value(num_dofs);
    spline_value.setZero();
    for (int j = 0; j < num_control_points; ++j)
    {
      double basis_value = bSplineBasis(j, degree, time, knots);
      spline_value += basis_value * path[j];
    }
    trajectory[i] = spline_value;
  }

  return trajectory;
}

}  // namespace krakel
