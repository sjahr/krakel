#include "optimal_planning/eigen_casadi_conversions.hpp"

namespace krakel
{

// Convert from Eigen::VectorXd to casadi::MX
casadi::MX convertEigenToCasadiVector(const Eigen::VectorXd& eigen_vector)
{
  int size = eigen_vector.size();
  casadi::MX casadi_vector = casadi::MX::zeros(size, 1);

  for (int i = 0; i < size; i++)
  {
    casadi_vector(i) = eigen_vector(i);
  }

  return casadi_vector;
}

// Convert from casadi::MX to Eigen::VectorXd
Eigen::VectorXd convertCasadiToEigenVector(const casadi::MX& casadi_vector)
{
  // Convert MX to DM to access numerical values
  casadi::DM numerical_vector = casadi::DM(casadi_vector);

  int size = numerical_vector.rows();
  Eigen::VectorXd eigen_vector(size);

  for (int i = 0; i < size; i++)
  {
    eigen_vector(i) = static_cast<double>(numerical_vector(i));
  }

  return eigen_vector;
}

casadi::MX convertEigenVectorsToCasadiMX(const std::vector<Eigen::VectorXd>& eigen_vector)
{
  // Check if the vector is empty
  if (eigen_vector.empty())
  {
    return casadi::MX();
  }

  // Get the dimension of each control point
  int n_dim = eigen_vector[0].size();
  int n_points = eigen_vector.size();

  // Create a matrix with n_points rows and n_dim columns
  casadi::MX casadi_mx = casadi::MX::zeros(n_points, n_dim);

  // Fill the matrix with values from the Eigen vectors
  for (int i = 0; i < n_points; i++)
  {
    // Ensure consistent dimensions
    if (eigen_vector[i].size() != n_dim)
    {
      throw std::runtime_error("Inconsistent dimensions in control points");
    }

    for (int j = 0; j < n_dim; j++)
    {
      // Set each element in the MX matrix
      casadi_mx(i, j) = eigen_vector[i](j);
    }
  }

  return casadi_mx;
}
}  // namespace krakel
