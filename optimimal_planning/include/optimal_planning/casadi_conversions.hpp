#include <Eigen/Dense>
#include <pinocchio/autodiff/casadi.hpp>

namespace krakel
{
casadi::MX controlPointVectorToMX(const std::vector<Eigen::VectorXd>& eigen_vector)
{
  int rows = eigen_vector.size();
  int cols = eigen_vector[0].size();
  casadi::DM result(rows, cols);

  for (int i = 0; i < rows; ++i)
  {
    for (int j = 0; j < cols; ++j)
    {
      result(i, j) = eigen_vector[i](j);
    }
  }

  return result;
}
}  // namespace krakel
