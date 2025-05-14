#include <pinocchio/autodiff/casadi.hpp>
#include <Eigen/Dense>
#include <vector>

namespace krakel
{
/**
 * @brief Convert a vector of Eigen::VectorXd to casadi::MX
 * @param eigen_vector Vector of Eigen::VectorXd control points
 * @return casadi::MX representation of the control points
 */
casadi::MX convertEigenVectorsToCasadiMX(const std::vector<Eigen::VectorXd>& eigen_vector);

// Convert from Eigen::VectorXd to casadi::MX
casadi::MX convertEigenToCasadiVector(const Eigen::VectorXd& eigen_vector);

// Convert from casadi::MX to Eigen::VectorXd
Eigen::VectorXd convertCasadiToEigenVector(const casadi::MX& casadi_vector);
}  // namespace krakel
