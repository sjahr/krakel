#include "optimal_planning/direct_collocation.hpp"

#include "optimal_planning/eigen_casadi_conversions.hpp"

namespace krakel
{

using namespace casadi;
namespace
{
constexpr int kCubicBSplineDegree = 3;
}  // namespace

// Dynamics constraint
Eigen::VectorXd dynamicsConstraint(const pinocchio::Model& model, pinocchio::Data& data, const Eigen::VectorXd& q,
                                   const Eigen::VectorXd& v, const Eigen::VectorXd& u)
{
  Eigen::VectorXd a = pinocchio::aba(model, data, q, v, u);  // Forward dynamics
  return a;
}

std::vector<Eigen::VectorXd> interpolateInJointSpace(const Eigen::VectorXd& q0, const Eigen::VectorXd& q_goal,
                                                     const double step_size)
{
  std::vector<Eigen::VectorXd> path;

  const double distance = (q_goal - q0).norm();
  const int num_steps = std::ceil(distance / step_size);
  path.reserve(num_steps);

  for (int i = 0; i < num_steps; ++i)
  {
    // Calculate interpolation parameter lambda (between 0 and 1)
    double lambda = double(i) / (num_steps - 1);

    // Perform linear interpolation between q0 and q_goal
    Eigen::VectorXd q_interp = (1 - lambda) * q0 + lambda * q_goal;
    path.emplace_back(q_interp);
  }
  return path;
}

// Main optimization routine
std::vector<Eigen::VectorXd> solveWithDirectCollocation(pinocchio::Model& model, BSpline& initial_spline)
{
  pinocchio::Data data(model);
  auto bspline_basis_evaluator =
      krakel::createBasisEvaluator(initial_spline.control_points.size(), initial_spline.degree);

  // Initialize optimizer
  auto opti = casadi::Opti();

  // Decision variables: control points
  MX control_points = convertEigenVectorsToCasadiMX(initial_spline.control_points);

  ////// Constraints //////
  // Initial state constraint

  ////// Objective function //////

  ////// Create solver //////
  casadi::Dict solver_opts;
  solver_opts["ipopt.tol"] = 1e-4;
  solver_opts["ipopt.max_iter"] = 3000;
  solver_opts["print_time"] = 0;
  solver_opts["ipopt.print_level"] = 0;

  // Set solver
  opti.solver("ipopt", solver_opts);

  ///// Solve the optimization problem
  auto result = opti.solve();

  return std::vector<Eigen::VectorXd>(10, Eigen::VectorXd::Zero(model.nq));
}
}  // namespace krakel
