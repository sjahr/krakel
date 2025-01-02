#include "direct_collocation.hpp"
//#include <tinysplinecpp.h>  // Or another B-spline library

namespace krakel
{  // Define the cost function
double costFunction(const std::vector<Eigen::VectorXd>& controlPoints)
{
  double cost = 0.0;
  // Add smoothness or energy cost
  for (size_t i = 1; i < controlPoints.size(); ++i)
  {
    cost += (controlPoints[i] - controlPoints[i - 1]).squaredNorm();
  }
  return cost;
}

// Dynamics constraint
Eigen::VectorXd dynamicsConstraint(const pinocchio::Model& model, pinocchio::Data& data, const Eigen::VectorXd& q,
                                   const Eigen::VectorXd& v, const Eigen::VectorXd& u)
{
  Eigen::VectorXd a = pinocchio::aba(model, data, q, v, u);  // Forward dynamics
  return a;
}

// Main optimization routine
std::vector<Eigen::VectorXd> solveWithDirectCollocation(pinocchio::Model& model)
{
  /*pinocchio::Data data(model);

  // B-spline setup
  int numKnots = 10;         // Number of knot points
  int splineOrder = 3;       // Cubic B-spline
  double timeHorizon = 1.0;  // Total time
  double dt = timeHorizon / (numKnots - 1);

  // Initialize B-spline control points
  std::vector<Eigen::VectorXd> controlPoints(numKnots, Eigen::VectorXd::Zero(model.nq));
  controlPoints.front() = Eigen::VectorXd::Zero(model.nq);  // Initial state
  controlPoints.back() = Eigen::VectorXd::Ones(model.nq);   // Final state

  // Optimization (e.g., gradient descent or a solver like IPOPT)
  for (int iter = 0; iter < 100; ++iter)
  {
    for (int i = 1; i < numKnots - 1; ++i)
    {
      Eigen::VectorXd q = controlPoints[i];
      Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);  // Initial velocity guess
      Eigen::VectorXd u = Eigen::VectorXd::Zero(model.nv);  // Initial control guess

      // Enforce dynamics at collocation points
      Eigen::VectorXd a = dynamicsConstraint(model, data, q, v, u);

      // Update control points
      controlPoints[i] -= 0.01 * a;  // Gradient step
    }
  }

  // Output optimized trajectory
  for (const auto& cp : controlPoints)
  {
    std::cout << "Control point: " << cp.transpose() << std::endl;
  }*/
return std::vector<Eigen::VectorXd>(10, Eigen::VectorXd::Zero(model.nq));
}
}  // namespace krakel
