#include "direct_collocation.hpp"

namespace krakel
{
namespace
{
constexpr int kCubicBSplineDegree = 3;
}

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

  for (int i = 0; i < num_steps; ++i)
  {
    // Calculate interpolation parameter lambda (between 0 and 1)
    double lambda = double(i) / (num_steps - 1);

    // Perform linear interpolation between q0 and q_goal
    Eigen::VectorXd q_interp = (1 - lambda) * q0 + lambda * q_goal;
  }
  return path;
}

// Main optimization routine
std::vector<Eigen::VectorXd> solveWithDirectCollocation(pinocchio::Model& model, const int num_control_points,
                                                        const int num_collocation_points)
{
  // pinocchio::Data data(model);
  //
  // using namespace casadi;
  //
  //// Define trajectory duration
  // SX cs_traj_duration = SX::sym("T");  // Total trajectory time
  //
  //// Define B-spline degree and control points
  //
  //// Control points (decision variables)
  // SX cs_control_points = SX::sym("control_points",  model.nq, num_control_points);
  //
  //// Time discretization for evaluation
  // auto cs_time_points = DM::linspace(0, 1, num_collocation_points);  // Normalized [0, 1]
  //
  //// Define B-spline basis
  // auto knots = DM::linspace(0, 1, num_control_points + kCubicBSplineDegree - 1).nonzeros();
  // Function bspline = Function("bspline", { MX::sym("x") }, { MX::bspline(knots, kCubicBSplineDegree, cs_time_points) });
  //
  //// Evaluate trajectory using B-spline
  // SX trajectory = bspline(cs_control_points);
  //
  //// Dynamics function
  // auto dynamics = [&model, &data](const SX& x, const SX& u) {
  //   Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(x.data(), model.nq);
  //   Eigen::VectorXd v = Eigen::Map<const Eigen::VectorXd>(x.data() + model.nq, model.nv);
  //   Eigen::VectorXd u_eigen = Eigen::Map<const Eigen::VectorXd>(u.data(), model.nv);
  //   Eigen::VectorXd a = pinocchio::aba(model, data, q, v, u_eigen);  // Forward dynamics
  //   SX a_sx = SX::zeros(model.nv);
  //   for (int i = 0; i < model.nv; ++i) {
  //     a_sx(i) = a(i);
  //   }
  //   return vertcat(v, a_sx);
  // };
  //
  //// Collocation constraints
  // SX constraints;
  // SX h = cs_traj_duration / (num_collocation_points - 1);  // Time step size
  // for (int k = 0; k < num_collocation_points - 1; ++k)
  //{
  //   // States and controls
  //   SX x_k = trajectory(Slice(), k);
  //   SX x_kp1 = trajectory(Slice(), k + 1);
  //   SX u_k = SX::sym("u",  model.nq);  // Controls (add control trajectory if needed)
  //
  //  // Discretized dynamics constraint
  //  SX x_next = x_k + h * dynamics(x_k, u_k);
  //  constraints = vertcat(constraints, x_kp1 - x_next);
  //}
  //
  //// Boundary conditions (e.g., start and goal states)
  // SX start_state = SX::zeros( model.nq);  // Replace with actual start
  // SX goal_state = SX::ones( model.nq);    // Replace with actual goal
  // constraints = vertcat(constraints, trajectory(Slice(), 0) - start_state);
  // constraints = vertcat(constraints, trajectory(Slice(), -1) - goal_state);
  //
  //// Define cost function (minimize execution time)
  // SX cost = cs_traj_duration;
  //
  //// Collect all decision variables
  // SX decision_vars = vertcat(reshape(cs_control_points,  model.nq * num_control_points, 1), cs_traj_duration);
  //
  //// Define NLP problem
  // SXDict nlp = { { "x", decision_vars }, { "f", cost }, { "g", constraints } };
  // Function solver = nlpsol("solver", "ipopt", nlp);
  //
  //// Solve the optimization problem
  // std::map<std::string, DM> arg;
  // arg["x0"] = DM::zeros(decision_vars.size());       // Initial guess
  // arg["lbx"] = DM::zeros(decision_vars.size());      // Lower bounds
  // arg["ubx"] = DM::ones(decision_vars.size()) * 10;  // Upper bounds
  // arg["lbg"] = DM::zeros(constraints.size());        // Constraint lower bound
  // arg["ubg"] = DM::zeros(constraints.size());        // Constraint upper bound
  //
  // std::map<std::string, DM> result = solver(arg);
  //
  //// Extract optimal solution
  // DM optimal_vars = result["x"];
  // DM optimal_cs_control_points = optimal_vars(Slice(0,  model.nq * num_control_points));
  // double optimal_T = static_cast<double>(optimal_vars(optimal_vars.size() - 1));
  // std::cout << "Optimal execution time: " << optimal_T << " seconds" << std::endl;

  return std::vector<Eigen::VectorXd>(10, Eigen::VectorXd::Zero(model.nq));
}
}  // namespace krakel
