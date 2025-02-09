#pragma once

#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <Eigen/Dense>

namespace krakel
{
/**
 * @brief
 *
 * @param model
 * @return std::vector<Eigen::VectorXd> Control points of the resulting spline
 */
std::vector<Eigen::VectorXd> solveWithDirectCollocation(pinocchio::Model& model, const int num_control_points = 100,
                                                        const int num_collocation_points = 50);

std::vector<Eigen::VectorXd> interpolateInJointSpace(const Eigen::VectorXd& q0, const Eigen::VectorXd& q_goal,
                                                     const double step_size = 0.01);
}  // namespace krakel
