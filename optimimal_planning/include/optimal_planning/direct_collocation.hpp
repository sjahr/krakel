#pragma once

#include <pinocchio/autodiff/casadi.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <Eigen/Dense>

#include "optimal_planning/b_spline.hpp"

namespace krakel
{
/**
 * @brief
 *
 * @param model
 * @return std::vector<Eigen::VectorXd> Control points of the resulting spline
 */
std::vector<Eigen::VectorXd> solveWithDirectCollocation(pinocchio::Model& model, BSpline& initial_spline);

std::vector<Eigen::VectorXd> interpolateInJointSpace(const Eigen::VectorXd& q0, const Eigen::VectorXd& q_goal,
                                                     const double step_size = 0.01);

}  // namespace krakel
