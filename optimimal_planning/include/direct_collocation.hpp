#pragma once

#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <Eigen/Dense>

namespace krakel
{
    double costFunction(const std::vector<Eigen::VectorXd>& controlPoints);
    /**
     * @brief 
     * 
     * @param model 
     * @return std::vector<Eigen::VectorXd> Control points of the resulting spline
     */
    std::vector<Eigen::VectorXd> solveWithDirectCollocation(pinocchio::Model& model);
}