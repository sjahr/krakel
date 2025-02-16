#pragma once

#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <cmath>

namespace krakel
{
struct BSpline
{
  std::vector<Eigen::VectorXd> control_points;
  std::vector<double> knots;
  int degree = 3;
};

// DeBoor's algorithm for B-spline basis function
Eigen::VectorXd deBoor(const BSpline& spline, double t);

}  // namespace krakel
