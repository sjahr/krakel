#pragma once

#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <cmath>

namespace krakel 
{/**
 * @brief Calculate time between two waypoints using velocity and acceleration constraints
 * 
 * @param distance 
 * @param max_velocity 
 * @param max_acceleration 
 * @return double 
 */
double calculateTimeStep(double distance, double max_velocity, double max_acceleration);

/**
 * @brief Create a non-uniform time step vector based on the path, velocity, and acceleration constraints
 * 
 */
std::vector<double> createNonUniformTimeVector(const std::vector<Eigen::VectorXd>& path, double max_velocity, double max_acceleration);

/**
 * @brief Create a time-parameterized B-spline from the path.
 * 
 */
std::vector<Eigen::VectorXd> createTimeParameterizedBSpline(const std::vector<Eigen::VectorXd>& path, int degree, double total_time, int num_samples, const std::vector<double>& time_vector);
}  // namespace krakel