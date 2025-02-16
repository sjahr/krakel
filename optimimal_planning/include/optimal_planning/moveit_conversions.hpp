#pragma once

#include <Eigen/Dense>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/robot_state/robot_state.hpp>

#include "b_spline.hpp"

namespace krakel
{
/** Transforms a path given std::vector<Eigen::VectorXd> to a robot_trajectory */
robot_trajectory::RobotTrajectory vectorToRobotTrajectory(const std::vector<Eigen::VectorXd>& trajectory,
                                                          const moveit::core::RobotModelPtr& robot_model,
                                                          const std::string& group);

/** Transform a bspline to a robot trajectory */
robot_trajectory::RobotTrajectory bSplineToRobotTrajectory(const BSpline& spline,
                                                           const moveit::core::RobotModelPtr& robot_model,
                                                           const std::string& group);
}  // namespace krakel
