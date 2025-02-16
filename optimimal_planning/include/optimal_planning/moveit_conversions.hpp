#pragma once

#include <Eigen/Dense>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/robot_state/robot_state.hpp>

namespace krakel
{
/** Transforms a path given std::vector<Eigen::VectorXd> to a robot_trajectory */
robot_trajectory::RobotTrajectory vectorToRobotTrajectory(const std::vector<Eigen::VectorXd>& trajectory,
                                                          const moveit::core::RobotModelPtr& robot_model,
                                                          const std::string& group);
}  // namespace krakel
