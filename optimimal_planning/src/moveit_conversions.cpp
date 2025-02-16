#include "optimal_planning/moveit_conversions.hpp"

namespace krakel
{
robot_trajectory::RobotTrajectory vectorToRobotTrajectory(const std::vector<Eigen::VectorXd>& trajectory,
                                                          const moveit::core::RobotModelPtr& robot_model,
                                                          const std::string& group)
{
  moveit::core::RobotState robot_state(robot_model);
  robot_state.setToDefaultValues();
  robot_trajectory::RobotTrajectory robot_trajectory(robot_model, group);

  for (const Eigen::VectorXd& joint_values : trajectory)
  {
    robot_state.setJointGroupPositions(group, joint_values);
    robot_trajectory.addSuffixWayPoint(robot_state, 0.1);
  }

  return robot_trajectory;
}

robot_trajectory::RobotTrajectory bSplineToRobotTrajectory(const BSpline& spline,
                                                           const moveit::core::RobotModelPtr& robot_model,
                                                           const std::string& group)
{
  moveit::core::RobotState robot_state(robot_model);
  robot_state.setToDefaultValues();
  robot_trajectory::RobotTrajectory robot_trajectory(robot_model, group);

  const double num_steps = 100;
  for (int i = 0; i < num_steps; ++i)
  {
    double t = i / num_steps;
    Eigen::VectorXd joint_values = deBoor(spline, t);
    robot_state.setJointGroupPositions(group, joint_values);
    robot_trajectory.addSuffixWayPoint(robot_state, 0.1);
  }
  return robot_trajectory;
}
}  // namespace krakel
