cmake_minimum_required(VERSION 3.22)
project(optimal_planning LANGUAGES CXX)

find_package(ament_cmake REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pinocchio REQUIRED)

include_directories(include)

add_library(optimal_planning src/direct_collocation.cpp
                             src/b_spline_parameterization.cpp)
target_link_libraries(optimal_planning pinocchio::pinocchio)
ament_target_dependencies(optimal_planning rclcpp Eigen3)

install(DIRECTORY include/ DESTINATION include/optimal_planning)

install(
  TARGETS optimal_planning
  EXPORT optimal_planningTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include/optimal_planning)

ament_export_targets(optimal_planningTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(pinocchio Eigen3 rclcpp)
ament_package()
