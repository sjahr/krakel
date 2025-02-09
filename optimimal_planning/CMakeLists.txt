cmake_minimum_required(VERSION 3.22)
project(optimal_planning LANGUAGES CXX)

find_package(ament_cmake REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pinocchio REQUIRED)

include_directories(include)

add_library(direct_collocation src/direct_collocation.cpp)
target_link_libraries(direct_collocation pinocchio::pinocchio)
ament_target_dependencies(direct_collocation rclcpp Eigen3)

install(
  TARGETS direct_collocation
  EXPORT direct_collocationTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include/optimal_planning)

ament_export_targets(direct_collocationTargets HAS_LIBRARY_TARGET)
ament_package()
