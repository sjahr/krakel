cmake_minimum_required(VERSION 3.22)
project(optimal_planning LANGUAGES CXX)

find_package(ament_cmake REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pinocchio REQUIRED)

include_directories(include)

add_library(direct_collocation src/direct_collocation.cpp)
ament_target_dependencies(direct_collocation
    rclcpp
    Eigen3
    pinocchio)
target_link_libraries(direct_collocation PUBLIC pinocchio::pinocchio)

install(TARGETS constrained_planning_demo DESTINATION lib/krakel_demos)
install(DIRECTORY launch DESTINATION share/krakel_demos)
install(DIRECTORY config DESTINATION share/krakel_demos)

ament_package()
