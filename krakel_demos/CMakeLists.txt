cmake_minimum_required(VERSION 3.22)
project(krakel_demos LANGUAGES CXX)

find_package(ament_cmake REQUIRED)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(class_loader REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(warehouse_ros REQUIRED)
find_package(shape_msgs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    ament_cmake
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_visual_tools
    rclcpp
    rviz_visual_tools
    shape_msgs
    warehouse_ros)

add_executable(constrained_planning_demo src/constrained_planning_demo.cpp)
ament_target_dependencies(constrained_planning_demo
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(TARGETS constrained_planning_demo DESTINATION lib/krakel_demos)
install(DIRECTORY launch DESTINATION share/krakel_demos)
install(DIRECTORY config DESTINATION share/krakel_demos)

ament_package()
