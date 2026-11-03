cmake_minimum_required(VERSION 3.8)
project(dofbot_two_joint_cycle)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(two_joint_cycle src/two_joint_cycle.cpp)
target_compile_features(two_joint_cycle PUBLIC cxx_std_17)
ament_target_dependencies(two_joint_cycle rclcpp builtin_interfaces trajectory_msgs)
target_link_libraries(two_joint_cycle moveit_ros_planning_interface::moveit_move_group_interface)

install(TARGETS two_joint_cycle DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
