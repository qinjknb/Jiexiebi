cmake_minimum_required(VERSION 3.8)
project(dofbot_cycle_demo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(cycle_demo src/cycle_demo.cpp)
target_compile_features(cycle_demo PUBLIC cxx_std_17)
ament_target_dependencies(cycle_demo rclcpp)
target_link_libraries(cycle_demo moveit_ros_planning_interface::moveit_move_group_interface)

install(TARGETS cycle_demo DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
