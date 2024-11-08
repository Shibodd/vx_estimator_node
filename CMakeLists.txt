cmake_minimum_required(VERSION 3.8)
project(estimator_node)

# find dependencies
find_package (ament_cmake REQUIRED)
find_package (rclcpp REQUIRED)
find_package (vesc_msgs REQUIRED)
find_package (Eigen3 REQUIRED)

add_executable(estimator_node src/estimator_node.cpp)
target_include_directories(estimator_node PUBLIC ./include)
target_link_libraries (estimator_node Eigen3::Eigen)
ament_target_dependencies (estimator_node rclcpp vesc_msgs)


install(TARGETS estimator_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
