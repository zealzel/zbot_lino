cmake_minimum_required(VERSION 3.5)
project(linorobot2_navigation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)  # Add this line
find_package(sensor_msgs REQUIRED)    # Add this line

add_executable(repub_node src/repub_node.cpp)
ament_target_dependencies(repub_node rclcpp geometry_msgs sensor_msgs)

install(
  DIRECTORY launch rviz config maps params masks
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  repub_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
