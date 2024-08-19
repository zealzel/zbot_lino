cmake_minimum_required(VERSION 3.8)
project(linorobot2_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED) # Add this line
find_package(sensor_msgs REQUIRED) # Add this line
find_package(apriltag_msgs REQUIRED) # Add this line
find_package(apriltag_ros REQUIRED) # Add this line

add_executable(repub_node src/repub_node.cpp)
ament_target_dependencies(repub_node rclcpp geometry_msgs sensor_msgs)

add_executable(detect_pose src/detect_pose.cpp)
ament_target_dependencies(detect_pose rclcpp geometry_msgs apriltag_msgs)

install(DIRECTORY launch rviz config maps params masks
        DESTINATION share/${PROJECT_NAME}
)
install(TARGETS repub_node detect_pose DESTINATION lib/${PROJECT_NAME})

ament_package()
