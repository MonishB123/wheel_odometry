cmake_minimum_required(VERSION 3.8)
project(wheel_odometry)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(wheel_odometry_node src/wheel_odometry_node.cpp)
ament_target_dependencies(wheel_odometry_node
  rclcpp nav_msgs geometry_msgs tf2 tf2_ros std_msgs)

install(TARGETS wheel_odometry_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})
  
install(PROGRAMS
  scripts/wheel_ticks_publisher.py
  scripts/encoder_node.py
  scripts/USART.py
  scripts/pose_printer.py
  DESTINATION lib/${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

