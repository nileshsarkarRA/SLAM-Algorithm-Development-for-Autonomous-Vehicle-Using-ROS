cmake_minimum_required(VERSION 3.5)
project(runs)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(motor_controller src/motor_controller.py)
ament_target_dependencies(motor_controller rclcpp std_msgs)

add_executable(lidar_publisher src/lidar_node.py)
ament_target_dependencies(lidar_node rclpy sensor_msgs)

add_executable(camera_node src/camera_node.py)
ament_target_dependencies(camera_node rclpy sensor_msgs cv_bridge OpenCV)

install(TARGETS
  motor_controller
  lidar_publisher
  camera_node
  DESTINATION lib/$runs)

install(DIRECTORY launch
  DESTINATION share/$runs/)

ament_package()
