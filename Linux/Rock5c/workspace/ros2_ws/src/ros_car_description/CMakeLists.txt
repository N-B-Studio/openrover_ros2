cmake_minimum_required(VERSION 3.8)

project(ros_car_description)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(robot_state_publisher REQUIRED)

install(
  DIRECTORY
    launch
    urdf
    rviz
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()