cmake_minimum_required(VERSION 3.0.2)
project(pid_controller)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS src/pid_controller.py src/motion_planner.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
