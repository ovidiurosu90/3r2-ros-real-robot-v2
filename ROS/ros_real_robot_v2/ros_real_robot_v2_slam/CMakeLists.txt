cmake_minimum_required(VERSION 2.8.3)
project(ros_real_robot_v2_slam)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
)

catkin_package(
    CATKIN_DEPENDS roscpp sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

