cmake_minimum_required(VERSION 2.8.3)
project(ros_real_robot_v2_bringup)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
)

catkin_package(
    CATKIN_DEPENDS roscpp std_msgs sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

