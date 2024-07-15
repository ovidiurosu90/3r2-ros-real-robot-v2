cmake_minimum_required(VERSION 3.0.2)
project(ros_real_robot_v2_utils)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    roscpp
    rospy
    std_msgs
)

catkin_package(
#    INCLUDE_DIRS include
#    LIBRARIES ros_real_robot_v2_utils
#    CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
#    DEPENDS system_lib
)

include_directories(
# include
    ${catkin_INCLUDE_DIRS}
)

