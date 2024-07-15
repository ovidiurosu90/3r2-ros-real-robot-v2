cmake_minimum_required(VERSION 2.8.3)
project(ros_real_robot_v2_navigation)
find_package(catkin REQUIRED)
catkin_package()
install(DIRECTORY launch maps param rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

