cmake_minimum_required(VERSION 2.8.3)
project(site_violation_detection)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge  
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(OpenCV)
#target_link_libraries(my_awesome_library ${OpenCV_LIBRARIES})
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES site_violation_detection
#  CATKIN_DEPENDS cv_bridge opencv2 roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

