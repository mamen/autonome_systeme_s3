cmake_minimum_required(VERSION 2.8.3)
project(custom_packages)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  rospy
)

################################################################################
# Setup for python modules and scripts
################################################################################
catkin_python_setup()

################################################################################
# Declare ROS messages, services and actions
################################################################################
#add_action_files(
#  FILES
#  Turtlebot3.action
#)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  geometry_msgs
#  actionlib_msgs
#)

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS
  rospy
)

################################################################################
# Build
################################################################################
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

################################################################################
# Install
################################################################################
catkin_install_python(PROGRAMS
    nodes/drive_straight.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch maps param rviz
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)