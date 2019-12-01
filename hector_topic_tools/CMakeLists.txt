cmake_minimum_required(VERSION 2.8.3)
project(hector_topic_tools)

find_package(catkin REQUIRED COMPONENTS
  roslib
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hector_topic_tools
  CATKIN_DEPENDS roslib rospy
#  DEPENDS system_lib
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS scripts/relay_repeater.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
