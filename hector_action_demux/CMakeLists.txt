cmake_minimum_required(VERSION 3.0.2)
project(hector_action_demux)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED roscpp topic_tools)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hector_action_demux
  CATKIN_DEPENDS roscpp topic_tools
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  include/hector_action_demux/action_client.h
  include/hector_action_demux/action_demuxer.h
  src/action_demuxer.cpp
  src/action_client.cpp
  include/hector_action_demux/common.h src/common.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## Declare a C++ executable
add_executable(action_demuxer_node src/action_demuxer_node.cpp)
add_dependencies(action_demuxer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(action_demuxer_node ${PROJECT_NAME})

#############
## Install ##
#############

#install(DIRECTORY
#  config
#  cfg
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#  )

install(TARGETS ${PROJECT_NAME} action_demuxer_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
