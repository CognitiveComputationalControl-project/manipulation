cmake_minimum_required(VERSION 2.8.3)
project(manipulation_functions)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  roscpp
  rospy
  trajectory_msgs
  control_msgs
  message_generation
  geometry_msgs
  hsr_kinematics
  sensor_msgs
  tmc_eigen_bridge
  tmc_manipulation_types_bridge
  tmc_utils
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES villa_manipulation
#  CATKIN_DEPENDS actionlib roscpp rospy trajectory_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS
  actionlib
  roscpp
  rospy
  trajectory_msgs
  control_msgs
  geometry_msgs
  hsr_kinematics
  sensor_msgs
  tmc_eigen_bridge
  tmc_manipulation_types_bridge
  tmc_utils
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(villa_manipulation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against



#############
## Install ##
#############

## Mark executables and/or libraries for installation

install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(DIRECTORY tests/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.py"
)
