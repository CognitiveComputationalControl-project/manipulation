cmake_minimum_required(VERSION 2.8.3)
project(villa_manipulation)

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
  sensor_msgs
  hsr_kinematics
  tmc_eigen_bridge
  tmc_manipulation_types_bridge
  tmc_utils
  actionlib_msgs
  actionlib
)


catkin_python_setup()


## Generate services in the 'srv' folder
add_service_files(
   FILES
   Pickup.srv
   BroadcastObjectFrames.srv
 )
add_action_files(DIRECTORY action FILES OpenDoor.action)
## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   actionlib_msgs
 )

find_package(catkin REQUIRED genmsg actionlib_msgs actionlib)



catkin_package(
  CATKIN_DEPENDS
  actionlib
  roscpp
  rospy
  trajectory_msgs
  control_msgs
  geometry_msgs
  sensor_msgs
  hsr_kinematics
  tmc_eigen_bridge
  tmc_manipulation_types_bridge
  tmc_utils
)

install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)


