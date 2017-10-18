cmake_minimum_required(VERSION 2.8.3)
project(manipulation_tutorials)

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

add_executable(arm_message src/arm_message.cpp)
add_executable(arm_trajectory src/arm_trajectory.cpp)
add_executable(arm_actionlib src/arm_actionlib.cpp)
add_executable(head_message src/head_message.cpp)
add_executable(head_actionlib src/head_actionlib.cpp)
add_executable(gripper_message src/gripper_message.cpp)
add_executable(gripper_actionlib src/gripper_actionlib.cpp)
add_executable(gripper_actionlib_grasp src/gripper_actionlib_grasp.cpp)
add_executable(omni_message src/omni_message.cpp)
add_executable(omni_message_velocity src/omni_message_velocity.cpp)
add_executable(omni_actionlib src/omni_actionlib.cpp)
add_executable(gazed_point_fixing_node  src/gazed_point_fixing_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(villa_manipulation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(arm_message
  ${catkin_LIBRARIES}
)
target_link_libraries(arm_trajectory
  ${catkin_LIBRARIES}
)

target_link_libraries(arm_actionlib
  ${catkin_LIBRARIES}
)
target_link_libraries(head_message
  ${catkin_LIBRARIES}
)
target_link_libraries(head_actionlib
  ${catkin_LIBRARIES}
)
target_link_libraries(gripper_message
  ${catkin_LIBRARIES}
)
target_link_libraries(gripper_actionlib
  ${catkin_LIBRARIES}
)
target_link_libraries(gripper_actionlib_grasp
  ${catkin_LIBRARIES}
)
target_link_libraries(omni_message
  ${catkin_LIBRARIES}
)
target_link_libraries(omni_message_velocity
  ${catkin_LIBRARIES}
)
target_link_libraries(omni_actionlib
  ${catkin_LIBRARIES}
)
target_link_libraries(gazed_point_fixing_node
  ${catkin_LIBRARIES}
)



#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS arm_message arm_trajectory arm_actionlib head_message head_actionlib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(DIRECTORY tests/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.py"
)
