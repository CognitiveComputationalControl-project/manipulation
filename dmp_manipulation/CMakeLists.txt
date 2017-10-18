cmake_minimum_required(VERSION 2.8.3)
project(dmp_manipulation)	

SET(SOURCES src/dmp_manager.cpp)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  message_generation
  std_msgs
  std_srvs
  tf
  pcl_ros
  pcl_conversions
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

#add_service_files(
   #FILES
#)

generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
   std_srvs
)


include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include)
include_directories(src)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES humantracking
#  CATKIN_DEPENDS roscpp geometry_msgs
   CATKIN_DEPENDS
  	roscpp
  	geometry_msgs
  	sensor_msgs
    std_msgs
    message_runtime
    tf
 )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_executable(dmp_test src/dmp_test.cpp ${SOURCES})
target_link_libraries(dmp_test ${catkin_LIBRARIES} ${PCL_LIBRARIES})



install(TARGETS dmp_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)