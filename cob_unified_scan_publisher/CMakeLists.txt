cmake_minimum_required(VERSION 2.8.3)
project(cob_unified_scan_publisher)

find_package(catkin REQUIRED COMPONENTS
  laser_geometry
  message_filters
  roscpp
  sensor_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS
    laser_geometry
    message_filters
    roscpp
    sensor_msgs
    tf
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(cob_unified_scan_publisher ros/src/cob_unified_scan_publisher.cpp)
target_link_libraries(cob_unified_scan_publisher ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS cob_unified_scan_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)