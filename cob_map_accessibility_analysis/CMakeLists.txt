cmake_minimum_required(VERSION 2.8.3)
project(cob_map_accessibility_analysis)

find_package(catkin REQUIRED COMPONENTS
  #cob_3d_mapping_common
  cob_3d_mapping_msgs
  cv_bridge
  geometry_msgs
  image_transport
  message_filters
  message_generation
  nav_msgs
  pcl_ros
  roscpp
  sensor_msgs
  tf
)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)

catkin_python_setup()

# Generate services in the 'srv' folder
add_service_files(
  FILES
    CheckPerimeterAccessibility.srv
    CheckPointAccessibility.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    geometry_msgs
)

catkin_package()

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)


# map_accessibility_analysis_server
add_executable(map_accessibility_analysis_server ros/src/map_accessibility_analysis_server.cpp)
target_link_libraries(map_accessibility_analysis_server
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)
add_dependencies(map_accessibility_analysis_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# map_accessibility_analysis_client
add_executable(map_accessibility_analysis_client ros/src/map_accessibility_analysis_client.cpp)
target_link_libraries(map_accessibility_analysis_client
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)
add_dependencies(map_accessibility_analysis_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############
install(TARGETS map_accessibility_analysis_server map_accessibility_analysis_client
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY ros/launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros
)
