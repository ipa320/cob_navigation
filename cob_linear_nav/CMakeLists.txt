cmake_minimum_required(VERSION 2.8.3)
project(cob_linear_nav)

find_package(catkin REQUIRED COMPONENTS
  angles
  actionlib
  cob_srvs
  geometry_msgs
  move_base_msgs
  nav_msgs
  roscpp
  tf
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_definitions(-std=c++11)

add_executable(${PROJECT_NAME} src/cob_linear_nav.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

install(TARGETS cob_linear_nav
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
