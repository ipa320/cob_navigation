cmake_minimum_required(VERSION 3.0.2)
project(cob_linear_nav)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  angles
  cob_srvs
  geometry_msgs
  move_base_msgs
  nav_msgs
  roscpp
  tf
)

catkin_package(
  CATKIN_DEPENDS actionlib cob_srvs geometry_msgs move_base_msgs nav_msgs roscpp tf
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
