cmake_minimum_required(VERSION 2.8.3)
project(cob_linear_nav)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  move_base_msgs
  nav_msgs
  actionlib
  tf
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    move_base_msgs
    nav_msgs
    actionlib
    tf
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(cob_linear_nav src/cob_linear_nav.cpp)
target_link_libraries(cob_linear_nav ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS cob_linear_nav
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)