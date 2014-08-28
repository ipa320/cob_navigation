cmake_minimum_required(VERSION 2.8.3)
project(cob_mapping_slam)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    gmapping
    cob_navigation_global
)


#############
## Install ##
#############

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})