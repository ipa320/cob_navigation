cmake_minimum_required(VERSION 2.8.3)
project(cob_mapping_slam)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    gmapping
    cob_navigation_config
    cob_unified_scan_publisher
)


#############
## Install ##
#############

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})