cmake_minimum_required(VERSION 2.8.3)
project(cob_navigation_config)

find_package(catkin REQUIRED)

catkin_package()


#############
## Install ##
#############

install(DIRECTORY cob3-1 cob3-2 cob3-3 cob3-4 cob3-5 cob3-6
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY cob4-1
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY desire
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY raw3-1 raw3-2 raw3-3 raw3-4 raw3-5 raw3-6
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
