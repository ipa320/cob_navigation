cmake_minimum_required(VERSION 2.8.3)
project(cob_navigation_config)

find_package(catkin REQUIRED)

catkin_package()

set(ROBOTLIST
  cob4-1
  cob4-2
  raw3-1
  raw3-3
  raw3-5
)

#############
## Install ##
#############

foreach(robot ${ROBOTLIST})
  install(DIRECTORY ${robot}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
endforeach()
