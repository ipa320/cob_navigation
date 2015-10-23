cmake_minimum_required(VERSION 2.8.3)
project(cob_navigation_config)

find_package(catkin REQUIRED)

catkin_package()

set(ROBOTLIST
  cob3-2
  cob3-6
  cob4-1
  cob4-2
  raw3-1
  raw3-2
  raw3-3
  raw3-4
  raw3-5
  raw3-6
)

#############
## Install ##
#############

foreach(robot ${ROBOTLIST})
  install(DIRECTORY ${robot}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
endforeach()