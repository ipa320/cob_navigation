cmake_minimum_required(VERSION 2.8.3)
project(cob_navigation_slam)

find_package(catkin REQUIRED)

catkin_package()

#############
## Install ##
#############

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
SET(robotlist
    cob4-2
    cob4-5
    cob4-7
    raw3-1
    raw3-3
    raw3-5
)
    
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  foreach(robot ${robotlist})
    message("testing for robot: ${robot}")
    roslaunch_add_file_check(launch ROBOT=${robot})    # testing environment variables
    roslaunch_add_file_check(launch robot:=${robot})   # testing launch file arguments
  endforeach()
endif()
