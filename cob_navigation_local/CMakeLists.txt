cmake_minimum_required(VERSION 2.8.3)
project(cob_navigation_local)

find_package(catkin REQUIRED)

catkin_package()

#############
## Install ##
#############

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(cob_supported_robots REQUIRED)
  find_package(roslaunch REQUIRED)
  foreach(robot ${cob_supported_robots_ROBOTLIST})
    message("testing for robot: ${robot}")
    roslaunch_add_file_check(launch ROBOT=${robot})   # testing environment variables
    roslaunch_add_file_check(launch robot:=${robot})  # testing launch file arguments
  endforeach()
endif()
