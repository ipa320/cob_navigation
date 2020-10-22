cmake_minimum_required(VERSION 3.0.2)
project(cob_mapping_slam)

find_package(catkin REQUIRED)

catkin_package()

#############
## Install ##
#############

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

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
