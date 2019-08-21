cmake_minimum_required(VERSION 2.8.3)
project(cob_navigation_global)

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
if(CATKIN_ENABLE_TESTING)
  find_package(cob_default_env_config REQUIRED)
  find_package(cob_supported_robots REQUIRED)
  find_package(roslaunch REQUIRED)
  foreach(robot ${cob_supported_robots_ROBOTLIST})
    foreach(env ${cob_default_env_config_ENVLIST})
      message("testing for robot: ${robot} in env: ${env}")
      roslaunch_add_file_check(launch ROBOT=${robot} ROBOT_ENV=${env})   # testing environment variables
      roslaunch_add_file_check(launch robot:=${robot} robot_env:=${env}) # testing launch file arguments
    endforeach()
  endforeach()
endif()
