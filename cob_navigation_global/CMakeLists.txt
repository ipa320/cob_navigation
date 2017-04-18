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
SET(robotlist
    cob4-2
    cob4-5
    cob4-7
    raw3-1
    raw3-3
    raw3-5
)
    
if(CATKIN_ENABLE_TESTING)
  find_package(cob_default_env_config REQUIRED)
  find_package(roslaunch REQUIRED)
  foreach(robot ${robotlist})
    foreach(env ${cob_default_env_config_ENVLIST})
      message("testing for robot: ${robot} in env: ${env}")
      roslaunch_add_file_check(launch ROBOT=${robot} ROBOT_ENV=${env})     # testing environment variables
      roslaunch_add_file_check(launch robot:=${robot} robot_env:=${env})   # testing launch file arguments
    endforeach(env)
  endforeach(robot)
endif()
