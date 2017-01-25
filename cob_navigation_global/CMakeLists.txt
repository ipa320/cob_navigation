cmake_minimum_required(VERSION 2.8.3)
project(cob_navigation_global)

find_package(catkin REQUIRED COMPONENTS
    roslaunch
)

catkin_package()

#############
## Install ##
#############

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# rostest for robots
SET(robotlist
    cob4-2
#    cob4-3
#    cob4-4
#    cob4-6
    raw3-1
    raw3-3
    raw3-5
)

SET(envlist
    ipa-kitchen
    ipa-apartment
)

### TESTING ###
if(CATKIN_ENABLE_TESTING)
    # loop through list of robots for testing
    foreach(robot ${robotlist})
        foreach(env ${envlist})
            MESSAGE("testing for robot: ${robot} in env: ${env}")
            roslaunch_add_file_check(launch ROBOT=${robot} ROBOT_ENV=${env})     # testing environment variables
            roslaunch_add_file_check(launch robot:=${robot} robot_env:=${env})   # testing launch file arguments
        endforeach(env)
    endforeach(robot)
endif()
