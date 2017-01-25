cmake_minimum_required(VERSION 2.8.3)
project(cob_navigation_local)

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

### TESTING ###
if(CATKIN_ENABLE_TESTING)
    # loop through list of robots for testing
    foreach(robot ${robotlist})
        MESSAGE("testing for robot: ${robot}")
        roslaunch_add_file_check(launch ROBOT=${robot})    # testing environment variables
        roslaunch_add_file_check(launch robot:=${robot})   # testing launch file arguments
    endforeach(robot)
endif()
