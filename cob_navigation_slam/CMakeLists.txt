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

# rostest for robots
# SET(robotlist
# raw3-1
# raw3-3
# )
#
# # loop through list of robots for testing
# FOREACH(robot ${robotlist})
#     MESSAGE("testing for robot: ${robot}")
#     roslaunch_add_file_check(launch ROBOT=${robot})
#     roslaunch_add_file_check(launch robot:=${robot})
# ENDFOREACH(robot)
