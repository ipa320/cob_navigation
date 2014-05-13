cmake_minimum_required(VERSION 2.8.3)
project(cob_navigation_slam)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    cob_navigation_config
    cob_navigation_global
    cob_mapping_slam
    rviz
)


#############
## Install ##
#############

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# rostest for robots
# SET(robotlist
# cob3-1
# cob3-2
# cob3-3
# cob3-4
# cob3-5
# cob3-6
# raw3-1
# raw3-2
# raw3-3
# raw3-4
# desire
# )
#
# # loop through list of robots for testing
# FOREACH(robot ${robotlist})
#     MESSAGE("testing for robot: ${robot}")
#     roslaunch_add_file_check(launch ROBOT=${robot})
#     roslaunch_add_file_check(launch robot:=${robot})
# ENDFOREACH(robot)