cmake_minimum_required(VERSION 2.8.3)
project(cob_navigation_global)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    amcl
    cob_default_env_config
    cob_linear_nav
    cob_navigation_config
    map_server
    move_base
    rviz
    topic_tools
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
# SET(envlist
# ipa-kitchen
# ipa-apartment
# )
#
# # loop through list of robots for testing
# FOREACH(robot ${robotlist})
# 	FOREACH(env ${envlist})
# 		MESSAGE("testing for robot: ${robot} in env: ${env}")
# 		roslaunch_add_file_check(launch ROBOT=${robot} ROBOT_ENV=${env})
# 		roslaunch_add_file_check(launch robot:=${robot} robot_env:=${env})
# 	ENDFOREACH(env)
# ENDFOREACH(robot)
