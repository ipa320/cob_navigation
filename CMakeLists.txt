cmake_minimum_required(VERSION 2.8.3)
project(cob_map_accessibility_analysis)

set(catkin_RUN_PACKAGES
	roscpp
	pcl_ros
	tf
	image_transport
	cv_bridge
	message_filters
	sensor_msgs
	nav_msgs
	geometry_msgs
	cob_3d_mapping_msgs
	cob_3d_mapping_common
)

set(catkin_BUILD_PACKAGES
	${catkin_RUN_PACKAGES}
	cmake_modules
	message_generation
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
## Creates a bunch of environment variables that may be used later in the script
## e.g. catkin_INCLUDE_DIRS
find_package(catkin REQUIRED COMPONENTS
	${catkin_BUILD_PACKAGES}
)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)


################################################
## Declare ROS messages, services and actions ##
################################################
## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
## your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
## * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
## * If MSG_DEP_SET isn't empty the following dependencies might have been
## pulled in transitively but can be declared for certainty nonetheless:
## * add a build_depend tag for "message_generation"
## * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
## * add "message_generation" and every package in MSG_DEP_SET to
## find_package(catkin REQUIRED COMPONENTS ...)
## * add "message_runtime" and every package in MSG_DEP_SET to
## catkin_package(CATKIN_DEPENDS ...)
## * uncomment the add_*_files sections below as needed
## and list every .msg/.srv/.action file to be processed
## * uncomment the generate_messages entry below
## * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
#add_message_files(
#	FILES
#		Dirt.msg
#)

# Generate services in the 'srv' folder
add_service_files(
	FILES
		CheckPerimeterAccessibility.srv
		CheckPointAccessibility.srv
)

## Generate actions in the 'action' folder
#add_action_files(
# DIRECTORY
# action
# FILES
# SetJointTarget.action
# TrackNewSetpoints.action
#)

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
		geometry_msgs
)

# dynamic reconfigure
#generate_dynamic_reconfigure_options(
#	cfg/DirtDetection.cfg
#)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	INCLUDE_DIRS
		ros/include
	LIBRARIES
	CATKIN_DEPENDS
		${catkin_RUN_PACKAGES}
		message_runtime
	DEPENDS
		OpenCV
		Boost
)

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
include_directories(
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)


# map_accessibility_analysis_server
add_executable(map_accessibility_analysis_server ros/src/map_accessibility_analysis_server.cpp)
target_link_libraries(map_accessibility_analysis_server
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	${Boost_LIBRARIES}
)
add_dependencies(map_accessibility_analysis_server ${catkin_EXPORTED_TARGETS})


# map_accessibility_analysis_client
add_executable(map_accessibility_analysis_client ros/src/map_accessibility_analysis_client.cpp)
target_link_libraries(map_accessibility_analysis_client
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	${Boost_LIBRARIES}
)
add_dependencies(map_accessibility_analysis_client ${catkin_EXPORTED_TARGETS})



#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS map_accessibility_analysis_server map_accessibility_analysis_client
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
	PATTERN ".svn" EXCLUDE
)

install(DIRECTORY ros/launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros
	PATTERN ".svn" EXCLUDE
)

#install(DIRECTORY common/files
#	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/common
#	PATTERN ".svn" EXCLUDE
#)
