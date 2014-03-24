cob_navigation
====================

The cob_navigation repository provides configuration and launch files for running the navigation stack on the Fraunhofer IPA robots Care-O-bot and rob@work in a number of common configurations.
For example, the `cob_navigation_local` package holds files that configure the `move_base` node to operate in an odometric frame, and the ’cob_mapping_slam’ package holds files that configure the robot for SLAM.
These configuration files are intended for use as building blocks for applications that wish to use autonomous navigation as a component.

The naming of the packages as well as the documentation is kept close to the one of `pr2_navigation` for ease of use for experienced ROS users. However, changes are made to suit our needs.

For more information, check out the [ROS Wiki page](http://wiki.ros.org/cob_navigation "cob_navigation wiki page")
