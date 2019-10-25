cob_navigation
===========

## ROS Distro Support

|         | Indigo | Jade | Kinetic |
|:-------:|:------:|:----:|:-------:|
| Branch  | [`indigo_dev`](https://github.com/ipa320/cob_navigation/tree/indigo_dev) | [`indigo_dev`](https://github.com/ipa320/cob_navigation/tree/indigo_dev) | [`indigo_dev`](https://github.com/ipa320/cob_navigation/tree/indigo_dev) |
| Status  |  supported | not supported |  supported |
| Version | [version](http://repositories.ros.org/status_page/ros_indigo_default.html?q=cob_navigation) | [version](http://repositories.ros.org/status_page/ros_jade_default.html?q=cob_navigation) | [version](http://repositories.ros.org/status_page/ros_kinetic_default.html?q=cob_navigation) |

## Travis - Continuous Integration

Status: [![Build Status](https://travis-ci.com/ipa320/cob_navigation.svg?branch=indigo_dev)](https://travis-ci.com/ipa320/cob_navigation)

## ROS Buildfarm

|         | Indigo Source | Indigo Debian | Jade Source | Jade Debian |  Kinetic Source  |  Kinetic Debian |
|:-------:|:-------------------:|:-------------------:|:-------------------:|:-------------------:|:-------------------:|:-------------------:|
| cob_navigation | [![not released](http://build.ros.org/buildStatus/icon?job=Isrc_uT__cob_navigation__ubuntu_trusty__source)](http://build.ros.org/view/Isrc_uT/job/Isrc_uT__cob_navigation__ubuntu_trusty__source/) | [![not released](http://build.ros.org/buildStatus/icon?job=Ibin_uT64__cob_navigation__ubuntu_trusty_amd64__binary)](http://build.ros.org/view/Ibin_uT64/job/Ibin_uT64__cob_navigation__ubuntu_trusty_amd64__binary/) | [![not released](http://build.ros.org/buildStatus/icon?job=Jsrc_uT__cob_navigation__ubuntu_trusty__source)](http://build.ros.org/view/Jsrc_uT/job/Jsrc_uT__cob_navigation__ubuntu_trusty__source/) | [![not released](http://build.ros.org/buildStatus/icon?job=Jbin_uT64__cob_navigation__ubuntu_trusty_amd64__binary)](http://build.ros.org/view/Jbin_uT64/job/Jbin_uT64__cob_navigation__ubuntu_trusty_amd64__binary/) | [![not released](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__cob_navigation__ubuntu_xenial__source)](http://build.ros.org/view/Ksrc_uX/job/Ksrc_uX__cob_navigation__ubuntu_xenial__source/) | [![not released](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__cob_navigation__ubuntu_xenial_amd64__binary)](http://build.ros.org/view/Kbin_uX64/job/Kbin_uX64__cob_navigation__ubuntu_xenial_amd64__binary/) |


The cob_navigation repository provides configuration and launch files for running the navigation stack on the Fraunhofer IPA robots Care-O-bot and rob@work in a number of common configurations.
For example, the `cob_navigation_local` package holds files that configure the `move_base` node to operate in an odometric frame, and the ’cob_mapping_slam’ package holds files that configure the robot for SLAM.
These configuration files are intended for use as building blocks for applications that wish to use autonomous navigation as a component.

The naming of the packages as well as the documentation is kept close to the one of `pr2_navigation` for ease of use for experienced ROS users. However, changes are made to suit our needs.

For more information, check out the [ROS Wiki page](http://wiki.ros.org/cob_navigation "cob_navigation wiki page")
