^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_linear_nav
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.14 (2023-01-04)
-------------------

0.6.13 (2022-07-29)
-------------------

0.6.12 (2021-05-10)
-------------------
* Merge pull request `#121 <https://github.com/ipa320/cob_navigation/issues/121>`_ from fmessmer/fix_catkin_lint
  fix catkin_lint
* fix catkin_lint
* Contributors: Felix Messmer, fmessmer

0.6.11 (2020-10-22)
-------------------
* Merge pull request `#120 <https://github.com/ipa320/cob_navigation/issues/120>`_ from fmessmer/test_noetic
  test noetic
* Bump CMake version to avoid CMP0048 warning
* Merge pull request `#119 <https://github.com/ipa320/cob_navigation/issues/119>`_ from AravindaDP/feat/cob_linear_nav-velocity_threshold_param
  feat: [cob_linear_nav] Goal abortion speed params
* feat: [cob_linear_nav] Goal abortion speed params
  Support adjusting linear and rotational velocity thresholds for goal
  abortion detection. (No movement due to obstacle)
* Merge pull request `#117 <https://github.com/ipa320/cob_navigation/issues/117>`_ from AravindaDP/feat/cob_linear_nav_footprint_tf_param
  feat: robot_footprint_frame rosparam
* feat: robot_footprint_frame rosparam
  Support setting robot_footprint_frame as a rosparam (issues/116)
  (defaults to "base_footprint")
* Contributors: Felix Messmer, Florian Weisshardt, Pramuditha Aravinda, fmessmer

0.6.10 (2020-03-18)
-------------------
* Merge pull request `#110 <https://github.com/ipa320/cob_navigation/issues/110>`_ from HannesBachter/fix/more_output
  more output for linear_nav
* more output for linear_nav
* Merge pull request `#111 <https://github.com/ipa320/cob_navigation/issues/111>`_ from fmessmer/ci_updates
  [travis] ci updates
* catkin_lint fixes
* Contributors: Felix Messmer, fmessmer, hyb

0.6.9 (2019-11-07)
------------------

0.6.8 (2019-08-07)
------------------

0.6.7 (2018-07-21)
------------------
* Merge pull request `#100 <https://github.com/ipa320/cob_navigation/issues/100>`_ from ipa-bnm/fix/simple_goal
  fixed goal Validation
* Merge pull request `#101 <https://github.com/ipa320/cob_navigation/issues/101>`_ from ipa-bnm/fix/angular_dist_calculation
  fixed shortest angular distance calculation
* fixed calculation of shortest angluar distance between current and desired yaw
* replaced canTransform with waitForTransform to give move_base some chance receiving the transformation needed for move base goal
* Contributors: Benjamin Maidel, Felix Messmer, Richard Bormann

0.6.6 (2018-01-07)
------------------
* Merge pull request `#97 <https://github.com/ipa320/cob_navigation/issues/97>`_ from ipa320/indigo_release_candidate
  Indigo release candidate
* Merge pull request `#95 <https://github.com/ipa320/cob_navigation/issues/95>`_ from ipa-fxm/less_hysteric_linear_nav
  less hysteric linear nav
* less hysteric linear nav
* Merge pull request `#91 <https://github.com/ipa320/cob_navigation/issues/91>`_ from ipa-fxm/update_maintainer
  update maintainer
* update maintainer
* Merge pull request `#89 <https://github.com/ipa320/cob_navigation/issues/89>`_ from ipa-fxm/APACHE_license
  use license apache 2.0
* use license apache 2.0
* Contributors: Felix Messmer, ipa-fez, ipa-fxm, ipa-uhr-mk

0.6.5 (2017-07-18)
------------------
* add c++11 definitions
* adjusted look up time stamps and durations of wait for transform
* added server to set base_frame, included some wait for transfrom and error handling
* Contributors: ipa-fxm, ipa-srd

0.6.4 (2016-04-01)
------------------

0.6.3 (2015-08-31)
------------------
* remove trailing whitespace
* migration to package format v2, indentation fixes
* Contributors: ipa-mig

0.6.2 (2015-06-17)
------------------

0.6.1 (2014-09-18)
------------------

0.6.0 (2014-09-10)
------------------

0.5.2 (2014-08-28)
------------------
* linear_nav: changed ROS_INFO to ROS_DEBUG in topicCB
* cob_linear_nav: fix indentation
* cob_linear_nav: initialize some more variables, check if received goal has invalid Quaternion
* cob_linear_nav: change order of nan and high velocity check to catch them with correct error msg
* cob_linear_nav: now hopefully really found the bug; ros::Time::now()-1 = verybig
* cob_linear_nav: change assert to try-catch to check that in all modes
* cob_linear_nav: initialize variable and add assertions
* Merge remote-tracking branch 'origin-ipa-aub/hydro_dev' into feature/baer_error_control
* included the option for linear_nav to perform the controller step frequently and update the goal with each topic callback instead of using actions
* Changed frame_ids to non slashed versions. TF should now work with hydro.
* add extra install tags to be consistent with rest of repos
* Contributors: Alexander Bubeck, Florian Mirus, ipa-mig

0.5.1 (2014-03-21)
------------------
* add changelog
* version bump
* remove more not needed files
* adjust CMakeLists and package.xml
* set author/maintainer email
* Catkinisation and gitignore.
* fix wiki links inf manifest.xml and stack.xml
* transform goal from rviz to /map frame
* remove unused publisher
* rearrange stack
* rename cob_linear_nav to cob_navigation_linear, rearrange again
* change default parameter
* tidy up
* change parameters
* remove collision avoidance from cob_linear_nav, now handled by cob_collision_velocity_filter
* add launch test
* update stack
* move linear_nav to cob_navigation
* Contributors: Florian Weisshardt, IPR-SR2, ipa-fmw, ipa-mig, nhg-ipa

* Catkinisation and gitignore.
* fix wiki links inf manifest.xml and stack.xml
* transform goal from rviz to /map frame
* remove unused publisher
* rearrange stack
* rename cob_linear_nav to cob_navigation_linear, rearrange again
* change default parameter
* tidy up
* change parameters
* remove collision avoidance from cob_linear_nav, now handled by cob_collision_velocity_filter
* add launch test
* update stack
* move linear_nav to cob_navigation
* Contributors: IPR-SR2, ipa-fmw, ipa-mig, nhg-ipa
