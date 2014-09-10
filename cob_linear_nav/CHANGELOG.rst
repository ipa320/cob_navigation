^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_linear_nav
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
