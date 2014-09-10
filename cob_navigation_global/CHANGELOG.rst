^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_navigation_global
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.0 (2014-09-10)
------------------
* Merge branch 'indigo_dev' into indigo_release_candidate
* fix dependencies
* Contributors: Florian Weisshardt

0.5.2 (2014-08-28)
------------------
* cob_navigation_global: add launch file argument to scan_unifier launch file for changing location of param file
* renamed ipa_navigation_scan_uniffier to cob_scan_unifier
* updated scan_unifier to parse scan_topics for subscription from parameter-server. added scan_unifier config-files for each robot. modified launch files for navigation and amcl to use scan_unifier
* set minimal linear velocity in local_planner_eband.yaml to 0 to allow in place rotation
* Update package.xml
* Contributors: Florian Mirus, Florian Weisshardt, ipa-mig

0.5.1 (2014-03-21)
------------------
* add changelog
* version bump
* adjust CMakeLists and package.xml
* set author/maintainer email
* remove leftover files from catkin migration
* change config to groovy
* Catkinisation and gitignore.
* some config changes
* added config files for eband
* added launch files for eband planner
* update tests
* remove dep to cob_driver
* add roslaunch tests
* remove publishing of voxel-grid
* Merge branch 'master', remote branch 'origin-ipa-mig-hm/master'
* integrate changes of cob_navigation into new setup
* Merge branch 'master' of github.com:ipa320/cob_navigation
* fix navigation for cob
* adjust launch files for planner config files and add launch arg to linear nav
* introduce launch args
* move ipa launch files to seperate folder
* adjust roslaunch checks
* added rviz launch files and config files
* using voxel costmap for ros navigation
* optimized parameters for platform movements, lowered control frequency
* optimized parameters for platform movements, lowered control frequency
* fix wiki links inf manifest.xml and stack.xml
* rename safety topic to safe
* update on xml-files for correct remapping and to delete old parameters
* update of the manifest-files in cob_navigation_global/cob_navigation_local, cob_vel_integrator -> cob_base_velocity_smoother
* removal of cob_base_velocity_smoother, moved to stack cob_driver
* namechanges from cob_vel_integrator to cob_base_velocity_smoother
* adjust manifests for documentation
* update deps
* fix roslaunch checks
* integrate cob_vel_integrator yaml and fix wrong inclusions
* merge branch raw3-1 into master
* fix roslaunch tests
* refactoring of cob_navigation_global
* refactoring of cob_navigation_config; current cob_drivers required
* modifications for raw3-1
* modifications for raw3-1
* fix roslaunch checks
* rearrange stack
* fix dependencies and launch files
* fix wrong inclusions
* rename cob_linear_nav to cob_navigation_linear, rearrange again
* fix include mistakes
* tidy up and rearrange
* Contributors: Alexander Bubeck, Florian Mirus, Florian Weisshardt, Florian Weißhardt, IPR-SR2, abubeck, ipa-fmw, ipa-frm, ipa-mig, ipa-mig-hm

* Catkinisation and gitignore.
* some config changes
* added config files for eband
* added launch files for eband planner
* update tests
* remove dep to cob_driver
* add roslaunch tests
* remove publishing of voxel-grid
* Merge branch 'master', remote branch 'origin-ipa-mig-hm/master'
* integrate changes of cob_navigation into new setup
* Merge branch 'master' of github.com:ipa320/cob_navigation
* fix navigation for cob
* adjust launch files for planner config files and add launch arg to linear nav
* introduce launch args
* move ipa launch files to seperate folder
* adjust roslaunch checks
* added rviz launch files and config files
* using voxel costmap for ros navigation
* optimized parameters for platform movements, lowered control frequency
* optimized parameters for platform movements, lowered control frequency
* fix wiki links inf manifest.xml and stack.xml
* rename safety topic to safe
* update on xml-files for correct remapping and to delete old parameters
* update of the manifest-files in cob_navigation_global/cob_navigation_local, cob_vel_integrator -> cob_base_velocity_smoother
* removal of cob_base_velocity_smoother, moved to stack cob_driver
* namechanges from cob_vel_integrator to cob_base_velocity_smoother
* adjust manifests for documentation
* update deps
* fix roslaunch checks
* integrate cob_vel_integrator yaml and fix wrong inclusions
* merge branch raw3-1 into master
* fix roslaunch tests
* refactoring of cob_navigation_global
* refactoring of cob_navigation_config; current cob_drivers required
* modifications for raw3-1
* modifications for raw3-1
* fix roslaunch checks
* rearrange stack
* fix dependencies and launch files
* fix wrong inclusions
* rename cob_linear_nav to cob_navigation_linear, rearrange again
* fix include mistakes
* tidy up and rearrange
* Contributors: Alexander Bubeck, Florian Mirus, Florian Weißhardt, IPR-SR2, abubeck, ipa-fmw, ipa-frm, ipa-mig, ipa-mig-hm
