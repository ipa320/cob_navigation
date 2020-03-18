^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_navigation_global
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.10 (2020-03-18)
-------------------
* Merge pull request `#115 <https://github.com/ipa320/cob_navigation/issues/115>`_ from fmessmer/speedup_roslaunch_checks
  speed-up roslaunch checks
* speed-up roslaunch checks
* Merge pull request `#112 <https://github.com/ipa320/cob_navigation/issues/112>`_ from fmessmer/fix_deprecation_warning
  remove pre-hydro parameter static_map
* remove pre-hydro parameter static_map
* Merge pull request `#111 <https://github.com/ipa320/cob_navigation/issues/111>`_ from fmessmer/ci_updates
  [travis] ci updates
* catkin_lint fixes
* Contributors: Felix Messmer, fmessmer

0.6.9 (2019-11-07)
------------------
* Merge pull request `#107 <https://github.com/ipa320/cob_navigation/issues/107>`_ from floweisshardt/fix/roslaunch_tests
  fix roslaunch checks for all cob_supported_robots
* fix roslaunch checks for all cob_supported_robots
* Contributors: Felix Messmer, floweisshardt

0.6.8 (2019-08-07)
------------------

0.6.7 (2018-07-21)
------------------

0.6.6 (2018-01-07)
------------------
* Merge pull request `#97 <https://github.com/ipa320/cob_navigation/issues/97>`_ from ipa320/indigo_release_candidate
  Indigo release candidate
* Merge pull request `#91 <https://github.com/ipa320/cob_navigation/issues/91>`_ from ipa-fxm/update_maintainer
  update maintainer
* update maintainer
* Merge pull request `#89 <https://github.com/ipa320/cob_navigation/issues/89>`_ from ipa-fxm/APACHE_license
  use license apache 2.0
* use license apache 2.0
* Contributors: Felix Messmer, ipa-fez, ipa-fxm, ipa-uhr-mk

0.6.5 (2017-07-18)
------------------
* remove nav config for unsupported bases
* arg pkg_nav_config
* refactoring env config
* restructure nav config
* use exported robotlist and envlist
* remove dependency to eband_local_planner
* remove cob4-1
* remove obsolete configs due to unsupported robots
* remove relays
* cob4-5 setup
* enable roslaunch checks (requires ros_comm `#814 <https://github.com/ipa320/cob_navigation/issues/814>`_ to be merged and released)
* add dependency to dwa_local_planner (was not automatically installed using apt-get)
* adapt global rviz config
* put specific launch files in correct namespaces
* first adaption of costmap parameters
* remove eband planner launch and config files
* Contributors: Felix Messmer, Florian Weisshardt, ipa-cob4-5, ipa-fxm, ipa-mig

0.6.4 (2016-04-01)
------------------
* remove slashes from frame_ids
* restructure laser topics
* adapt twist_mux topic names according to https://github.com/ipa320/orga/pull/1#issuecomment-159195427
* moved collision_velocity_filter to base namespace
* changed 2dnav_linear twist topic to twist_mux input topic
* changed base command topic names to twist_mux topic input
* removed scan_unifier from cob_navigation
* Remap to command_safe for linear nav
  see discussion in https://github.com/ipa320/cob_navigation/pull/54
* removed global prefix
* use base  new name spaces
* Contributors: Benjamin Maidel, Florian Weisshardt, ipa-fmw, ipa-fxm, ipa-mig, ipa-nhg

0.6.3 (2015-08-31)
------------------
* remove trailing whitespace
* migration to package format v2, indentation fixes
* remove deprecated launchfiles
* Contributors: ipa-mig

0.6.2 (2015-06-17)
------------------

0.6.1 (2014-09-18)
------------------

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
