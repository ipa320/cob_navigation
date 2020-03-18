^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_navigation_config
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.10 (2020-03-18)
-------------------
* Merge pull request `#114 <https://github.com/ipa320/cob_navigation/issues/114>`_ from HannesBachter/add_cob4-23
  add cob4-23
* add cob4-23
* Contributors: Felix Messmer, hyb

0.6.9 (2019-11-07)
------------------
* Merge pull request `#108 <https://github.com/ipa320/cob_navigation/issues/108>`_ from HannesBachter/add_cob4-24
  add cob4-24
* add cob4-24
* Merge pull request `#107 <https://github.com/ipa320/cob_navigation/issues/107>`_ from floweisshardt/fix/roslaunch_tests
  fix roslaunch checks for all cob_supported_robots
* fix roslaunch checks for all cob_supported_robots
* Contributors: Felix Messmer, Florian Weisshardt, floweisshardt, hyb

0.6.8 (2019-08-07)
------------------

0.6.7 (2018-07-21)
------------------
* Merge pull request `#105 <https://github.com/ipa320/cob_navigation/issues/105>`_ from floweisshardt/feature/add_cob4-13_and_cob4-16
  add cob4-13 and cob4-16
* add cob4-13 and cob4-16
* Contributors: Florian Weisshardt, ipa-fmw

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
* refactoring env config
* restructure nav config
* remove cob4-10
* add configs for missing supported robots
* remove cob4-1
* remove obsolete configs due to unsupported robots
* nav config for raw3-6
* use new costmap param layout
* cob4-5 setup
* Adapted config file for cob3-2
* Adapted config file for cob3-6
* Added map types to changed configs
* Adapted config file for cob4-2
* Adapted config file for cob4-1
* Adapted config file for raw3-6
* Adapted config file for raw3-5
* Adapted config file for raw3-4
* Adapted config file for raw3-1
* Adapted costmap config file for raw3-3
* first adaption of costmap parameters
* Contributors: Benjamin Maidel, Florian Weisshardt, ipa-cob4-5, ipa-fxm, ipa-mig, mig-em, teddy

0.6.4 (2016-04-01)
------------------
* restructure laser topics
* removed scan_unifier from cob_navigation
* Contributors: Benjamin Maidel, ipa-fxm

0.6.3 (2015-08-31)
------------------
* cob_navigation_config: remove robots not supported anymore
* remove trailing whitespace
* migration to package format v2, indentation fixes
* Contributors: ipa-mig

0.6.2 (2015-06-17)
------------------

0.6.1 (2014-09-18)
------------------
* added cob4-2 configuration
* Contributors: ipa-cob4-2

0.6.0 (2014-09-10)
------------------

0.5.2 (2014-08-28)
------------------
* removed start_delay from scan-unifier configs and intendation-fix
* updated scan_unifier to parse scan_topics for subscription from parameter-server. added scan_unifier config-files for each robot. modified launch files for navigation and amcl to use scan_unifier
* added the new scan_unifier as well as a dummy config file and removed the old one
* added softkinetic cameras as voxel based observation sources
* Changed frame_ids to non slashed versions. TF should now work with hydro.
* Contributors: Alexander Bubeck, Florian Mirus, flg

0.5.1 (2014-03-21)
------------------
* cob_navigation: removing cob3-5b configs
* add changelog
* version bump
* remove more not needed files
* adjust CMakeLists and package.xml
* set author/maintainer email
* Update cob4-1 config
  due to @ipa-mig suggestion from https://github.com/ipa320/cob_navigation/pull/22
* Configuration for cob4-1
* change config to groovy
* add config for cob3-5b
* Catkinisation and gitignore.
* some config changes
* added navigation config for raw3-6
* modified footprint dimensions
* added raw3-3 and raw3-5 navigation config
* slower down robot
* add voxel costmap for all cob's
* integrate changes of cob_navigation into new setup
* fix navigation for cob
* allow dwa backwards movement for all cobs
* use dwa parameter for cob3-1 from janpaulus
* move planner config files from common to robot specific folder
* using voxel costmap for ros navigation
* Set clearing = true for Hokuyo sensor
* adjust footprint to fit emergency stop fields
* optimized parameters for platform movements, lowered control frequency
* add raw3-4 parameter
* optimized parameters for platform movements, lowered control frequency
* extend footpint g
* fix wiki links inf manifest.xml and stack.xml
* extend footprint
* hokuyo marking but not clearing
* extend footprint and use laser_top
* adjust footprint for desire
* extend footprint in x-direction
* removal of cob_base_velocity_smoother, moved to stack cob_driver
* namechanges from cob_vel_integrator to cob_base_velocity_smoother
* adjust manifests for documentation
* merge
* adapt max_vel_theta and footprint
* remove unavailable observation sources from raw3-2 costmap
* integrate cob_vel_integrator yaml and fix wrong inclusions
* merge branch raw3-1 into master
* default rviz config
* fix roslaunch tests
* added robot specific paramters
* removed old config file
* refactoring of cob_navigation_config; current cob_drivers required
* modifications for raw3-1
* rearrange stack
* rename cob_linear_nav to cob_navigation_linear, rearrange again
* tidy up and rearrange
* Contributors: Alexander Bubeck, Florian Weisshardt, Florian Weißhardt, IPR-SR2, abubeck, cob3-5, desire, ipa-bnm, ipa-cob3-5, ipa-fmw, ipa-frm, ipa-mig, ipa-mig-hm, robot, uh-klk

* Catkinisation and gitignore.
* some config changes
* added navigation config for raw3-6
* modified footprint dimensions
* added raw3-3 and raw3-5 navigation config
* slower down robot
* add voxel costmap for all cob's
* integrate changes of cob_navigation into new setup
* fix navigation for cob
* allow dwa backwards movement for all cobs
* use dwa parameter for cob3-1 from janpaulus
* move planner config files from common to robot specific folder
* using voxel costmap for ros navigation
* Set clearing = true for Hokuyo sensor
* adjust footprint to fit emergency stop fields
* optimized parameters for platform movements, lowered control frequency
* add raw3-4 parameter
* optimized parameters for platform movements, lowered control frequency
* extend footpint g
* fix wiki links inf manifest.xml and stack.xml
* extend footprint
* hokuyo marking but not clearing
* extend footprint and use laser_top
* adjust footprint for desire
* extend footprint in x-direction
* removal of cob_base_velocity_smoother, moved to stack cob_driver
* namechanges from cob_vel_integrator to cob_base_velocity_smoother
* adjust manifests for documentation
* merge
* adapt max_vel_theta and footprint
* remove unavailable observation sources from raw3-2 costmap
* integrate cob_vel_integrator yaml and fix wrong inclusions
* merge branch raw3-1 into master
* default rviz config
* fix roslaunch tests
* added robot specific paramters
* removed old config file
* refactoring of cob_navigation_config; current cob_drivers required
* modifications for raw3-1
* rearrange stack
* rename cob_linear_nav to cob_navigation_linear, rearrange again
* tidy up and rearrange
* Contributors: Alexander Bubeck, Florian Weißhardt, IPR-SR2, abubeck, cob3-5, desire, ipa-bnm, ipa-fmw, ipa-frm, ipa-mig, ipa-mig-hm, uh-klk
