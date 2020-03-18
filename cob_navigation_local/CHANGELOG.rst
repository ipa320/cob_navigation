^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_navigation_local
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.10 (2020-03-18)
-------------------
* Merge pull request `#112 <https://github.com/ipa320/cob_navigation/issues/112>`_ from fmessmer/fix_deprecation_warning
  remove pre-hydro parameter static_map
* remove pre-hydro parameter static_map
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
* restructure nav config
* use exported robotlist and envlist
* remove cob4-1
* remove obsolete configs due to unsupported robots
* enable roslaunch checks (requires ros_comm `#814 <https://github.com/ipa320/cob_navigation/issues/814>`_ to be merged and released)
* add dependency to dwa_local_planner (was not automatically installed using apt-get)
* Added namespace to parameter loading
* Adapted config for cob_navigation_local
* adapt local rviz config
* Contributors: Florian Weisshardt, ipa-fxm, teddy

0.6.4 (2016-04-01)
------------------
* restructure laser topics
* adapt twist_mux topic names according to https://github.com/ipa320/orga/pull/1#issuecomment-159195427
* changed base command topic names to twist_mux topic input
* Contributors: Benjamin Maidel, ipa-fmw, ipa-fxm

0.6.3 (2015-08-31)
------------------
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
* Update package.xml
* Contributors: Florian Weisshardt

0.5.1 (2014-03-21)
------------------
* add changelog
* version bump
* remove more not needed files
* adjust CMakeLists and package.xml
* set author/maintainer email
* change config to groovy
* change to groovy rviz config
* Catkinisation and gitignore.
* remove dep to cob_driver
* add roslaunch tests
* adjust launch files for planner config files and add launch arg to linear nav
* introduce launch args
* add missing dependency
* added rviz launch files and config files
* add dwa as local navigation mode
* fix wiki links inf manifest.xml and stack.xml
* update on xml-files for correct remapping and to delete old parameters
* update of the manifest-files in cob_navigation_global/cob_navigation_local, cob_vel_integrator -> cob_base_velocity_smoother
* removal of cob_base_velocity_smoother, moved to stack cob_driver
* namechanges from cob_vel_integrator to cob_base_velocity_smoother
* adjust manifests for documentation
* update deps
* integrate cob_vel_integrator yaml and fix wrong inclusions
* merge branch raw3-1 into master
* fix roslaunch tests
* refactoring of cob_navigation_local
* refactoring of cob_navigation_config; current cob_drivers required
* modifications for raw3-1
* rearrange stack
* add missed file
* rename cob_linear_nav to cob_navigation_linear, rearrange again
* fix include mistakes
* tidy up and rearrange
* Contributors: Alexander Bubeck, Florian Mirus, Florian Weisshardt, Florian Weißhardt, IPR-SR2, ipa-fmw, ipa-frm, ipa-mig

* Catkinisation and gitignore.
* remove dep to cob_driver
* add roslaunch tests
* adjust launch files for planner config files and add launch arg to linear nav
* introduce launch args
* add missing dependency
* added rviz launch files and config files
* add dwa as local navigation mode
* fix wiki links inf manifest.xml and stack.xml
* update on xml-files for correct remapping and to delete old parameters
* update of the manifest-files in cob_navigation_global/cob_navigation_local, cob_vel_integrator -> cob_base_velocity_smoother
* removal of cob_base_velocity_smoother, moved to stack cob_driver
* namechanges from cob_vel_integrator to cob_base_velocity_smoother
* adjust manifests for documentation
* update deps
* integrate cob_vel_integrator yaml and fix wrong inclusions
* merge branch raw3-1 into master
* fix roslaunch tests
* refactoring of cob_navigation_local
* refactoring of cob_navigation_config; current cob_drivers required
* modifications for raw3-1
* rearrange stack
* add missed file
* rename cob_linear_nav to cob_navigation_linear, rearrange again
* fix include mistakes
* tidy up and rearrange
* Contributors: Alexander Bubeck, Florian Mirus, Florian Weißhardt, IPR-SR2, ipa-fmw, ipa-frm, ipa-mig
