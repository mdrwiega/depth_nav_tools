^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package laserscan_kinect
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

X.Y.Z (2021.05.02)
------------------
* First release after migration to ROS2
* Fixed unit tests and ported to ROS2
* Ported configs/launchers/code to ROS2
* Abandon lazy subscription feature
* Cleanup of CMakelists

1.1.0 (2021.02.08)
------------------
* Add debug image to use during the tuning process
* Add parameter publish_dbg_info to enable/disable debug image publishing
* Fixed bug in ground removal feature
* Moved topics to the private namespace
* Units unification

1.0.1 (2016-11-11)
------------------
* Support for depthmaps with float32 encoding

1.0.0 (2016-01-01)
------------------
* Fix dynamic reconfigure bug associated with config header generation
* Attempt to fix dynamic reconfigure bug
* Dynamic configuration file update
* Add package laserscan_kinect to depth_nav_tools metapackage
* Contributors: Michal Drwiega
