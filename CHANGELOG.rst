^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbag_fancy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* bag_writer: fix uninitialized read in run() during shutdown
  This happens in certain cases during shutdown, where msg is not set.
* bag_writer: make sure we don't overwrite bag files with same stamp
* ui: move paused/recording indicator to bottom
* bag_writer: special handling for tf2 static transforms on bag reopen
* add --paused flag to start in paused mode
* UI improvements for start/stop
* start/stop service calls
* add tf2_republisher tool (helpful for playback)
* topic_manager: compile fix for clang
* Contributors: Max Schwarz

0.1.1 (2019-10-22)
------------------
* Compatibility with ROS Kinetic (PR: #7)
  also helps with Debian Stretch
* Contributors: Davide Faconti, Max Schwarz

0.1.0 (2019-10-18)
------------------
* Initial release 
* Contributors: Jan Quenzel, Max Schwarz
