^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbag_fancy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* play: refuse to play non-monotonic bags
* Add --no-ui for play with status info and start/stop/pause services (PR #19)
* cmd_play: basic topic information in status message
* rosbag_fancy: link executable for devel space
  Fixes rosbag_fancy usage in catkin devel spaces (which is the default
  with catkin_make and catkin_tools). I mainly test & develop with colcon,
  so I did not notice so far that the command was not available properly.
* Contributors: Jan Quenzel, Max Schwarz

1.0.1 (2023-07-11)
------------------
* add missing std_srvs dependency
* Contributors: Max Schwarz

1.0.0 (2023-07-10)
------------------
* play: decompression support (PR #18)
* BagView & TF2 scanner (PR #17)
* CI & unit tests
* play: support for multiple bag files
* play: ui: show current date/time
* play: support for --clock
* ui: remove sub column
* play: faster startup, don't crash if select() is interrupted
* play command
* record: wait for ros::Time to become valid
  This fixes recording with use_sim_time=true (Issue #16)
* tui: fix count display
* tui: display byte & message counts correctly
* GUI/TUI: display messages in bag, not total messages
* rosbag_fancy: properly initialize compression
* split into separate packages (main, _msgs, and _gui)
* Contributors: Max Schwarz

0.2.0 (2020-06-16)
------------------
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
