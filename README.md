
rosbag_fancy
============

<img src="https://xqms.github.io/rosbag_fancy/anim2.svg" width="100%" />

`rosbag_fancy` is a fancy terminal UI frontend for the venerable [rosbag]
tool.

At the moment, the `record` command is `rosbag_fancy`'s main feature.
It offers the following advantages over plain `rosbag record`:

 * Live display of statistics per topic, such as number of messages, bandwidth,
   dropped messages, etc. Never notice *after* recording that you misspelled a
   topic name!
 * Bash completion for topic names
 * Optional per-topic rate limiting

`rosbag_fancy` also offers an `info` command similar to `rosbag info`, which
has the same features but is much faster.

[rosbag]: http://wiki.ros.org/rosbag
