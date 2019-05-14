
rosbag_fancy
============

![animation](https://xqms.github.io/rosbag_fancy/anim.svg)

`rosbag_fancy` is a fancy terminal UI frontend for the venerable [rosbag]
tool. It is highly experimental and the CLI should be considered unstable.

At the moment, only the `record` command is implemented. It offers the following
advantages over plain `rosbag record`:

 * Live display of statistics per topic, such as number of messages, bandwidth,
   dropped messages, etc
 * Bash completion for topic names

[rosbag]: http://wiki.ros.org/rosbag
