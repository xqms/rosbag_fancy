
rosbag_fancy
============

<img src="https://xqms.github.io/rosbag_fancy/anim2.svg" width="100%" />

`rosbag_fancy` is a fancy terminal UI frontend for the venerable [rosbag]
tool.

Recording
---------

At the moment, the `record` command is `rosbag_fancy`'s main feature.
It offers the following advantages over plain `rosbag record`:

 * Live display of statistics per topic, such as number of messages, bandwidth,
   dropped messages, etc. Never notice *after* recording that you misspelled a
   topic name!
 * Bash completion for topic names
 * Optional per-topic rate limiting

Info
----

`rosbag_fancy` also offers an `info` command similar to `rosbag info`, which
has the same features but is much faster.

Playback
--------

Using `rosbag_fancy play <bagfile>` you can play bag files interactively.
Similar to `info`, this is much faster than plain `rosbag`.

As an additional feature, `rosbag_fancy play` aggregates the `tf_static`
topic over time, so no matter how many tf publishers were active or how
much you seek in the file, the static transforms will be kept up-to-date.

[rosbag]: http://wiki.ros.org/rosbag
