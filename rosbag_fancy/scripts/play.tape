# Where should we write the GIF?
Output play.gif

Set TypingSpeed 50ms
Set FontSize 24
Set Width 1300
Set Height 680
Set Padding 10
Set Framerate 24

################################### RECORD ###########################################

Type "rosbag_fancy record -o out.bag /tf /test /misspelled"
Sleep 500ms
Enter

Sleep 5s

Ctrl+C

Sleep 1s

################################### INFO ###########################################

Type "rosbag_fancy info out.bag"
Sleep 500ms
Enter

Sleep 5s

Hide
Type "clear"
Enter
Show

################################### PLAY #############################################
# Type a command in the terminal.
Type "rosbag_fancy play b0-2014-07-11-10-58-16.bag"

# Pause for dramatic effect...
Sleep 500ms

# Run the command by pressing enter.
Enter

# Admire the output for a bit.
Sleep 5s

# Skip forward
Right
Sleep 500ms
Right
Sleep 500ms
Right

Sleep 2s

# Skip backward
Left
Sleep 500ms
Left
Sleep 500ms
Left

Sleep 2s

Type " "

Sleep 2s

Type " "

Sleep 2s

Type "q"
