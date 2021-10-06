
cmake_minimum_required(VERSION 3.0)
project(rosbag_fancy_msgs)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	std_msgs
)

add_message_files(FILES
    Status.msg
    TopicStatus.msg
)

generate_messages(DEPENDENCIES
    std_msgs
)

catkin_package()
