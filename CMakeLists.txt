
cmake_minimum_required(VERSION 3.0)
project(rosbag_fancy)

find_package(catkin REQUIRED COMPONENTS
	rosbag_storage
	roscpp
	rosfmt
	topic_tools
	tf2_ros
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

find_package(Curses REQUIRED)
include_directories(${CURSES_INCLUDE_DIRS})

set(CMAKE_CXX_STANDARD 14)

add_executable(rosbag_fancy
	src/bag_writer.cpp
	src/main.cpp
	src/message_queue.cpp
	src/terminal.cpp
	src/topic_subscriber.cpp
	src/topic_manager.cpp
	src/ui.cpp
)
target_link_libraries(rosbag_fancy
	${catkin_LIBRARIES}
	${CURSES_LIBRARIES}
)

# Tools
add_executable(tf2_republisher
	tools/tf2_republisher.cpp
)
target_link_libraries(tf2_republisher
	${catkin_LIBRARIES}
)

# Shell helper
# Version 1.7 (increment this comment to trigger a CMake update)
catkin_add_env_hooks(50-rosbag_fancy
	SHELLS bash
	DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks
)

install(TARGETS rosbag_fancy
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
