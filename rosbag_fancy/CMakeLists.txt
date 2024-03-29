
cmake_minimum_required(VERSION 3.0)
project(rosbag_fancy)

find_package(catkin REQUIRED COMPONENTS
	rosbag_fancy_msgs
	rosbag_storage
	roscpp
	rosfmt
	roslz4
	std_srvs
	topic_tools
	tf2_ros
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

find_package(Curses REQUIRED)
include_directories(${CURSES_INCLUDE_DIRS})

find_package(BZip2 REQUIRED)

include_directories(contrib/doctest)

set(CMAKE_CXX_STANDARD 17)

add_executable(rosbag_fancy
	src/bag_reader.cpp
	src/bag_view.cpp
	src/bag_writer.cpp
	src/cmd_info.cpp
	src/cmd_play.cpp
	src/cmd_record.cpp
	src/cmd_test.cpp
	src/doctest.cpp
	src/main.cpp
	src/mem_str.cpp
	src/message_queue.cpp
	src/terminal.cpp
	src/tf2_scanner.cpp
	src/topic_manager.cpp
	src/topic_subscriber.cpp
	src/ui.cpp
)
target_link_libraries(rosbag_fancy
	${catkin_LIBRARIES}
	${CURSES_LIBRARIES}
	BZip2::BZip2
)

# Tools
add_executable(tf2_republisher
	tools/tf2_republisher.cpp
)
target_link_libraries(tf2_republisher
	${catkin_LIBRARIES}
)

# Shell helper
# Version 1.11 (increment this comment to trigger a CMake update)
catkin_add_env_hooks(50-rosbag_fancy
	SHELLS bash
	DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks
)

# Link executable into bin/ in the devel space
add_custom_target(symlink ALL
	COMMAND ${CMAKE_COMMAND} -E make_directory ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_BIN_DESTINATION}
	COMMAND ${CMAKE_COMMAND} -E create_symlink $<TARGET_FILE:rosbag_fancy> ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_BIN_DESTINATION}/rosbag_fancy
)


install(TARGETS rosbag_fancy tf2_republisher
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS rosbag_fancy
	RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
