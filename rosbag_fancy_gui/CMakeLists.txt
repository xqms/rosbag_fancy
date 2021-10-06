
cmake_minimum_required(VERSION 3.0)
project(rosbag_fancy_gui)

find_package(catkin REQUIRED COMPONENTS
	rosbag_fancy_msgs
	rosfmt
	rqt_gui
	qt_gui_cpp
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

find_package(Qt5Widgets REQUIRED)

include_directories(${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR})

qt5_wrap_ui(UI_HDRS
	src/fancy_gui.ui
)

qt5_wrap_cpp(MOC_SRCS
	src/fancy_gui.h
	src/topic_model.h
)

add_library(rosbag_fancy_gui
	${UI_HDRS}
	${MOC_SRCS}
	src/fancy_gui.cpp
	src/topic_model.cpp
)
target_link_libraries(rosbag_fancy_gui
	${catkin_LIBRARIES}
	Qt5::Widgets
)

install(TARGETS rosbag_fancy_gui
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(FILES ${PROJECT_SOURCE_DIR}/rqt_plugins.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
