find_package(Qt5Widgets REQUIRED)

include_directories(${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR})

qt5_wrap_ui(UI_HDRS
	fancy_gui.ui
)

qt5_wrap_cpp(MOC_SRCS
	fancy_gui.h
	topic_model.h
)

add_library(rosbag_fancy_gui
	${UI_HDRS}
	${MOC_SRCS}
	fancy_gui.cpp
	topic_model.cpp
)
target_link_libraries(rosbag_fancy_gui
	${catkin_LIBRARIES}
	Qt5::Widgets
)
add_dependencies(rosbag_fancy_gui
	rosbag_fancy_generate_messages_cpp
)

install(TARGETS rosbag_fancy_gui
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(FILES ${PROJECT_SOURCE_DIR}/rqt_plugins.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

