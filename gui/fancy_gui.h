// rosbag_fancy rqt gui
// Author: Christian Lenz <lenz@ais.uni-bonn.de>

#ifndef ROSBAGFANCY_GUI_H
#define ROSBAGFANCY_GUI_H

#include <rqt_gui_cpp/plugin.h>
#include <std_msgs/String.h>

#include <ros/subscriber.h>

#include <rosbag_fancy/Status.h>

#include "topic_model.h"
#include "ui_fancy_gui.h"

namespace rosbag_fancy
{

class FancyGui : public rqt_gui_cpp::Plugin
{
Q_OBJECT
public:
	FancyGui();
	virtual ~FancyGui();

	virtual void initPlugin(qt_gui_cpp::PluginContext& context) override;
	virtual void shutdownPlugin() override;
	virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const override;
	virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings) override;

Q_SIGNALS:
	void receivedStatus(const rosbag_fancy::StatusConstPtr& msg);
public Q_SLOTS:
	void refreshTopicList();
	void subscribe();
	void updateView(const rosbag_fancy::StatusConstPtr& msg);
private:
	ros::Subscriber m_sub_status;

	QWidget* m_w;
	Ui_FancyGui m_ui;

	TopicModel m_model;

	std::string m_prefix;
	
	QString rateToString(double rate) const;
	QString memoryToString(uint64_t memory) const;
};

}

#endif
