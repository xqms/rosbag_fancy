// Rosbag_fancy gui
// Author: Christian Lenz <lenz@ais.uni-bonn.de>

#include "fancy_gui.h"

#include "topic_model.h"

#include <rosfmt/full.h>
#include <ros/node_handle.h>
#include <ros/service.h>
#include <ros/master.h>
#include <boost/foreach.hpp>

#include <pluginlib/class_list_macros.h>

Q_DECLARE_METATYPE(rosbag_fancy::StatusConstPtr)

namespace rosbag_fancy
{

FancyGui::FancyGui()
{
}

FancyGui::~FancyGui()
{
}

void FancyGui::initPlugin(qt_gui_cpp::PluginContext& context)
{
	m_w = new QWidget;
	m_ui.setupUi(m_w);

	context.addWidget(m_w);

	ros::NodeHandle nh = getPrivateNodeHandle();

	qRegisterMetaType<rosbag_fancy::StatusConstPtr>();
	connect(this, &FancyGui::receivedStatus, &m_model, &TopicModel::setState, Qt::QueuedConnection);
	connect(this, &FancyGui::receivedStatus, this, &FancyGui::updateView, Qt::QueuedConnection);

	m_ui.tableView->setModel(&m_model);

	QHeaderView *verticalHeader = m_ui.tableView->verticalHeader();
	verticalHeader->setDefaultSectionSize(verticalHeader->fontMetrics().height()+2);
	verticalHeader->hide();

	QObject::connect(m_ui.prefixComboBox, SIGNAL(activated(QString)), SLOT(subscribe()));
	QObject::connect(m_ui.refreshButton, SIGNAL(clicked(bool)), SLOT(refreshTopicList()));

}

void FancyGui::shutdownPlugin()
{
    m_sub_status.shutdown();
}

void FancyGui::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
	qt_gui_cpp::Plugin::saveSettings(plugin_settings, instance_settings);

	instance_settings.setValue("prefix", QString::fromStdString(m_prefix));
	instance_settings.setValue("columns", m_ui.tableView->horizontalHeader()->saveState());
}

void FancyGui::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
	qt_gui_cpp::Plugin::restoreSettings(plugin_settings, instance_settings);

	m_prefix = instance_settings.value("prefix").toString().toStdString();
	refreshTopicList();

	m_ui.tableView->horizontalHeader()->restoreState(instance_settings.value("columns").toByteArray());
}

void FancyGui::refreshTopicList()
{
// 	if(m_shuttingDown)
// 		return;

	ros::master::V_TopicInfo topics;
	ros::master::getTopics(topics);

	m_ui.prefixComboBox->clear();

	int idx = 0;
	BOOST_FOREACH(const ros::master::TopicInfo topic, topics)
	{
		if(topic.datatype != "rosbag_fancy/Status")
			continue;

		std::string prefix = topic.name;

		m_ui.prefixComboBox->addItem(QString::fromStdString(prefix));
		if(prefix == m_prefix)
		{
			m_ui.prefixComboBox->setCurrentIndex(idx);
			subscribe();
		}
		++idx;
	}

}

void FancyGui::subscribe()
{
	ros::NodeHandle nh = getPrivateNodeHandle();
	std::string prefix = m_ui.prefixComboBox->currentText().toStdString();
	m_prefix = prefix;

	m_sub_status = nh.subscribe(prefix, 1, &FancyGui::receivedStatus, this);

	m_w->setWindowTitle(QString::fromStdString(m_prefix));
}

void FancyGui::updateView(const rosbag_fancy::StatusConstPtr& msg)
{
	m_ui.file_name->setText(QString::fromStdString(msg->bagfile));
	m_ui.bandwidth->setText(memoryToString(msg->bandwidth) + "/s");
	m_ui.size->setText(memoryToString(msg->bytes));
	m_ui.free_space->setText(memoryToString(msg->free_bytes));
	
	int totalMsgs = 0;
	for(auto& t : msg->topics)
		totalMsgs += t.messages;
	
	m_ui.messages->setText(QString::number(totalMsgs));

	switch(msg->status)
	{
		case rosbag_fancy::Status::STATUS_PAUSED:
			m_ui.status->setText("PAUSED");
			break;
		case rosbag_fancy::Status::STATUS_RUNNING:
			m_ui.status->setText("RUNNING");
			break;
		default:
			m_ui.status->setText("UNKNOWN");
			break;
	}
	
	m_ui.tableView->resizeRowsToContents();
}

QString FancyGui::rateToString(double rate) const
{
	std::string s;
	if(rate < 1000.0)
		s = fmt::format("{:5.1f}  Hz", rate);
	else if(rate < 1e6)
		s = fmt::format("{:5.1f} kHz", rate / 1e3);
	else
		s = fmt::format("{:5.1f} MHz", rate / 1e6);
	
	return QString::fromStdString(s);
}

QString FancyGui::memoryToString(uint64_t memory) const
{
	std::string s;
	if(memory < static_cast<uint64_t>(1<<10))
		s = fmt::format("{}.0   B", memory);
	else if(memory < static_cast<uint64_t>(1<<20))
		s = fmt::format("{:.1f} KiB", static_cast<double>(memory) / static_cast<uint64_t>(1<<10));
	else if(memory < static_cast<uint64_t>(1<<30))
		s = fmt::format("{:.1f} MiB", static_cast<double>(memory) / static_cast<uint64_t>(1<<20));
	else if(memory < static_cast<uint64_t>(1ull<<40))
		s = fmt::format("{:.1f} GiB", static_cast<double>(memory) / static_cast<uint64_t>(1ull<<30));
	else
		s = fmt::format("{:.1f} TiB", static_cast<double>(memory) / static_cast<uint64_t>(1ull<<40));
	
	return QString::fromStdString(s);
}

}//NS

PLUGINLIB_EXPORT_CLASS(rosbag_fancy::FancyGui, rqt_gui_cpp::Plugin)
