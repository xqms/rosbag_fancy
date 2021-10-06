// rosbag_fancy GUI
// Author: Christian Lenz <lenz@ais.uni-bonn.de>

#include "fancy_gui.h"

#include "topic_model.h"

#include <rosfmt/rosfmt.h>
#include <ros/node_handle.h>
#include <ros/service.h>
#include <ros/master.h>
#include <pluginlib/class_list_macros.h>

#include <std_srvs/Trigger.h>

#include <QMessageBox>

Q_DECLARE_METATYPE(rosbag_fancy_msgs::StatusConstPtr)

namespace rqt_rosbag_fancy
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

	qRegisterMetaType<rosbag_fancy_msgs::StatusConstPtr>();
	connect(this, &FancyGui::receivedStatus, &m_model, &TopicModel::setState, Qt::QueuedConnection);
	connect(this, &FancyGui::receivedStatus, this, &FancyGui::updateView, Qt::QueuedConnection);

	m_ui.tableView->setModel(&m_model);

	QHeaderView *verticalHeader = m_ui.tableView->verticalHeader();
	verticalHeader->setDefaultSectionSize(verticalHeader->fontMetrics().height()+2);
	verticalHeader->hide();

	connect(m_ui.prefixComboBox, QOverload<int>::of(&QComboBox::activated), this, &FancyGui::subscribe);
	connect(m_ui.refreshButton, &QToolButton::clicked, this, &FancyGui::refreshTopicList);

	connect(m_ui.startButton, &QPushButton::clicked, this, &FancyGui::start);
	connect(m_ui.stopButton, &QPushButton::clicked, this, &FancyGui::stop);

	context.addWidget(m_w);
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
	ros::master::V_TopicInfo topics;
	ros::master::getTopics(topics);

	m_ui.prefixComboBox->clear();

	int idx = 0;
	for(const ros::master::TopicInfo& topic : topics)
	{
		if(topic.datatype != "rosbag_fancy_msgs/Status")
			continue;

		QString name = QString::fromStdString(topic.name);

		// Strip /status suffix
		if(name.endsWith("/status"))
			name = name.left(name.length() - 7);
		else
			continue;

		m_ui.prefixComboBox->addItem(name);
		if(name.toStdString() == m_prefix)
		{
			m_ui.prefixComboBox->setCurrentIndex(idx);
			subscribe();
		}
		++idx;
	}

}

void FancyGui::subscribe()
{
	ros::NodeHandle nh;
	std::string prefix = m_ui.prefixComboBox->currentText().toStdString();
	m_prefix = prefix;

	m_sub_status = nh.subscribe(prefix + "/status", 1, &FancyGui::receivedStatus, this);

	m_w->setWindowTitle(QString::fromStdString(m_prefix));
}

void FancyGui::updateView(const rosbag_fancy_msgs::StatusConstPtr& msg)
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
		case rosbag_fancy_msgs::Status::STATUS_PAUSED:
			m_ui.status->setText("PAUSED");
			m_ui.status->setStyleSheet("color: white; background-color: red;");
			m_ui.startButton->setEnabled(true);
			m_ui.stopButton->setEnabled(false);
			break;
		case rosbag_fancy_msgs::Status::STATUS_RUNNING:
			m_ui.status->setText("RUNNING");
			m_ui.status->setStyleSheet("color: white; background-color: green;");
			m_ui.startButton->setEnabled(false);
			m_ui.stopButton->setEnabled(true);
			break;
		default:
			m_ui.status->setText("UNKNOWN");
			m_ui.status->setStyleSheet("");
			m_ui.startButton->setEnabled(false);
			m_ui.stopButton->setEnabled(false);
			break;
	}
	
	m_ui.tableView->resizeRowsToContents();
}

QString FancyGui::rateToString(double rate) const
{
	std::string s;
	if(rate < 1000.0)
		s = fmt::format("{:.1f} Hz", rate);
	else if(rate < 1e6)
		s = fmt::format("{:.1f} kHz", rate / 1e3);
	else
		s = fmt::format("{:.1f} MHz", rate / 1e6);
	
	return QString::fromStdString(s);
}

QString FancyGui::memoryToString(uint64_t memory) const
{
	std::string s;
	if(memory < static_cast<uint64_t>(1<<10))
		s = fmt::format("{}.0 B", memory);
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

void FancyGui::start()
{
	std_srvs::Trigger srv;
	if(!ros::service::call(m_prefix + "/start", srv))
		QMessageBox::critical(m_w, "Error", "Could not call start service");
}

void FancyGui::stop()
{
	std_srvs::Trigger srv;
	if(!ros::service::call(m_prefix + "/stop", srv))
		QMessageBox::critical(m_w, "Error", "Could not call stop service");
}

}

PLUGINLIB_EXPORT_CLASS(rqt_rosbag_fancy::FancyGui, rqt_gui_cpp::Plugin)
