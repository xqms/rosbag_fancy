// Contains the topic configuration & status
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "topic_manager.h"

#include <ros/node_handle.h>

namespace rosbag_fancy
{

constexpr float STAT_TIME = 0.5;

const ros::WallTime Topic::T0 = ros::WallTime::now();

TopicManager::TopicManager()
{
	ros::NodeHandle nh;
	m_timer = nh.createSteadyTimer(ros::WallDuration(STAT_TIME),
		boost::bind(&TopicManager::updateStatistics, this)
	);
}

void TopicManager::addTopic(const std::string& topic, float rateLimit)
{
	m_topics.emplace_back(topic, rateLimit);
}

void TopicManager::updateStatistics()
{
	for(auto& topic : m_topics)
	{
		topic.messageRate = topic.messagesInStatsPeriod / STAT_TIME;
		topic.messagesInStatsPeriod = 0;

		topic.bandwidth = topic.bytesInStatsPeriod / STAT_TIME;
		topic.bytesInStatsPeriod = 0;
	}
}

}
