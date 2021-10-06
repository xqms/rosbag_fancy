// Contains the topic configuration & status
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "topic_manager.h"

#include <ros/node_handle.h>
#include <rosfmt/rosfmt.h>

#include <ros/names.h>

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

void TopicManager::addTopic(const std::string& topic, float rateLimit, int flags)
{
	std::string resolvedName = ros::names::resolve(topic);

	auto it = std::find_if(m_topics.begin(), m_topics.end(), [&](Topic& t){
		return t.name == resolvedName;
	});

	if(it != m_topics.end())
	{
		ROSFMT_WARN(
			"You tried to record topic '{}' twice. I'll ignore that (and use the first rate limit given, if applicable)",
			resolvedName
		);
		return;
	}

	m_topics.emplace_back(resolvedName, m_topics.size(), rateLimit, flags);
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
