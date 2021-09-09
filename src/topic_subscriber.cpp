// Subcribes to input topics
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include "topic_subscriber.h"
#include "message_queue.h"
#include "topic_manager.h"

namespace rosbag_fancy
{

TopicSubscriber::TopicSubscriber(rosbag_fancy::TopicManager& topicManager, rosbag_fancy::MessageQueue& queue)
 : m_topicManager{topicManager}
 , m_queue{queue}
{
	ros::NodeHandle nh;

	for(auto& topic : topicManager.topics())
	{
		ros::TransportHints transportHints;

		if(topic.flags & static_cast<int>(Topic::Flag::UDP))
			transportHints = transportHints.udp();

		boost::function<void(const ros::MessageEvent<topic_tools::ShapeShifter const>&)> cb{
			boost::bind(&TopicSubscriber::handle, this, boost::ref(topic), _1)
		};
		m_subscribers.push_back(nh.subscribe<topic_tools::ShapeShifter>(
			topic.name, 10,
			cb, {}, transportHints
		));
	}

	m_timer = nh.createSteadyTimer(ros::WallDuration(1.0),
		boost::bind(&TopicSubscriber::updateStats, this)
	);
}

void TopicSubscriber::handle(Topic& topic, const ros::MessageEvent<topic_tools::ShapeShifter const>& msg)
{
	std::uint64_t bytes = msg.getConstMessage()->size();

	if(topic.rateLimit != 0.0f)
	{
		ros::Time now = ros::Time::now();
		if(topic.lastMessageReceivedROSTime == ros::Time(0))
			topic.lastMessageReceivedROSTime = now;
		else
		{
			// Basic token bucket algorithm for rate limiting
			ros::Duration elapsed = now - topic.lastMessageReceivedROSTime;
			topic.lastMessageReceivedROSTime = now;

			topic.throttleAllowance = std::min(2.0f,
				topic.throttleAllowance + static_cast<float>(elapsed.toSec()) * topic.rateLimit
			);

			if(topic.throttleAllowance < 1.0f)
				return;

			topic.throttleAllowance -= 1.0f;
		}
	}

	topic.notifyMessage(bytes);

	if(!m_queue.push({topic.name, msg, &topic}))
		topic.dropCounter++;
}

void TopicSubscriber::updateStats()
{
	for(std::size_t i = 0; i < m_topicManager.topics().size(); ++i)
	{
		auto& topic = m_topicManager.topics()[i];
		auto& sub = m_subscribers[i];

		topic.numPublishers = sub.getNumPublishers();
	}
}


}
