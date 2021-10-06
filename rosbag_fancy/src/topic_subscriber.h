// Subcribes to input topics
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef ROSBAG_FANCY_TOPIC_SUBSCRIBER_H
#define ROSBAG_FANCY_TOPIC_SUBSCRIBER_H

#include <ros/message_event.h>
#include <ros/subscriber.h>
#include <ros/steady_timer.h>

namespace topic_tools { class ShapeShifter; }

namespace rosbag_fancy
{

class TopicManager;
class MessageQueue;
struct Topic;

class TopicSubscriber
{
public:
	explicit TopicSubscriber(TopicManager& topicManager, MessageQueue& queue);
private:
	void handle(Topic& topic, const ros::MessageEvent<topic_tools::ShapeShifter const>& msg);
	void updateStats();

	TopicManager& m_topicManager;
	MessageQueue& m_queue;

	std::vector<ros::Subscriber> m_subscribers;

	ros::SteadyTimer m_timer;
};

}

#endif
