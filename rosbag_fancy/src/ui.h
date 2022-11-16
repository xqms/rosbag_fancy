// Terminal UI
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef ROSBAG_FANCY_UI_H
#define ROSBAG_FANCY_UI_H

#include "terminal.h"
#include "topic_manager.h"
#include "message_queue.h"

#include <ros/steady_timer.h>

namespace rosbag_fancy
{

class BagWriter;
class BagReader;

class UI
{
public:
	UI(TopicManager& config, MessageQueue& queue, BagWriter& writer);

	void draw();

private:
	template<class... Args>
	void printLine(unsigned int& lineCounter, const Args& ... args);

	TopicManager& m_topicManager;
	MessageQueue& m_queue;
	BagWriter& m_bagWriter;

	Terminal m_term;

	ros::SteadyTimer m_timer;
	ros::WallTime m_lastDrawTime;
};

class PlaybackUI
{
public:
	explicit PlaybackUI(TopicManager& topics, BagReader& reader);

	void setPositionInBag(const ros::Time& stamp);

	void draw();

private:
	template<class... Args>
	void printLine(unsigned int& lineCounter, const Args& ... args);

	TopicManager& m_topicManager;
	BagReader& m_bagReader;

	Terminal m_term;

	ros::SteadyTimer m_timer;
	ros::WallTime m_lastDrawTime;

	ros::Time m_positionInBag;
};

}

#endif
