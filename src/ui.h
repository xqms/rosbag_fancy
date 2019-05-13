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

class UI
{
public:
	enum class Mode
	{
		Recording,
		Playback
	};

	UI(TopicManager& config, MessageQueue& queue, BagWriter& writer, Mode mode);

	void draw();

private:
	TopicManager& m_topicManager;
	MessageQueue& m_queue;
	BagWriter& m_bagWriter;

	Mode m_mode;
	Terminal m_term;

	ros::SteadyTimer m_timer;
	ros::WallTime m_lastDrawTime;
};

}

#endif
