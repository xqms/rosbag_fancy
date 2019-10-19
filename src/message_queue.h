// In-Memory message buffer
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef ROSBAG_FANCY_MESSAGE_QUEUE_H
#define ROSBAG_FANCY_MESSAGE_QUEUE_H

#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <ros/message_event.h>
#include <boost/optional.hpp>
#include <topic_tools/shape_shifter.h>

namespace rosbag_fancy
{

class MessageQueue
{
public:
	struct Message
	{
		std::string topic;
		ros::MessageEvent<const topic_tools::ShapeShifter> message;

		uint64_t size() const
		{ return message.getConstMessage()->size(); }
	};

	explicit MessageQueue(uint64_t byteLimit);

	bool push(const Message& msg);
  boost::optional<Message> pop();

	void shutdown();

	uint64_t bytesInQueue() const
	{ return m_bytesInQueue; }

	uint64_t messagesInQueue() const
	{ return m_msgsInQueue; }
private:
	std::queue<Message> m_queue;
	std::mutex m_mutex;
	std::condition_variable m_cond;

  std::atomic<std::uint64_t> m_bytesInQueue{0};
  std::atomic<std::uint64_t> m_msgsInQueue{0};
	uint64_t m_byteLimit;

	bool m_shuttingDown{false};
};

}

#endif
