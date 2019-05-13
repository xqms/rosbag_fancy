// In-Memory message buffer
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include "message_queue.h"

namespace rosbag_fancy
{

MessageQueue::MessageQueue(uint64_t byteLimit)
 : m_byteLimit{byteLimit}
{
}

bool MessageQueue::push(const rosbag_fancy::MessageQueue::Message& msg)
{
	uint64_t len = msg.size();
	if(m_bytesInQueue + len > m_byteLimit)
		return false;

	{
		std::unique_lock<std::mutex> lock(m_mutex);
		m_queue.push(msg);
		m_bytesInQueue += len;
		m_msgsInQueue++;

		m_cond.notify_all();
	}

	return true;
}

std::optional<MessageQueue::Message> MessageQueue::pop()
{
	std::unique_lock<std::mutex> lock(m_mutex);

	if(m_shuttingDown)
		return {};

	while(m_queue.empty())
	{
		m_cond.wait(lock);
		if(m_shuttingDown)
			return {};
	}

	auto msg = m_queue.front();
	uint64_t len = msg.size();

	m_bytesInQueue -= len;
	m_msgsInQueue--;
	m_queue.pop();

	return {msg};
}

void MessageQueue::shutdown()
{
	m_shuttingDown = true;
	m_cond.notify_all();
}

}
