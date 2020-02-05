// Output thread
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef ROSBAG_FANCY_BAG_WRITER_H
#define ROSBAG_FANCY_BAG_WRITER_H

#include <atomic>
#include <string>
#include <thread>
#include <mutex>

#include <rosbag/bag.h>
#include <ros/steady_timer.h>

namespace rosbag_fancy
{

class MessageQueue;

class BagWriter
{
public:
	enum class Naming
	{
		Verbatim,
		AppendTimestamp
	};

	explicit BagWriter(MessageQueue& queue, const std::string& filename, Naming namingMode);
	~BagWriter();

	void start();
	void stop();
	bool running() const
	{ return m_running; }

	std::uint64_t sizeInBytes() const
	{ return m_sizeInBytes; }

	std::uint64_t freeSpace() const
	{ return m_freeSpace; }
private:
	void run();

	void checkFreeSpace();

	MessageQueue& m_queue;

	std::string m_filename;
	Naming m_namingMode;

	rosbag::Bag m_bag;
	bool m_bagOpen{false};

	std::thread m_thread;

	bool m_shouldShutdown{false};

	std::atomic<std::uint64_t> m_sizeInBytes{0};
	std::uint64_t m_freeSpace = 0;

	ros::SteadyTimer m_freeSpaceTimer;

	std::atomic<bool> m_running{false};
	std::mutex m_mutex;
};

}

#endif
