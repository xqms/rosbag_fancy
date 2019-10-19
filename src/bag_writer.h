// Output thread
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef ROSBAG_FANCY_BAG_WRITER_H
#define ROSBAG_FANCY_BAG_WRITER_H

#include <string>
#include <thread>
#include <atomic>
#include <rosbag/bag.h>
#include <ros/steady_timer.h>

namespace rosbag_fancy
{

class MessageQueue;

class BagWriter
{
public:
	explicit BagWriter(MessageQueue& queue);
	~BagWriter();

	void start(const std::string& filename);

	std::uint64_t sizeInBytes() const
	{ return m_sizeInBytes; }

	std::uint64_t freeSpace() const
	{ return m_freeSpace; }
private:
	void run();

	void checkFreeSpace();

	MessageQueue& m_queue;
	rosbag::Bag m_bag;

	std::thread m_thread;

	bool m_shouldShutdown{false};
  bool m_opened{false};

  std::atomic<std::uint64_t> m_sizeInBytes{0};
	std::uint64_t m_freeSpace = 0;

	ros::SteadyTimer m_freeSpaceTimer;

	std::string m_filename;
};

}

#endif
