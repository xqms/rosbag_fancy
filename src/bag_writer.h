// Output thread
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef ROSBAG_FANCY_BAG_WRITER_H
#define ROSBAG_FANCY_BAG_WRITER_H

#include <atomic>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <rosbag/bag.h>
#include <ros/steady_timer.h>

#include <tf2_ros/buffer.h>

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

	explicit BagWriter(
		MessageQueue& queue, const std::string& filename, Naming namingMode,
                std::uint64_t splitSizeInBytes, std::uint64_t deleteOldAtInBytes
	);
	~BagWriter();

	void start();
	void stop();
	bool running() const
	{ return m_running; }

	std::uint64_t sizeInBytes() const
	{ return m_sizeInBytes; }

	std::uint64_t splitSizeInBytes() const
	{ return m_splitSizeInBytes; }

	std::uint64_t directorySizeInBytes() const
	{ return m_directorySizeInBytes; }

	std::uint64_t deleteOldAtInBytes() const
	{ return m_deleteOldAtInBytes; }

	std::uint64_t freeSpace() const
	{ return m_freeSpace; }

	std::string bagfileName() const
	{ return m_expandedFilename; }

	const std::vector<std::uint64_t>& messageCounts() const
	{ return m_messageCounts; }
private:
	void run();
	void cleanupThread();

	void checkFreeSpace();

	MessageQueue& m_queue;

	std::string m_filename;
	Naming m_namingMode;

	std::string m_expandedFilename;

	bool m_isReopeningBag{false};
	std::uint64_t m_splitSizeInBytes = 0;
	std::uint64_t m_deleteOldAtInBytes = 0;
	std::atomic<std::uint64_t> m_directorySizeInBytes{0};

	rosbag::Bag m_bag;
	bool m_bagOpen{false};

	std::thread m_thread;
	std::thread m_cleanup_thread;

	bool m_shouldShutdown{false};

	std::atomic<std::uint64_t> m_sizeInBytes{0};
	std::uint64_t m_freeSpace = 0;

	ros::SteadyTimer m_freeSpaceTimer;

	std::atomic<bool> m_running{false};
	std::mutex m_mutex;

	std::mutex m_cleanupMutex;
	std::condition_variable m_cleanupCondition;

	tf2_ros::Buffer m_tf_buf;
	boost::shared_ptr<std::map<std::string, std::string>> m_tf_header;

	std::vector<std::uint64_t> m_messageCounts;
};

}

#endif
