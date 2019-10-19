// Output thread
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include "bag_writer.h"
#include "message_queue.h"
#include <boost/filesystem.hpp>
#include <ros/node_handle.h>

namespace rosbag_fancy
{

BagWriter::BagWriter(rosbag_fancy::MessageQueue& queue)
 : m_queue{queue}
{
	ros::NodeHandle nh;
	m_freeSpaceTimer = nh.createSteadyTimer(ros::WallDuration(5.0),
		boost::bind(&BagWriter::checkFreeSpace, this)
	);
	checkFreeSpace();
}

BagWriter::~BagWriter()
{
  if(m_opened)
	{
		m_shouldShutdown = true;
		m_queue.shutdown();
		m_thread.join();
	}
}

void BagWriter::run()
{
	while(!m_shouldShutdown)
	{
		auto msg = m_queue.pop();
		if(msg)
		{
			m_bag.write(msg->topic, msg->message);
			m_sizeInBytes = m_bag.getSize();
		}
	}
}

void BagWriter::start(const std::string& filename)
{
	m_filename = filename;

	m_bag.open(filename, rosbag::bagmode::Write);
  m_opened = true;
	m_thread = std::thread{std::bind(&BagWriter::run, this)};
}

void BagWriter::checkFreeSpace()
{
	namespace fs = boost::filesystem;

	auto space = fs::space(fs::absolute(fs::path(m_filename)).parent_path());

	m_freeSpace = space.available;
}

}
