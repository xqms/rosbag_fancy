// Output thread
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include "bag_writer.h"
#include "message_queue.h"

#include <rosfmt/rosfmt.h>

#include <boost/filesystem.hpp>

#include <ros/node_handle.h>

namespace rosbag_fancy
{

namespace
{
	std::string timeToString(const ros::Time& cur_ros_t)
	{
		std::time_t cur_t = cur_ros_t.sec;
		std::stringstream ss;
		ss << std::put_time(std::localtime(&cur_t), "%Y-%m-%d-%H-%M-%S");
		return ss.str();
	}
}

BagWriter::BagWriter(rosbag_fancy::MessageQueue& queue, const std::string& filename, Naming namingMode)
 : m_queue{queue}
 , m_filename{filename}
 , m_namingMode{namingMode}
{
	ros::NodeHandle nh;
	m_freeSpaceTimer = nh.createSteadyTimer(ros::WallDuration(5.0),
		boost::bind(&BagWriter::checkFreeSpace, this)
	);
	checkFreeSpace();

	m_thread = std::thread{std::bind(&BagWriter::run, this)};
}

BagWriter::~BagWriter()
{
	m_shouldShutdown = true;
	m_queue.shutdown();
	m_thread.join();
}

void BagWriter::run()
{
	while(!m_shouldShutdown)
	{
		auto msg = m_queue.pop();
		if(m_running && msg)
		{
			std::unique_lock<std::mutex> lock(m_mutex);
			m_bag.write(msg->topic, msg->message);
			m_sizeInBytes = m_bag.getSize();
		}
	}
}

void BagWriter::start()
{
	std::unique_lock<std::mutex> lock(m_mutex);

	if(m_running)
		return;

	std::string filename;
	switch(m_namingMode)
	{
		case Naming::AppendTimestamp:
			filename = fmt::format("{}_{}.bag", m_filename, timeToString(ros::Time::now()));
			break;
		case Naming::Verbatim:
			filename = m_filename;
			break;
	}

	if(!m_bagOpen)
	{
		ROSFMT_INFO("Opening bag file: {}", filename.c_str());
		m_bag.open(filename, rosbag::bagmode::Write);
	}

	m_bagOpen = true;
	m_running = true;
}

void BagWriter::stop()
{
	std::unique_lock<std::mutex> lock(m_mutex);

	switch(m_namingMode)
	{
		case Naming::AppendTimestamp:
			m_bag.close();
			m_bagOpen = false;
			break;
		case Naming::Verbatim:
			break;
	}

	m_running = false;

	ROSFMT_INFO("Recording stopped.");
}

void BagWriter::checkFreeSpace()
{
	namespace fs = boost::filesystem;

	auto space = fs::space(fs::absolute(fs::path(m_filename)).parent_path());

	m_freeSpace = space.available;
}

}
