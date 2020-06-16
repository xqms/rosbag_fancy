// Output thread
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include "bag_writer.h"
#include "message_queue.h"

#include <rosfmt/rosfmt.h>

#include <boost/filesystem.hpp>

#include <ros/node_handle.h>

#include <tf2_msgs/TFMessage.h>

namespace fs = boost::filesystem;

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

	std::string getNewFilename(const std::string& old)
	{
		for(int i = 2; i < 10; ++i)
		{
			auto tst_filename = fmt::format("{}.{}", old, i);
			if(!fs::exists(tst_filename))
				return tst_filename;
		}

		return {};
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

	m_tf_header = boost::make_shared<std::map<std::string, std::string>>();
	{
		auto& connectionHeader = *m_tf_header;
		connectionHeader["topic"] = "/tf_static";
		connectionHeader["callerid"] = ros::this_node::getName();
		connectionHeader["md5sum"] = ros::message_traits::MD5Sum<tf2_msgs::TFMessage>::value();
		connectionHeader["type"] = "tf2_msgs/TFMessage";
		connectionHeader["message_definition"] = ros::message_traits::Definition<tf2_msgs::TFMessage>::value();
		connectionHeader["latching"] = "1";
	}

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

		{
			std::unique_lock<std::mutex> lock(m_mutex);
			if(m_running && msg)
			{
				m_bag.write(msg->topic, msg->message);
				m_sizeInBytes = m_bag.getSize();
			}
		}

		if(msg && msg->topic == "/tf_static")
		{
			auto shifter = msg->message.getMessage();
			auto tf_msg = shifter->instantiate<tf2_msgs::TFMessage>();
			if(tf_msg)
			{
				for(auto& transformMsg : tf_msg->transforms)
					m_tf_buf.setTransform(transformMsg, "bag", true);
			}
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
		// Don't overwrite existing files
		if(fs::exists(filename))
		{
			auto replacement = getNewFilename(filename);
			if(replacement.empty())
			{
				ROSFMT_ERROR("Could not find a bag file name to write to (template '{}')", filename);
				ROSFMT_ERROR("Ignoring start request.");
				m_bagOpen = false;
				m_running = false;
				return;
			}

			filename = replacement;
		}

		ROSFMT_INFO("Opening bag file: {}", filename.c_str());
		m_bag.open(filename, rosbag::bagmode::Write);

		// Write all known transforms to /tf_static
		{
			auto tf_msg = boost::make_shared<tf2_msgs::TFMessage>();
			std::vector<std::string> frames;
			m_tf_buf._getFrameStrings(frames);

			for(auto& frame : frames)
			{
				std::string parent;
				if(!m_tf_buf._getParent(frame, ros::Time(0), parent))
					continue;

				geometry_msgs::TransformStamped transform = m_tf_buf.lookupTransform(parent, frame, ros::Time(0));
				tf_msg->transforms.push_back(std::move(transform));
			}

			ros::MessageEvent<tf2_msgs::TFMessage> event{tf_msg, m_tf_header, ros::Time::now()};

			m_bag.write("/tf_static", event);
		}
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
