// Output thread
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include "bag_writer.h"
#include "message_queue.h"
#include "topic_manager.h"

#include <rosfmt/rosfmt.h>

#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#include <map>

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

	std::vector<fs::path> getBagFilesInCurrentFolder(const std::string& filename)
	{
		std::vector<fs::path> bagFiles;
		auto path = fs::absolute(fs::path(filename)).parent_path();

		for(auto& entry : boost::make_iterator_range(fs::directory_iterator(path), {}))
		{
			auto filePath = entry.path();
			if(fs::extension(filePath) == ".bag")
			{
				bagFiles.emplace_back(filePath);
			}
		}

		return bagFiles;
	}

	std::vector<fs::path> sortFilesByTime(const std::vector<fs::path>& files)
	{
		std::multimap<std::time_t,fs::path> timeFiles;
		for(auto & entry : files)
		{
			timeFiles.emplace(fs::last_write_time(entry),entry);
		}

		std::vector<fs::path> timeSortedFiles;
		if(!timeFiles.empty())
			std::transform(timeFiles.begin(), timeFiles.end(), std::back_inserter(timeSortedFiles), [](auto &kv){ return kv.second;});

		return timeSortedFiles;
	}

	std::uint64_t getTotalSizeInBytes(const std::vector<fs::path>& files)
	{
		std::uint64_t totalSizeInBytes = 0;
		for(auto& entry : files)
			totalSizeInBytes += fs::file_size(entry);

		return totalSizeInBytes;
	}
}

BagWriter::BagWriter(rosbag_fancy::MessageQueue& queue, const std::string& filename, Naming namingMode, std::uint64_t splitSizeInBytes, std::uint64_t deleteOldAtInBytes)
 : m_queue{queue}
 , m_filename{filename}
 , m_namingMode{namingMode}
 , m_splitSizeInBytes{splitSizeInBytes}
 , m_deleteOldAtInBytes{deleteOldAtInBytes}
{
	ros::NodeHandle nh;
	m_freeSpaceTimer = nh.createSteadyTimer(ros::WallDuration(5.0),
		boost::bind(&BagWriter::checkFreeSpace, this)
	);
	checkFreeSpace();
        m_deleteOldBagTimer = nh.createSteadyTimer(ros::WallDuration(5.0),
                boost::bind(&BagWriter::checkOldBagSpace, this)
        );
        checkOldBagSpace();

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
        m_cleanup_thread = std::thread{std::bind(&BagWriter::run_cleanup, this)};
}

BagWriter::~BagWriter()
{
	m_shouldShutdown = true;
	m_queue.shutdown();
        m_cleanup_condition_variable.notify_one();
	m_thread.join();
        m_cleanup_thread.join();
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

				if(msg->topicData->id >= m_messageCounts.size())
					m_messageCounts.resize(msg->topicData->id+1, 0);

				m_messageCounts[msg->topicData->id]++;
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

                if(m_sizeInBytes >= m_splitSizeInBytes)
		{
			m_isReopeningBag = true;
			stop();
			start();
			m_isReopeningBag = false;
		}
	}
}

void BagWriter::start()
{
	std::unique_lock<std::mutex> lock(m_mutex);

	if(m_running)
		return;

	m_messageCounts.clear();

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

		m_expandedFilename = filename;

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

	auto path = fs::absolute(fs::path(m_filename)).parent_path();
	auto space = fs::space(path);

	m_freeSpace = space.available;
}

void BagWriter::checkOldBagSpace()
{
        namespace fs = boost::filesystem;
        std::vector<fs::path> bagFiles = getBagFilesInCurrentFolder(m_filename);
        m_directorySizeInBytes = getTotalSizeInBytes(bagFiles);

        if(m_directorySizeInBytes > m_deleteOldAtInBytes)
        {
                std::unique_lock<std::mutex> lock(m_cleanup_mutex);
                m_cleanup_necessary = true;
                m_cleanup_condition_variable.notify_one();
        }
}

void BagWriter::run_cleanup()
{
        namespace fs = boost::filesystem;
        while(!m_shouldShutdown)
        {
                std::unique_lock<std::mutex> lock(m_cleanup_mutex);
                m_cleanup_condition_variable.wait(lock, [&]{ return m_cleanup_necessary || m_shouldShutdown;});
                if ( m_shouldShutdown )
                        break;
                //get current filename to prevent its deletion
                fs::path currentPath = fs::path(bagfileName());

                if(m_directorySizeInBytes > m_deleteOldAtInBytes)
                {
                        std::vector<fs::path> bagFiles = getBagFilesInCurrentFolder(m_filename);
                        std::vector<fs::path> timeSortedBagFiles = sortFilesByTime (bagFiles);

                        // explicit new computation, since there might have been some changes in the mean time.
                        std::uint64_t directorySizeInBytes = getTotalSizeInBytes(bagFiles);
                        for(auto& entry : timeSortedBagFiles)
                        {
                                // do not delete current one !
                                if ( fs::equivalent(entry, currentPath) )
                                        continue;
                                std::uint64_t bagSize = fs::file_size(entry);
                                directorySizeInBytes -= bagSize;
                                bool smallEnough = directorySizeInBytes < m_deleteOldAtInBytes;

                                ROSFMT_INFO("Bag directory requires too much space. Removing old bag file: {}", entry.c_str());
                                fs::remove(entry);

                                if(smallEnough)
                                        break;
                        }
                }
                m_cleanup_necessary = false;
        }
}

}
