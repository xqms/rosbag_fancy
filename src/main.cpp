// Fancy rosbag terminal UI
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <boost/program_options.hpp>

#include <ros/ros.h>
#include <rosfmt/rosfmt.h>

#include <rosbag/bag.h>

#include "topic_manager.h"
#include "message_queue.h"
#include "bag_writer.h"
#include "terminal.h"
#include "topic_subscriber.h"
#include "ui.h"

namespace po = boost::program_options;

int main(int argc, char** argv)
{
	using namespace rosbag_fancy;

	ros::init(argc, argv, "rosbag_fancy", ros::init_options::AnonymousName);

	po::variables_map vm;

	// Handle CLI arguments
	{
		po::options_description desc("Options");
		desc.add_options()
			("help", "Display this help message")
			("output", po::value<std::string>()->required(), "Output bag file")
			("topic", po::value<std::vector<std::string>>()->required(), "Topics to record")
			("queue-size", po::value<std::uint64_t>()->default_value(500ULL*1024*1024), "Queue size in bytes")
		;

		po::store(po::parse_command_line(argc, argv, desc), vm);

		if(vm.count("help"))
		{
			std::cout << desc << "\n";
			return 0;
		}

		po::notify(vm);
	}

	ros::NodeHandle nh{"~"};

	std::vector<std::string> topics = vm["topic"].as<std::vector<std::string>>();
	std::sort(topics.begin(), topics.end());

	TopicManager topicManager;
	for(auto& topicName : topics)
	{
		topicManager.addTopic(topicName);
	}

	MessageQueue queue{vm["queue-size"].as<std::uint64_t>()};
	BagWriter writer{queue};

	try
	{
		writer.start(vm["output"].as<std::string>());
	}
	catch(rosbag::BagException& e)
	{
		ROSFMT_ERROR("Could not open output bag file: %s", e.what());
	}

	TopicSubscriber subscriber{topicManager, queue};

	UI ui{topicManager, queue, writer, UI::Mode::Recording};

	ros::spin();

	return 0;
}
