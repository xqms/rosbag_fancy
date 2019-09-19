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

std::string timeToString(const ros::Time& cur_ros_t)
{
	std::time_t cur_t = cur_ros_t.sec;
	std::stringstream ss;
	ss << std::put_time(std::localtime(&cur_t), "%Y-%m-%d-%H-%M-%S");
	return ss.str();
}

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
			("prefix,p", po::value<std::string>()->default_value("bag"), "Prefix for output bag file. The prefix is extended with a timestamp.")
			("output,o", po::value<std::string>(), "Output bag file (overrides --prefix)")
			("topic", po::value<std::vector<std::string>>()->required(), "Topics to record")
			("queue-size", po::value<std::uint64_t>()->default_value(500ULL*1024*1024), "Queue size in bytes")
		;

		po::positional_options_description p;
		p.add("topic", -1);

		auto usage = [&](){
			std::cout << "Usage: rosbag_fancy [options] -o <bag file> <topics...>\n\n";
			std::cout << desc << "\n\n";
			std::cout << "Topics may be annotated with a rate limit in Hz, e.g.:\n";
			std::cout << "  rosbag_fancy /camera/image_raw=10.0\n";
			std::cout << "\n";
		};

		try
		{
			po::store(
				po::command_line_parser(argc, argv).options(desc).positional(p).run(),
				vm
			);

			if(vm.count("help"))
			{
				usage();
				return 0;
			}

			po::notify(vm);
		}
		catch(po::error& e)
		{
			std::cerr << "Could not parse arguments: " << e.what() << "\n\n";
			usage();
			return 1;
		}
	}

	ros::NodeHandle nh{"~"};

	std::vector<std::string> topics = vm["topic"].as<std::vector<std::string>>();
	std::sort(topics.begin(), topics.end());

	TopicManager topicManager;
	for(auto& topicSpec : topics)
	{
		std::string name = topicSpec;
		float rateLimit = 0.0f;

		auto sepIdx = topicSpec.find('=');

		if(sepIdx != std::string::npos)
		{
			name = topicSpec.substr(0, sepIdx);

			try
			{
				rateLimit = boost::lexical_cast<float>(topicSpec.substr(sepIdx+1));
			}
			catch(boost::bad_lexical_cast&)
			{
				std::cerr << "Bad topic spec: '" << topicSpec << "'\n";
				return 1;
			}
		}

		topicManager.addTopic(name, rateLimit);
	}

	MessageQueue queue{vm["queue-size"].as<std::uint64_t>()};
	BagWriter writer{queue};

	// Figure out the output file name
	std::string bagName = "";
	if(vm.count("output"))
		bagName = vm["output"].as<std::string>();
	else
	{
		std::string prefix = vm["prefix"].as<std::string>();

		bagName = fmt::format("{}_{}.bag",
			vm["prefix"].as<std::string>(),
			timeToString(ros::Time::now())
		);
	}
	ROSFMT_INFO("Bagfile name: {}", bagName);

	try
	{
		writer.start(bagName);
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
