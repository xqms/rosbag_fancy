// Fancy rosbag terminal UI
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <boost/program_options.hpp>

#include <ros/ros.h>
#include <rosfmt/rosfmt.h>

#include <rosbag/bag.h>

#include <std_srvs/Trigger.h>

#include "topic_manager.h"
#include "message_queue.h"
#include "bag_writer.h"
#include "terminal.h"
#include "topic_subscriber.h"
#include "ui.h"

namespace po = boost::program_options;
using namespace rosbag_fancy;

int record(const std::vector<std::string>& options)
{
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
			std::cout << "Usage: rosbag_fancy record [options] -o <bag file> <topics...>\n\n";
			std::cout << desc << "\n\n";
			std::cout << "Topics may be annotated with a rate limit in Hz, e.g.:\n";
			std::cout << "  rosbag_fancy /camera/image_raw=10.0\n";
			std::cout << "\n";
		};

		try
		{
			po::store(
				po::command_line_parser(options).options(desc).positional(p).run(),
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

	// Figure out the output file name
	auto namingMode = BagWriter::Naming::Verbatim;
	std::string bagName = "";
	if(vm.count("output"))
	{
		bagName = vm["output"].as<std::string>();
		namingMode = BagWriter::Naming::Verbatim;
	}
	else
	{
		bagName = vm["prefix"].as<std::string>();
		namingMode = BagWriter::Naming::AppendTimestamp;
	}

	BagWriter writer{queue, bagName, namingMode};

	auto start = [&](){
		try
		{
			writer.start();
		}
		catch(rosbag::BagException& e)
		{
			ROSFMT_ERROR("Could not open output bag file: {}", e.what());
			return false;
		}
		return true;
	};

	// Start/Stop service calls
	ros::ServiceServer srv_start = nh.advertiseService("start", boost::function<bool(std_srvs::TriggerRequest&, std_srvs::TriggerResponse&)>(
		std::bind(start)
	));
	ros::ServiceServer srv_stop = nh.advertiseService("stop", boost::function<bool(std_srvs::TriggerRequest&, std_srvs::TriggerResponse&)>([&](auto&, auto&){
		writer.stop();
		return true;
	}));

	if(!start())
		return 1;

	TopicSubscriber subscriber{topicManager, queue};

	UI ui{topicManager, queue, writer, UI::Mode::Recording};

	ros::spin();

	return 0;
}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "rosbag_fancy", ros::init_options::AnonymousName);

	auto usage = [](std::FILE* f){
		fmt::print(f,
			"Usage: rosbag_fancy <command> [args]\n\n"
			"Available commands:\n"
			"  record: Record a bagfile\n"
			"\n"
			"See rosbag_fancy <command> --help for command-specific instructions.\n"
			"\n"
		);
	};

	if(argc < 2)
	{
		usage(stderr);
		return 1;
	}

	std::string cmd = std::string(argv[1]);
	std::vector<std::string> arguments(argc - 2);
	std::copy(argv + 2, argv + argc, arguments.begin());

	if(cmd == "-h" || cmd == "--help")
	{
		usage(stdout);
		return 0;
	}

	if(cmd == "record")
		return record(arguments);
	else
	{
		fmt::print(stderr, "Unknown command {}, see --help\n", cmd);
		return 1;
	}

	return 0;
}

