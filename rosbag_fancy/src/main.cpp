// Fancy rosbag terminal UI
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <boost/program_options.hpp>

#include <chrono>
#include <regex>

#include <ros/ros.h>
#include <rosfmt/rosfmt.h>
#include <fmt/chrono.h>
#include <fmt/ranges.h>

#include <rosbag/bag.h>

#include <std_srvs/Trigger.h>

#include <rosbag_fancy_msgs/Status.h>

#include "topic_manager.h"
#include "message_queue.h"
#include "bag_reader.h"
#include "bag_writer.h"
#include "terminal.h"
#include "topic_subscriber.h"
#include "ui.h"

namespace po = boost::program_options;
using namespace rosbag_fancy;
using namespace rosbag_fancy_msgs;

namespace
{
	std::string memoryToString(uint64_t memory)
	{
		if(memory < static_cast<uint64_t>(1<<10))
			return fmt::format("{}.0   B", memory);
		else if(memory < static_cast<uint64_t>(1<<20))
			return fmt::format("{:.1f} KiB", static_cast<double>(memory) / static_cast<uint64_t>(1<<10));
		else if(memory < static_cast<uint64_t>(1<<30))
			return fmt::format("{:.1f} MiB", static_cast<double>(memory) / static_cast<uint64_t>(1<<20));
		else if(memory < static_cast<uint64_t>(1ull<<40))
			return fmt::format("{:.1f} GiB", static_cast<double>(memory) / static_cast<uint64_t>(1ull<<30));
		else
			return fmt::format("{:.1f} TiB", static_cast<double>(memory) / static_cast<uint64_t>(1ull<<40));
	}

	uint64_t stringToMemory(std::string humanSize)
	{
		// format should be "10MB" or "3 GB" or "10 B" or "10GiB"
		static std::regex memoryRegex{R"EOS((\d+)\s*(K|M|G|T|E|)i?B?)EOS"};

		std::smatch match;
		if(!std::regex_match(humanSize, match, memoryRegex))
		{
			fmt::print(stderr, "Could not parse memory string '{}' - it should be something like '120B' or '10GB'\n",
				humanSize
			);
			std::exit(1);
		}

		std::string number = match[1].str();
		std::string unit = match[2].str();

		std::uint64_t multiplier = [&]() -> std::uint64_t {
			if(unit.empty())
				return 1;

			switch(unit[0])
			{
				case 'K': return 1ULL << 10;
				case 'M': return 1ULL << 20;
				case 'G': return 1ULL << 30;
				case 'T': return 1ULL << 40;
				case 'E': return 1ULL << 50;
			}

			throw std::logic_error{"I got regexes wrong :("};
		}();

		return std::stoull(humanSize) * multiplier;
	}
}

int record(const std::vector<std::string>& options)
{
	po::variables_map vm;

	// Handle CLI arguments
	{
		po::options_description desc("Options");
		desc.add_options()
			("help", "Display this help message")
			("prefix,p", po::value<std::string>()->default_value("bag"), "Prefix for output bag file. The prefix is extended with a timestamp.")
			("output,o", po::value<std::string>()->value_name("FILE"), "Output bag file (overrides --prefix)")
			("topic", po::value<std::vector<std::string>>()->required(), "Topics to record")
			("queue-size", po::value<std::string>()->value_name("SIZE")->default_value("500MB"), "Queue size")
			("delete-old-at", po::value<std::string>()->value_name("SIZE"), "Delete old bags at given size, e.g. 100GB or 1TB")
			("split-bag-size", po::value<std::string>()->value_name("SIZE"), "Bag size for splitting, e.g. 1GB")
			("paused", "Start paused")
			("no-ui", "Disable terminal UI")
			("udp", "Subscribe using UDP transport")
			("bz2", "Enable BZ2 compression")
			("lz4", "Enable LZ2 compression")
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

		int flags = 0;
		if(vm.count("udp"))
			flags |= static_cast<int>(Topic::Flag::UDP);

		topicManager.addTopic(name, rateLimit, flags);
	}

	std::uint64_t queueSize = stringToMemory(vm["queue-size"].as<std::string>());
	MessageQueue queue{queueSize};

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

	std::uint64_t splitBagSizeInBytes = 0;
	if(vm.count("split-bag-size"))
	{
		splitBagSizeInBytes = stringToMemory(vm["split-bag-size"].as<std::string>());
	}

	std::uint64_t deleteOldAtInBytes = 0;
	if(vm.count("delete-old-at"))
	{
		deleteOldAtInBytes = stringToMemory(vm["delete-old-at"].as<std::string>());
		if(splitBagSizeInBytes != 0 && deleteOldAtInBytes < splitBagSizeInBytes)
		{
			ROSFMT_WARN("Chosen split-bag-size is larger than delete-old-at size!");
		}
	}

	if(!ros::Time::isValid())
	{
		ROSFMT_INFO("Waiting for ros::Time to become valid...");
		ros::Time::waitForValid();
	}

	BagWriter writer{queue, bagName, namingMode, splitBagSizeInBytes, deleteOldAtInBytes};

	if(vm.count("bz2") && vm.count("lz4"))
	{
		fmt::print(stderr, "Options --bz2 and --lz4 are mutually exclusive\n");
		return 1;
	}

	if(vm.count("bz2"))
		writer.setCompression(rosbag::compression::BZ2);
	if(vm.count("lz4"))
		writer.setCompression(rosbag::compression::LZ4);

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
	ros::ServiceServer srv_start = nh.advertiseService("start", boost::function<bool(std_srvs::TriggerRequest&, std_srvs::TriggerResponse&)>([&](auto&, auto& resp){
		resp.success = start();
		return true;
	}));
	ros::ServiceServer srv_stop = nh.advertiseService("stop", boost::function<bool(std_srvs::TriggerRequest&, std_srvs::TriggerResponse&)>([&](auto&, auto& resp){
		writer.stop();
		resp.success = true;
		return true;
	}));

	// Status publisher
	ros::Publisher pub_status = nh.advertise<Status>("status", 1);
	ros::SteadyTimer timer_status = nh.createSteadyTimer(ros::WallDuration(0.1), boost::function<void(const ros::SteadyTimerEvent&)>([&](auto&){
		ros::WallTime now = ros::WallTime::now();

		StatusPtr msg = boost::make_shared<Status>();
		msg->header.stamp = ros::Time::now();

		msg->status = writer.running() ? Status::STATUS_RUNNING : Status::STATUS_PAUSED;

		msg->bagfile = writer.bagfileName();

		msg->bytes = writer.sizeInBytes();
		msg->free_bytes = writer.freeSpace();
		msg->bandwidth = 0;

		auto& counts = writer.messageCounts();

		for(auto& topic : topicManager.topics())
		{
			msg->topics.emplace_back();
			auto& topicMsg = msg->topics.back();

			msg->bandwidth += topic.bandwidth;

			topicMsg.name = topic.name;
			topicMsg.publishers = topic.numPublishers;
			topicMsg.bandwidth = topic.bandwidth;
			topicMsg.bytes = topic.totalBytes;
			topicMsg.messages = topic.totalMessages;

			if(topic.id < counts.size())
				topicMsg.messages_in_current_bag = counts[topic.id];

			topicMsg.rate = topic.messageRateAt(now);
		}

		pub_status.publish(msg);
	}));

	// Start recording if --paused is not given
	if(vm.count("paused") == 0)
	{
		if(!start())
			return 1;
	}

	TopicSubscriber subscriber{topicManager, queue};

	std::unique_ptr<UI> ui;

	if(!vm.count("no-ui"))
		ui.reset(new UI{topicManager, queue, writer, UI::Mode::Recording});

	ros::spin();

	return 0;
}

int info(const std::vector<std::string>& options)
{
	po::variables_map vm;

	// Handle CLI arguments
	{
		po::options_description desc("Options");
		desc.add_options()
			("help", "Display this help message")
		;

		po::options_description hidden("Hidden");
		hidden.add_options()
			("input", po::value<std::string>()->required(), "Input file")
		;

		po::options_description all("All");
		all.add(desc).add(hidden);

		po::positional_options_description p;
		p.add("input", 1);

		auto usage = [&](){
			std::cout << "Usage: rosbag_fancy info [options] <bag file>\n\n";
			std::cout << desc << "\n\n";
		};

		try
		{
			po::store(
				po::command_line_parser(options).options(all).positional(p).run(),
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

	std::string filename = vm["input"].as<std::string>();

	BagReader reader(filename);

	std::map<std::string, std::vector<std::uint32_t>> connectionsForTopic;
	for(auto& c : reader.connections())
		connectionsForTopic[c.second.topicInBag].push_back(c.second.id);

	// A lot of std::chrono magic to get local/UTC time
	std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> startTimeC(std::chrono::nanoseconds(reader.startTime().toNSec()));
	std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> endTimeC(std::chrono::nanoseconds(reader.endTime().toNSec()));

	std::chrono::seconds startTimeS = std::chrono::duration_cast<std::chrono::seconds>(startTimeC.time_since_epoch());
	std::time_t startTimeSC(startTimeS.count());
	std::tm startTimeB;
	std::tm startTimeBUTC;
	localtime_r(&startTimeSC, &startTimeB);
	gmtime_r(&startTimeSC, &startTimeBUTC);

	std::chrono::seconds endTimeS = std::chrono::duration_cast<std::chrono::seconds>(endTimeC.time_since_epoch());
	std::time_t endTimeSC(endTimeS.count());
	std::tm endTimeB;
	std::tm endTimeBUTC;
	localtime_r(&endTimeSC, &endTimeB);
	gmtime_r(&endTimeSC, &endTimeBUTC);

	std::chrono::nanoseconds duration{(reader.endTime() - reader.startTime()).toNSec()};

	fmt::print("File:           {}\n", filename);
	fmt::print("Start time:     {:%Y-%m-%d %H:%M:%S} ({}) / {:%Y-%m-%d %H:%M:%S} (UTC)\n", startTimeB, daylight ? tzname[1] : tzname[0], startTimeBUTC);
	fmt::print("End time:       {:%Y-%m-%d %H:%M:%S} ({}) / {:%Y-%m-%d %H:%M:%S} (UTC)\n", endTimeB, daylight ? tzname[1] : tzname[0], endTimeBUTC);
	fmt::print("Duration:       {:%H:%M:%S} ({:.2f}s)\n", duration, (reader.endTime() - reader.startTime()).toSec());
	fmt::print("Size:           {}\n", memoryToString(reader.size()));

	fmt::print("Types:\n");
	std::map<std::string, std::set<std::string>> types;
	for(auto& con : reader.connections())
		types[con.second.type].insert(con.second.md5sum);

	std::size_t maxTypeLength = 0;
	for(auto& t : types)
		maxTypeLength = std::max(maxTypeLength, t.first.size());

	for(auto& t : types)
		fmt::print(" - {:{}} {}\n", t.first, maxTypeLength, t.second);

	fmt::print("Topics:\n");
	std::size_t maxTopicLength = 0;
	for(auto& topicPair : connectionsForTopic)
		maxTopicLength = std::max(maxTopicLength, topicPair.first.size());

	for(auto& topicPair : connectionsForTopic)
	{
		std::uint64_t count = 0;
		std::set<std::string> types;
		for(auto& conID : topicPair.second)
		{
			auto it = reader.connections().find(conID);
			auto& conn = it->second;

			types.insert(conn.type);

			count += conn.totalCount;
		}

		fmt::print(" - {:{}} {:8d} msgs: ",
			topicPair.first, maxTopicLength, count
		);

		if(types.size() == 1)
			fmt::print("{:10}", *types.begin());
		else
			fmt::print("{:10}", types);

		if(topicPair.second.size() > 1)
		{
			fmt::print(" ({} connections)",
				topicPair.second.size()
			);
		}

		fmt::print("\n");
	}

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
			"  info: Display information about a bagfile\n"
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
	else if(cmd == "info")
		return info(arguments);
	else
	{
		fmt::print(stderr, "Unknown command {}, see --help\n", cmd);
		return 1;
	}

	return 0;
}

