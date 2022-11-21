// Info command
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <boost/program_options.hpp>

#include <iostream>
#include <rosfmt/full.h>
#include <fmt/chrono.h>

#include <chrono>

#include "bag_reader.h"
#include "mem_str.h"

namespace po = boost::program_options;
using namespace rosbag_fancy;

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
	fmt::print("Size:           {}\n", mem_str::memoryToString(reader.size()));

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
