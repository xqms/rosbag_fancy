// Fancy rosbag terminal UI
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <boost/program_options.hpp>

#include <ros/init.h>

#include <rosfmt/full.h>

namespace po = boost::program_options;

// Implemented in cmd_*.cpp
int record(const std::vector<std::string>& options);
int info(const std::vector<std::string>& options);
int play(const std::vector<std::string>& options);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rosbag_fancy", ros::init_options::AnonymousName);

	auto usage = [](std::FILE* f){
		fmt::print(f,
			"Usage: rosbag_fancy <command> [args]\n\n"
			"Available commands:\n"
			"  record: Record a bagfile\n"
			"  info: Display information about a bagfile\n"
			"  play: Play bagfile\n"
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
	else if(cmd == "play")
		return play(arguments);
	else
	{
		fmt::print(stderr, "Unknown command {}, see --help\n", cmd);
		return 1;
	}

	return 0;
}

