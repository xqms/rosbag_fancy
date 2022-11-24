// Play command
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <boost/program_options.hpp>

#include <iostream>
#include <rosfmt/full.h>
#include <fmt/chrono.h>

#include <chrono>

#include <ros/node_handle.h>
#include <ros/publisher.h>

#include <rosgraph_msgs/Clock.h>

#include "bag_reader.h"
#include "bag_view.h"
#include "topic_manager.h"
#include "ui.h"

namespace po = boost::program_options;
using namespace rosbag_fancy;

int play(const std::vector<std::string>& options)
{
	po::variables_map vm;

	// Handle CLI arguments
	{
		po::options_description desc("Options");
		desc.add_options()
			("help", "Display this help message")
			("clock", "Publish clock (requires use_sim_time)")
		;

		po::options_description hidden("Hidden");
		hidden.add_options()
			("input", po::value<std::vector<std::string>>()->required(), "Input file")
		;

		po::options_description all("All");
		all.add(desc).add(hidden);

		po::positional_options_description p;
		p.add("input", -1);

		auto usage = [&](){
			std::cout << "Usage: rosbag_fancy play [options] <bag file(s)>\n\n";
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

	bool publishClock = vm.count("clock");

	ros::Publisher pub_clock;
	ros::SteadyTime lastClockPublishTime;

	if(publishClock)
		pub_clock = ros::NodeHandle{}.advertise<rosgraph_msgs::Clock>("/clock", 1, true);

	struct Bag
	{
		explicit Bag(const std::string& filename)
		 : reader{filename}
		{}

		BagReader reader;
		std::unordered_map<int, ros::Publisher> publishers;
	};

	std::vector<Bag> bags;

	TopicManager topicManager;
	std::map<std::string, int> topicMap;

	ros::NodeHandle nh;

	ros::Time startTime;
	ros::Time endTime;

	for(auto& filename : vm["input"].as<std::vector<std::string>>())
	{
		auto& bag = bags.emplace_back(filename);

		if(startTime == ros::Time(0) || bag.reader.startTime() < startTime)
			startTime = bag.reader.startTime();

		if(endTime == ros::Time(0) || bag.reader.endTime() > endTime)
			endTime = bag.reader.endTime();

		for(auto& [id, con] : bag.reader.connections())
		{
			ros::AdvertiseOptions opts;
			opts.datatype = con.type;
			opts.md5sum = con.md5sum;
			opts.message_definition = con.msgDef;
			opts.topic = con.topicInBag;
			opts.latch = con.latching;
			opts.queue_size = 100;
			opts.has_header = false; // FIXME: Is this correct?

			bag.publishers[id] = nh.advertise(opts);

			auto it = topicMap.find(con.topicInBag);
			if(it == topicMap.end())
			{
				topicMap[con.topicInBag] = topicManager.topics().size();
				topicManager.addTopic(con.topicInBag);
			}
		}
	}

	BagView view;
	for(auto& bag : bags)
		view.addBag(&bag.reader);

	ros::WallDuration(0.2).sleep();

	ros::Time bagRefTime = startTime;
	ros::SteadyTime wallRefTime = ros::SteadyTime::now();
	ros::Time currentTime = bagRefTime;

	std::unique_ptr<PlaybackUI> ui;

	bool paused = false;

	BagView::Iterator it = view.begin();

	if(!vm.count("no-ui"))
	{
		ui.reset(new PlaybackUI{topicManager, startTime, endTime});

		ui->seekForwardRequested.connect([&](){
			currentTime += ros::Duration{5.0};
			bagRefTime += ros::Duration{5.0};

			it = view.findTime(currentTime);
		});
		ui->seekBackwardRequested.connect([&](){
			currentTime -= ros::Duration{5.0};
			bagRefTime -= ros::Duration{5.0};

			it = view.findTime(currentTime);
		});
		ui->pauseRequested.connect([&](){
			paused = !paused;
			bagRefTime = currentTime;
			wallRefTime = ros::SteadyTime::now();
		});
	}

	while(ros::ok())
	{
		if(paused)
			ros::WallDuration{0.1}.sleep();
		else
		{
			if(it == view.end())
				break;

			auto& msg = *it->msg;

			if(msg.stamp < bagRefTime)
				continue;

			ros::SteadyTime wallStamp = wallRefTime + ros::WallDuration().fromNSec((msg.stamp - bagRefTime).toNSec());
			ros::SteadyTime::sleepUntil(wallStamp);

			currentTime = msg.stamp;
			ui->setPositionInBag(msg.stamp);

			bags[it->bagIndex].publishers[msg.connection->id].publish(msg);

			auto& topic = topicManager.topics()[topicMap[msg.connection->topicInBag]];
			topic.notifyMessage(msg.size());

			if(publishClock && wallStamp - lastClockPublishTime > ros::WallDuration{0.001})
			{
				rosgraph_msgs::Clock msg;
				msg.clock = currentTime;
				pub_clock.publish(msg);
				lastClockPublishTime = wallStamp;
			}

			++it;
		}

		ros::spinOnce();

		// Handle key input
		if(ui)
		{
			fd_set fds{};
			FD_ZERO(&fds);
			FD_SET(STDIN_FILENO, &fds);
			timeval timeout{};
			int ret = select(STDIN_FILENO+1, &fds, nullptr, nullptr, &timeout);
			if(ret < 0)
			{
				if(errno != EAGAIN && errno != EINTR)
					throw std::runtime_error{fmt::format("select() error: {}", strerror(errno))};
			}
			else if(ret != 0)
				ui->handleInput();
		}
	}

	return 0;
}
