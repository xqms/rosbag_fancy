// Scans multiple bag files for tf2 messages and aggregates them
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include "tf2_scanner.h"

#include <tf2_msgs/TFMessage.h>

#include <mutex>
#include <thread>
#include <condition_variable>

#include <rosbag/bag.h>

#include "bag_view.h"

#include "doctest.h"

namespace rosbag_fancy
{

namespace
{
	bool tf2StaticPredicate(const BagReader::Connection& con)
	{
		return con.topicInBag == "/tf_static" && con.type == "tf2_msgs/TFMessage";
	}
}

class TF2Scanner::Private
{
public:
	explicit Private(const std::vector<BagReader*>& readers)
	{
		for(auto& reader : readers)
			m_view.addBag(reader, tf2StaticPredicate);

		m_thread = std::thread{[&](){ process(); }};
	}

	~Private()
	{
		m_shouldExit = true;
		m_thread.join();
	}

	const tf2_msgs::TFMessage* fetchUpdate(const ros::Time& time)
	{
		std::unique_lock lock{m_mutex};

		// Wait until the message list is populated until the point we are interested in
		m_cond.wait(lock, [&]() -> bool {
			return m_scanningFinished || (!m_msgs.empty() && m_scannedTime >= time);
		});

		// No messages?
		if(m_msgs.empty())
			return {};

		bool newMessage = false;

		// If we made a skip back in time, reset the cache idx
		if(m_msgs[m_cacheIdx].stamp > time)
		{
			m_cacheIdx = 0;
			newMessage = true;

			// If time is earlier than the first entry, we have no transforms.
			if(m_msgs[m_cacheIdx].stamp > time)
				return &m_emptyMessage;
		}

		// Advance
		while(m_cacheIdx < m_msgs.size()-1 && m_msgs[m_cacheIdx+1].stamp < time)
		{
			m_cacheIdx++;
			newMessage = true;
		}

		if(newMessage)
			return &m_msgs[m_cacheIdx].msg;
		else
			return {};
	}

private:
	struct Msg
	{
		ros::Time stamp;
		tf2_msgs::TFMessage msg;
	};

	void process()
	{
		std::unordered_map<std::string, geometry_msgs::TransformStamped> transforms;

		for(auto& pmsg : m_view)
		{
			if(m_shouldExit)
				return;

			auto msg = pmsg.msg->instantiate<tf2_msgs::TFMessage>();
			if(!msg)
				continue;

			bool changed = false;
			for(auto& transform : msg->transforms)
			{
				auto [it, inserted] = transforms.try_emplace(transform.child_frame_id, transform);
				if(inserted)
					changed = true;
				else
				{
					if(it->second != transform)
					{
						it->second = transform;
						changed = true;
					}
				}
			}

			if(changed)
			{
				std::unique_lock lock{m_mutex};

				auto& entry = m_msgs.emplace_back();
				entry.stamp = pmsg.msg->stamp;
				entry.msg.transforms.resize(transforms.size());
				for(auto& [_, transform] : transforms)
					entry.msg.transforms.push_back(transform);

				m_scannedTime = pmsg.msg->stamp;

				m_cond.notify_all();
			}
		}

		{
			std::unique_lock lock{m_mutex};
			m_scannedTime = m_view.endTime();
			m_scanningFinished = true;
			m_cond.notify_all();
		}
	}

	BagView m_view;

	std::atomic_bool m_shouldExit = false;

	std::mutex m_mutex;
	std::vector<Msg> m_msgs;
	ros::Time m_scannedTime;
	bool m_scanningFinished = false;
	std::condition_variable m_cond;

	std::thread m_thread;

	unsigned int m_cacheIdx = 0;

	tf2_msgs::TFMessage m_emptyMessage{};
};

TF2Scanner::TF2Scanner(const std::vector<BagReader*>& readers)
 : m_d{std::make_unique<Private>(readers)}
{
}

TF2Scanner::~TF2Scanner()
{
}

const tf2_msgs::TFMessage* TF2Scanner::fetchUpdate(const ros::Time& time)
{
	return m_d->fetchUpdate(time);
}


// TESTS

TEST_CASE("TF2Scanner")
{
	// Generate a bag file
	// NOTE: This generates a bag file which is slightly different, since
	// the latch=true header is not set for tf_static messages.
	// For the purpose of testing the above, this is enough, however.
	char bagfileName[] = "/tmp/rosbag_fancy_test_XXXXXX";
	{
		int fd = mkstemp(bagfileName);
		REQUIRE(fd >= 0);
		close(fd);

		rosbag::Bag bag{bagfileName, rosbag::BagMode::Write};

		{
			std_msgs::Header msg;
			msg.frame_id = "a";
			bag.write("/topicA", ros::Time(1000, 0), msg);
		}
		{
			std_msgs::Header msg;
			msg.frame_id = "b";
			bag.write("/topicB", ros::Time(1001, 0), msg);
		}
		{
			tf2_msgs::TFMessage msg;
			msg.transforms.resize(2);
			msg.transforms[0].header.frame_id = "base_link";
			msg.transforms[0].child_frame_id = "arm_link";
			msg.transforms[0].transform.translation.x = 1.0;
			msg.transforms[1].header.frame_id = "base_link";
			msg.transforms[1].child_frame_id = "leg_link";
			msg.transforms[1].transform.translation.x = 4.0;
			bag.write("/tf_static", ros::Time(1002, 0), msg);
		}
		{
			tf2_msgs::TFMessage msg;
			msg.transforms.resize(1);
			msg.transforms[0].header.frame_id = "base_link";
			msg.transforms[0].child_frame_id = "arm_link";
			msg.transforms[0].transform.translation.x = 2.0;
			bag.write("/tf_static", ros::Time(1010, 0), msg);
		}

		bag.close();
	}

	BagReader reader{bagfileName};

	TF2Scanner scanner{{&reader}};

	{
		auto msg = scanner.fetchUpdate(ros::Time(0));
		REQUIRE(msg);
		CHECK(msg->transforms.size() == 0);
	}
	{
		auto msg = scanner.fetchUpdate(ros::Time(1005));
		REQUIRE(msg);
		REQUIRE(msg->transforms.size() == 2);

		auto it = std::find_if(msg->transforms.begin(), msg->transforms.end(), [&](auto& trans){ return trans.child_frame_id == "arm_link"; });
		REQUIRE(it != msg->transforms.end());
		CHECK(it->transform.translation.x == doctest::Approx(1.0));
	}
	{
		auto msg = scanner.fetchUpdate(ros::Time(1006));
		REQUIRE(!msg);
	}
	{
		auto msg = scanner.fetchUpdate(ros::Time(1012));
		REQUIRE(msg);
		REQUIRE(msg->transforms.size() == 2);

		auto it = std::find_if(msg->transforms.begin(), msg->transforms.end(), [&](auto& trans){ return trans.child_frame_id == "arm_link"; });
		REQUIRE(it != msg->transforms.end());
		CHECK(it->transform.translation.x == doctest::Approx(2.0));
	}

	unlink(bagfileName);
}

}
