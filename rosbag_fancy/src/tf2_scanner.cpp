// Scans multiple bag files for tf2 messages and aggregates them
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include "tf2_scanner.h"

#include <tf2_msgs/TFMessage.h>

#include <mutex>
#include <thread>
#include <condition_variable>

#include "bag_view.h"

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
		if(!m_msgs.empty() && m_msgs[m_cacheIdx].stamp > time)
		{
			m_cacheIdx = 0;
			newMessage = true;
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

	mutable unsigned int m_cacheIdx = 0;
};

TF2Scanner::TF2Scanner(const std::vector<BagReader*>& readers)
 : m_d{std::make_unique<Private>(readers)}
{
}

const tf2_msgs::TFMessage* TF2Scanner::fetchUpdate(const ros::Time& time)
{
	return m_d->fetchUpdate(time);
}

}
