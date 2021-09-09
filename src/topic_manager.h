// Contains the topic configuration & status
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef ROSBAG_FANCY_TOPIC_CONFIG_MANAGER_H
#define ROSBAG_FANCY_TOPIC_CONFIG_MANAGER_H

#include <string>

#include <ros/time.h>
#include <ros/steady_timer.h>

namespace rosbag_fancy
{

struct Topic
{
	enum class Flag
	{
		UDP = (1 << 0)
	};

	explicit Topic(const std::string& name, std::size_t id, float rateLimit = 0.0f, int flags = 0)
	 : name(name), rateLimit(rateLimit), flags(flags)
	 , id(id)
	{
	}

	Topic(const Topic& other) = delete;
	Topic& operator=(const Topic& other) = delete;

	Topic(Topic&& other) = default;
	Topic& operator=(Topic&& other) = default;

	std::string name;
	float rateLimit;
	int flags;

	std::size_t id;

	// Status
	ros::WallTime lastMessageTime;
	ros::Time lastMessageROSTime;
	ros::Time lastMessageReceivedROSTime;
	std::uint64_t messagesInStatsPeriod = 0;
	std::uint64_t bytesInStatsPeriod = 0;

	float throttleAllowance = 0.0f;

	float messageRate = 0.0f;
	float bandwidth = 0.0f;

	std::uint64_t dropCounter = 0;

	std::uint64_t totalMessages = 0;
	std::uint64_t totalMessagesInBag = 0;
	std::uint64_t totalBytes = 0;

	unsigned int numPublishers = 0;

	// The smooth rate estimate is taken from here:
	// https://stackoverflow.com/a/23617678

	float lambdaLast = 0.0f;
	float lambdaSmoothLast = 0.0f;

	static constexpr float HALF_LIFE = 1.0f;
	static constexpr float LOG05 = -0.6931471805599453f; // std::log(0.5f)
	static constexpr float DECAY = -LOG05/HALF_LIFE;
	static const ros::WallTime T0;

	void notifyMessage(std::uint64_t bytes)
	{
		ros::WallTime time = ros::WallTime::now();

		totalMessages++;
		totalBytes += bytes;
		bytesInStatsPeriod += bytes;

		float tDelta = (time - lastMessageTime).toSec();

		float expL = std::exp(-DECAY * tDelta);

		lambdaSmoothLast = DECAY * tDelta * expL * lambdaLast + expL * lambdaSmoothLast;
		lambdaLast = DECAY + expL * lambdaLast;
		lastMessageTime = time;
		lastMessageROSTime = ros::Time::now();
	}

	inline float messageRateAt(const ros::WallTime& time)
	{
		float tDelta = (time - lastMessageTime).toSec();
		float expL = std::exp(-DECAY * tDelta);

		// Bias correction
		float t0Delta = (time - T0).toSec();
		float S = (1.0f + DECAY * t0Delta) * std::exp(-DECAY * t0Delta);

		return (DECAY * tDelta * expL * lambdaLast + expL * lambdaSmoothLast) / (1.0f - S);
	}
};

class TopicManager
{
public:
	TopicManager();

	inline std::vector<Topic>& topics()
	{ return m_topics; }

	void addTopic(const std::string& topic, float rateLimit = 0.0f, int flags = 0);
private:
	void updateStatistics();

	std::vector<Topic> m_topics;
	ros::SteadyTimer m_timer;
};

}

#endif
