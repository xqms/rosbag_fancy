// Fast native bag reader
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef ROSBAG_FANCY_BAGREADER_H
#define ROSBAG_FANCY_BAGREADER_H

#include <map>
#include <memory>
#include <string>

#include <ros/time.h>

namespace rosbag_fancy
{

class BagReader
{
public:
	class Exception : public std::runtime_error
	{
	public:
		explicit Exception(const std::string& what)
		 : std::runtime_error{what}
		{}
	};

	struct Connection
	{
		std::uint32_t id;
		std::string topicInBag;

		std::string topicAsPublished;
		std::string type;
		std::string md5sum;
		std::string msgDef;
		std::string callerID;
		bool latching = false;

		std::uint64_t totalCount = 0;
	};

	using ConnectionMap = std::map<std::uint32_t, Connection>;


	explicit BagReader(const std::string& filename);
	~BagReader();

	BagReader(const BagReader&) = delete;
	BagReader& operator=(const BagReader&) = delete;

	const ConnectionMap& connections();

	ros::Time startTime() const;
	ros::Time endTime() const;

	std::size_t size() const;

private:
	class Private;

	std::unique_ptr<Private> m_d;
};

}

#endif
