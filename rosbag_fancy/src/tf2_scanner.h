// Scans multiple bag files for tf2 messages and aggregates them
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef ROSBAG_FANCY_TF2_SCANNER
#define ROSBAG_FANCY_TF2_SCANNER

#include <vector>
#include <memory>

#include <tf2_msgs/TFMessage.h>

namespace rosbag_fancy
{

class BagReader;

class TF2Scanner
{
public:
	explicit TF2Scanner(const std::vector<BagReader*>& readers);
	~TF2Scanner();

	/**
	 * @brief Fetch next aggregated message
	 *
	 * Returns the next tf_static message that needs to be published.
	 * If there has been no change since the last call, it will return nullptr.
	 **/
	const tf2_msgs::TFMessage* fetchUpdate(const ros::Time& time);

private:
	friend class Cursor;
	class Private;
	std::unique_ptr<Private> m_d;
};

}

#endif
