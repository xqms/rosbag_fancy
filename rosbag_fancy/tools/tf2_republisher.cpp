// Consolidate & republish static transforms from a bag file
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <ros/ros.h>

#include <tf2_msgs/TFMessage.h>

#include <tf2_ros/static_transform_broadcaster.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tf2_republisher");

	ros::NodeHandle nh{"~"};

	tf2_ros::StaticTransformBroadcaster pub_tf;

	using Msg = tf2_msgs::TFMessage;
	using Event = ros::MessageEvent<Msg>;

	ros::Subscriber sub = nh.subscribe<Msg>("/tf_static", 100, boost::function<void(const Event&)>(
		[&](const Event& event)
		{
			// Did we publish this message?
			if(event.getPublisherName() == ros::this_node::getName())
				return;

			pub_tf.sendTransform(event.getMessage()->transforms);
		}
	));

	ros::spin();

	return 0;
}
