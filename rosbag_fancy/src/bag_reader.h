// Fast native bag reader
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef ROSBAG_FANCY_BAGREADER_H
#define ROSBAG_FANCY_BAGREADER_H

#include <map>
#include <memory>
#include <string>

#include <ros/time.h>
#include <ros/serialization.h>

#include <boost/make_shared.hpp>

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

	struct Message
	{
		ros::Time stamp;
		Connection* connection;

		template<class M>
		boost::shared_ptr<M> instantiate() const
		{
			if(ros::message_traits::DataType<M>::value() != connection->type)
				return {};
			if(ros::message_traits::MD5Sum<M>::value() != connection->md5sum)
				return {};

			namespace ser = ros::serialization;
			ser::IStream stream(const_cast<uint8_t*>(m_data), m_size);

			auto ret = boost::make_shared<M>();
			ser::deserialize(stream, *ret);
			return ret;
		}

	private:
		const uint8_t* m_data;
		std::size_t m_size;
	};

	struct Iterator
	{
		using iterator_category = std::input_iterator_tag;
		using value_type        = Message;
		using reference         = Message&;
		using pointer           = Message*;

		Iterator(const Iterator&) = default;
		Iterator& operator=(const Iterator&) = default;

		reference operator*() { return m_msg; }
		pointer operator->() { return &m_msg; }

		Iterator& operator++() { *this = m_reader->next(*this); return *this; }
		Iterator operator++(int) { Iterator tmp = *this; ++(*this); return tmp; }

		friend bool operator== (const Iterator& a, const Iterator& b) { return a.m_chunk == b.m_chunk && a.m_offset == b.m_offset; };
		friend bool operator!= (const Iterator& a, const Iterator& b) { return a.m_chunk != b.m_chunk || a.m_chunk != b.m_chunk; };

	private:
		explicit Iterator(BagReader* reader, int chunk, std::size_t offset)
		 : m_reader{reader}
		 , m_chunk{chunk}
		 , m_offset{offset}
		{
			m_msg = reader->readMessage(chunk, offset);
		}

		BagReader* m_reader;
		int m_chunk;
		std::size_t m_offset;
		Message m_msg;
	};

	explicit BagReader(const std::string& filename);
	~BagReader();

	BagReader(const BagReader&) = delete;
	BagReader& operator=(const BagReader&) = delete;

	const ConnectionMap& connections();

	ros::Time startTime() const;
	ros::Time endTime() const;

	std::size_t size() const;

	Iterator begin() const;
	Iterator end() const;

private:
	Message readMessage(int chunk, std::size_t offset);
	Iterator next(const Iterator& current);

	class Private;
	std::unique_ptr<Private> m_d;
};

}

#endif
