// Fast native bag reader
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef ROSBAG_FANCY_BAGREADER_H
#define ROSBAG_FANCY_BAGREADER_H

#include <map>
#include <memory>
#include <string>

#include <ros/time.h>
#include <ros/serialization.h>

#include <rosbag/stream.h>

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

	struct ConnectionInfo
	{
		std::uint32_t id;
		std::uint32_t msgCount;
	};

	class ChunkIterator;
	class Iterator;

	struct Message
	{
		ros::Time stamp;
		const Connection* connection;

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

		const uint8_t* data() const
		{ return m_data; }

		std::size_t size() const
		{ return m_size; }

	private:
		friend class BagReader::ChunkIterator;
		friend struct ros::serialization::Serializer<rosbag_fancy::BagReader::Message>;

		const uint8_t* m_data = nullptr;
		std::size_t m_size;
	};

	class ChunkIterator
	{
	public:
		using iterator_category = std::input_iterator_tag;
		using value_type        = Message;
		using reference         = const Message&;
		using pointer           = const Message*;

		ChunkIterator(const ChunkIterator&) = default;
		ChunkIterator& operator=(const ChunkIterator&) = default;

		reference operator*() { return m_msg; }
		pointer operator->() { return &m_msg; }

		ChunkIterator& operator++();
		ChunkIterator operator++(int) { ChunkIterator tmp = *this; ++(*this); return tmp; }

		friend bool operator== (const ChunkIterator& a, const ChunkIterator& b) { return a.m_d == b.m_d && a.m_idx == b.m_idx; };
		friend bool operator!= (const ChunkIterator& a, const ChunkIterator& b) { return a.m_d != b.m_d || a.m_idx != b.m_idx; };

	private:
		friend class BagReader::Iterator;

		ChunkIterator() {}
		explicit ChunkIterator(const BagReader* reader, int chunk);

		class Private;
		std::shared_ptr<Private> m_d;
		std::size_t m_idx = 0;
		Message m_msg;
	};

	class Iterator
	{
	public:
		using iterator_category = std::input_iterator_tag;
		using value_type        = Message;
		using reference         = const Message&;
		using pointer           = const Message*;

		Iterator() {}
		Iterator(const Iterator&) = default;
		Iterator& operator=(const Iterator&) = default;

		reference operator*() { return *m_it; }
		pointer operator->() { return &*m_it; }

		Iterator& operator++();
		Iterator operator++(int) { Iterator tmp = *this; ++(*this); return tmp; }

		void advanceWithPredicates(const std::function<bool(const ConnectionInfo&)>& connPredicate, const std::function<bool(const Message&)>& messagePredicate);
		void findNextWithPredicates(const std::function<bool(const ConnectionInfo&)>& connPredicate, const std::function<bool(const Message&)>& messagePredicate);

		friend bool operator== (const Iterator& a, const Iterator& b) { return a.m_chunk == b.m_chunk && a.m_it == b.m_it; };
		friend bool operator!= (const Iterator& a, const Iterator& b) { return a.m_chunk != b.m_chunk || a.m_it != b.m_it; };

		std::vector<ConnectionInfo>& currentChunkConnections() const;
		rosbag::CompressionType currentChunkCompression() const;

		void skipChunk();
		int chunk() const
		{ return m_chunk; }

	private:
		friend class BagReader;

		explicit Iterator(const BagReader* reader, int chunk);

		const BagReader* m_reader;
		int m_chunk = -1;
		ChunkIterator m_it;
	};

	explicit BagReader(const std::string& filename);
	~BagReader();

	BagReader(const BagReader&) = delete;
	BagReader& operator=(const BagReader&) = delete;

	BagReader(BagReader&& other);

	const ConnectionMap& connections() const;

	ros::Time startTime() const;
	ros::Time endTime() const;

	std::size_t size() const;

	Iterator begin() const;
	Iterator end() const;
	Iterator findTime(const ros::Time& time) const;

	std::size_t numChunks() const;

	int findChunk(const ros::Time& time) const;
	Iterator chunkBegin(int chunk) const;
private:
	class Private;
	std::unique_ptr<Private> m_d;
};

}

namespace ros
{
namespace message_traits
{
template<> struct IsMessage<rosbag_fancy::BagReader::Message> : TrueType { };

template<>
struct MD5Sum<rosbag_fancy::BagReader::Message>
{
	static const char* value(const rosbag_fancy::BagReader::Message& m)
	{
		return m.connection->md5sum.c_str();
	}
};

template<>
struct DataType<rosbag_fancy::BagReader::Message>
{
	static const char* value(const rosbag_fancy::BagReader::Message& m)
	{
		return m.connection->type.c_str();
	}
};

template<>
struct Definition<rosbag_fancy::BagReader::Message>
{
	static const char* value(const rosbag_fancy::BagReader::Message& m)
	{
		return m.connection->msgDef.c_str();
	}
};
}

namespace serialization
{
template<>
struct Serializer<rosbag_fancy::BagReader::Message>
{
	template<typename Stream>
	inline static void write(Stream& stream, const rosbag_fancy::BagReader::Message& t)
	{
		std::memcpy(stream.advance(t.m_size), t.m_data, t.m_size);
	}

	inline static uint32_t serializedLength(const rosbag_fancy::BagReader::Message& t)
	{
		return t.m_size;
	}
};
}
}

#endif
