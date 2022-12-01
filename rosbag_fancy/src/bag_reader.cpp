// Fast native bag reader
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include "bag_reader.h"

#include <rosfmt/rosfmt.h>
#include <fmt/ostream.h>

#include <sys/mman.h>

#include <bzlib.h>

#include <fcntl.h>

#include <set>

namespace
{
	using Exception = rosbag_fancy::BagReader::Exception;

	struct Span
	{
		uint8_t* start;
		std::size_t size;
	};

	struct Record
	{
		uint8_t* headerBegin;
		uint32_t headerSize;

		uint8_t* dataBegin;
		uint32_t dataSize;

		uint8_t* end;

		std::map<std::string, Span> headers;

		uint8_t op;

		template<class T>
		T integralHeader(const std::string& name)
		{
			auto it = headers.find(name);
			if(it == headers.end())
				throw Exception{fmt::format("Could not find header '{}' in record of type {}\n", name, op)};

			if(it->second.size != sizeof(T))
				throw Exception{fmt::format("Header '{}' has wrong size {} (expected {})\n", name, it->second.size, sizeof(T))};

			T ret;
			std::memcpy(&ret, it->second.start, sizeof(T));
			return ret;
		}

		std::string stringHeader(const std::string& name, const std::optional<std::string>& defaultValue = {})
		{
			auto it = headers.find(name);
			if(it == headers.end())
			{
				if(defaultValue)
					return *defaultValue;
				else
					throw Exception{fmt::format("Could not find header '{}' in record of type {}\n", name, op)};
			}

			return {reinterpret_cast<char*>(it->second.start), it->second.size};
		}
	};

	struct Chunk
	{
		static_assert(sizeof(rosbag_fancy::BagReader::ConnectionInfo) == 8, "ConnectionInfo should have size 8");

		uint8_t* chunkStart;
		std::size_t chunkCompressedSize = 0;

		ros::Time startTime;
		ros::Time endTime;

		std::vector<rosbag_fancy::BagReader::ConnectionInfo> connectionInfos;
	};

	std::map<std::string, Span> readHeader(uint8_t* base, std::size_t remainingSize)
	{
		std::map<std::string, Span> ret;

		while(remainingSize > 0)
		{
			if(remainingSize < 4)
				throw Exception{"Record too small"};

			uint32_t entrySize;
			std::memcpy(&entrySize, base, 4);

			base += 4;
			remainingSize -= 4;

			// Find '=' character
			auto* equal = reinterpret_cast<uint8_t*>(std::memchr(base, '=', entrySize));
			if(!equal)
				throw Exception{"Invalid header map entry"};

			ret[std::string(reinterpret_cast<char*>(base), equal - base)]
				= Span{equal+1, static_cast<std::size_t>(entrySize - 1 - (equal - base))};

			base += entrySize;
			remainingSize -= entrySize;
		}

		return ret;
	}

	Record readRecord(uint8_t* base, std::size_t remainingSize)
	{
		Record record;

		if(remainingSize < 4)
			throw Exception{"Record too small"};

		std::memcpy(&record.headerSize, base, 4);
		remainingSize -= 4;
		base += 4;

		if(remainingSize < record.headerSize)
			throw Exception{"Record too small"};

		record.headerBegin = base;

		base += record.headerSize;
		remainingSize -= record.headerSize;

		if(remainingSize < 4)
			throw Exception{"Record too small"};

		std::memcpy(&record.dataSize, base, 4);
		remainingSize -= 4;
		base += 4;

		if(remainingSize < record.dataSize)
			throw Exception{"Record too small"};

		record.dataBegin = base;

		record.end = base + record.dataSize;

		// Parse header
		record.headers = readHeader(record.headerBegin, record.headerSize);

		auto it = record.headers.find("op");
		if(it == record.headers.end())
			throw Exception{"Record without op header"};

		if(it->second.size != 1)
			throw Exception{fmt::format("op header has invalid size {}", it->second.size)};

		record.op = it->second.start[0];

		return record;
	}


	class MappedFile
	{
	public:
		explicit MappedFile(const std::string& file)
		{
			m_fd = open(file.c_str(), O_RDONLY);
			if(m_fd < 0)
				throw Exception{fmt::format("Could not open file: {}", strerror(errno))};

			// Measure size
			m_size = lseek(m_fd, 0, SEEK_END);
			if(m_size < 0)
			{
				close(m_fd);
				throw Exception{fmt::format("Could not seek: {}", strerror(errno))};
			}

			lseek(m_fd, 0, SEEK_SET);

			m_data = reinterpret_cast<uint8_t*>(mmap(nullptr, m_size, PROT_READ, MAP_PRIVATE, m_fd, 0));
			if(m_data == MAP_FAILED)
			{
				close(m_fd);
				throw Exception{fmt::format("Could not mmap() bagfile: {}", strerror(errno))};
			}
		}

		~MappedFile()
		{
			munmap(m_data, m_size);
			close(m_fd);
		}

		off_t size() const
		{ return m_size; }

		void* data()
		{ return m_data; }

	private:
		int m_fd;
		void* m_data;
		off_t m_size;
	};
}

template <> struct fmt::formatter<Span>: formatter<string_view>
{
	template <typename FormatContext>
	auto format(Span c, FormatContext& ctx)
	{
		if(std::all_of(c.start, c.start + c.size, [](uint8_t c){return std::isprint(c);}))
			return formatter<string_view>::format(string_view(reinterpret_cast<char*>(c.start), c.size), ctx);
		else
		{
			std::stringstream s;
			for(std::size_t i = 0; i < c.size; ++i)
			{
				if(i != 0)
					fmt::print(s, " ");

				fmt::print(s, "{:02X}", c.start[i]);
			}

			return formatter<string_view>::format(string_view(s.str()), ctx);
		}
	}
};


namespace rosbag_fancy
{

class BagReader::Private
{
public:
	explicit Private(const std::string& filename)
	 : file{filename}
	{}

	MappedFile file;
	off_t size;
	uint8_t* data;

	std::vector<Chunk> chunks;
	std::map<std::uint32_t, Connection> connections;
	std::map<std::string, std::vector<std::uint32_t>> connectionsForTopic;

	ros::Time startTime;
	ros::Time endTime;
};

class BagReader::ChunkIterator::Private
{
public:
	enum class Compression
	{
		None,
		BZ2,
		LZ4
	};

	const BagReader* m_reader{};
	std::size_t m_chunk = 0;

	Compression m_compression;
	bz_stream m_bzStream{};

	uint8_t* m_dataPtr = {};
	std::size_t m_remaining = 0;
};

BagReader::ChunkIterator::ChunkIterator(const BagReader* reader, int chunk)
 : m_d{std::make_unique<Private>()}
{
	m_d->m_reader = reader;
	m_d->m_chunk = chunk;

	auto& chunkData = reader->m_d->chunks[chunk];
	auto rec = readRecord(chunkData.chunkStart, chunkData.chunkCompressedSize);

	std::string compression = rec.stringHeader("compression");
	if(compression == "none")
		m_d->m_compression = Private::Compression::None;
	else if(compression == "bz2")
		m_d->m_compression = Private::Compression::BZ2;
	else if(compression == "lz4")
		m_d->m_compression = Private::Compression::LZ4;
	else
		throw Exception{fmt::format("Unknown compression type '{}'", compression)};

	if(m_d->m_compression != Private::Compression::None)
		throw Exception{"Only uncompressed chunks supported for now"};

	m_d->m_dataPtr = rec.dataBegin;
	m_d->m_remaining = rec.dataSize;

	(*this)++;
}

BagReader::ChunkIterator& BagReader::ChunkIterator::operator++()
{
	if(!m_d)
		return *this;

	while(true)
	{
		if(m_d->m_remaining == 0)
		{
			m_d.reset();
			m_idx = 0;
			return *this;
		}

		auto rec = readRecord(m_d->m_dataPtr, m_d->m_remaining);

		m_d->m_remaining -= (rec.end - m_d->m_dataPtr);
		m_d->m_dataPtr = rec.end;

		if(rec.op == 0x02)
		{
			m_msg.m_data = rec.dataBegin;
			m_msg.m_size = rec.dataSize;

			uint32_t connID = rec.integralHeader<uint32_t>("conn");
			if(connID > m_d->m_reader->connections().size())
				throw Exception{fmt::format("Invalid connection ID {}", connID)};

			m_msg.connection = &m_d->m_reader->connections().at(connID);

			uint64_t time = rec.integralHeader<uint64_t>("time");
			m_msg.stamp = ros::Time(time & 0xFFFFFFFF, time >> 32);

			break;
		}
	}

	return *this;
}

BagReader::Iterator::Iterator(const BagReader* reader, int chunk)
 : m_reader{reader}
 , m_chunk{chunk}
 , m_it{reader, chunk}
{
}

BagReader::Iterator& BagReader::Iterator::operator++()
{
	m_it++;
	if(m_it == ChunkIterator{})
	{
		m_chunk++;
		if(m_chunk < static_cast<int>(m_reader->m_d->chunks.size()))
		{
			m_it = ChunkIterator{m_reader, m_chunk};
		}
		else
		{
			m_chunk = -1;
			m_it = {};
		}
	}

	return *this;
}

std::vector<BagReader::ConnectionInfo>& BagReader::Iterator::currentChunkConnections() const
{
	return m_reader->m_d->chunks[m_chunk].connectionInfos;
}


BagReader::BagReader(const std::string& filename)
 : m_d{std::make_unique<Private>(filename)}
{
	m_d->data = reinterpret_cast<uint8_t*>(m_d->file.data());
	m_d->size = m_d->file.size();

	// Read version line
	std::string versionLine;
	uint8_t* bagHeaderBegin{};
	{
		constexpr int MAX_VERSION_LENGTH = 20;
		auto* newline = reinterpret_cast<uint8_t*>(std::memchr(
			m_d->data, '\n', std::min<off_t>(m_d->size, MAX_VERSION_LENGTH))
		);
		if(!newline)
			throw Exception{fmt::format("Could not read version line\n")};

		versionLine = std::string(reinterpret_cast<char*>(m_d->data), newline - m_d->data);
		bagHeaderBegin = newline + 1;
	}

	if(versionLine != "#ROSBAG V2.0")
		throw Exception{"I can't read this file (I can only read ROSBAG V2.0"};

	Record bagHeader = readRecord(bagHeaderBegin, m_d->size - (bagHeaderBegin - m_d->data));

	if(bagHeader.op != 0x03)
		throw Exception{"First record is not a bag header\n"};

	std::uint64_t indexPos = bagHeader.integralHeader<std::uint64_t>("index_pos");
	std::uint32_t connectionCount = bagHeader.integralHeader<std::uint32_t>("conn_count");
	std::uint32_t chunkCount = bagHeader.integralHeader<std::uint32_t>("chunk_count");

	if(indexPos >= static_cast<std::uint64_t>(m_d->size))
		throw Exception{fmt::format("Index position is too large: {} vs data size {}", indexPos, m_d->size)};

	// Read all index records
	uint8_t* rptr = m_d->data + indexPos;
	std::size_t remaining = m_d->size - indexPos;

	// Connection records
	for(std::size_t i = 0; i < connectionCount; ++i)
	{
		Record rec = readRecord(rptr, remaining);
		if(rec.op != 7)
			throw Exception{"Expected connection record"};

		Record recData;
		recData.headers = readHeader(rec.dataBegin, rec.dataSize);

		Connection con;
		con.id = rec.integralHeader<uint32_t>("conn");
		con.topicInBag = rec.stringHeader("topic");
		con.topicAsPublished = recData.stringHeader("topic", std::string{});
		con.type = recData.stringHeader("type");
		con.md5sum = recData.stringHeader("md5sum");
		con.msgDef = recData.stringHeader("message_definition");

		{
			auto it = recData.headers.find("callerid");
			if(it != recData.headers.end())
				con.callerID = recData.stringHeader("callerid");
		}
		{
			auto it = recData.headers.find("latching");
			if(it != recData.headers.end())
			{
				if(it->second.size != 1)
					throw Exception{"Invalid latching header"};

				if(it->second.start[0] == '1')
					con.latching = true;
			}
		}

		m_d->connections[con.id] = con;
		m_d->connectionsForTopic[con.topicInBag].push_back(con.id);

		rptr = rec.end;
		remaining = m_d->size - (rptr - m_d->data);
	}

	// Chunk infos
	m_d->chunks.reserve(chunkCount);
	Chunk* lastChunk = {};
	for(std::size_t i = 0; i < chunkCount; ++i)
	{
		Record rec = readRecord(rptr, remaining);
		if(rec.op != 6)
			throw Exception{"Expected chunk info record"};

		auto& chunk = m_d->chunks.emplace_back();

		auto version = rec.integralHeader<std::uint32_t>("ver");
		if(version != 1)
			throw Exception{fmt::format("Unsupported chunk info version {}", version)};

		auto chunkPos = rec.integralHeader<std::uint64_t>("chunk_pos");
		if(chunkPos >= static_cast<std::uint64_t>(m_d->size))
			throw Exception{"chunk_pos points outside of valid range"};

		chunk.chunkStart = m_d->data + chunkPos;
		if(lastChunk)
			lastChunk->chunkCompressedSize = chunk.chunkStart - lastChunk->chunkStart;

		std::uint64_t startTime = rec.integralHeader<std::uint64_t>("start_time");
		std::uint64_t endTime = rec.integralHeader<std::uint64_t>("end_time");

		chunk.startTime = ros::Time(startTime & 0xFFFFFFFF, startTime >> 32);
		chunk.endTime = ros::Time(endTime & 0xFFFFFFFF, endTime >> 32);

		auto numConnections = rec.integralHeader<std::uint32_t>("count");

		if(rec.dataSize < numConnections * sizeof(ConnectionInfo))
			throw Exception{"Chunk info is too small"};

		chunk.connectionInfos.resize(numConnections);
		std::memcpy(chunk.connectionInfos.data(), rec.dataBegin, numConnections * sizeof(ConnectionInfo));

		for(auto& connInfo : chunk.connectionInfos)
		{
			auto it = m_d->connections.find(connInfo.id);
			if(it == m_d->connections.end())
				throw Exception{fmt::format("Could not find connection with id {}", connInfo.id)};

			it->second.totalCount += connInfo.msgCount;
		}

		rptr = rec.end;
		remaining = m_d->size - (rptr - m_d->data);
		lastChunk = &chunk;
	}
	if(lastChunk)
		lastChunk->chunkCompressedSize = (m_d->data + m_d->size) - lastChunk->chunkStart;

	if(!m_d->chunks.empty())
		m_d->startTime = m_d->chunks.front().startTime;

	for(auto& c : m_d->chunks)
	{
		m_d->startTime = std::min(c.startTime, m_d->startTime);
		m_d->endTime = std::max(c.endTime, m_d->endTime);
	}
}

BagReader::BagReader(BagReader&& other)
 : m_d{std::move(other.m_d)}
{
}

BagReader::~BagReader()
{
}

const BagReader::ConnectionMap& BagReader::connections() const
{
	return m_d->connections;
}

ros::Time BagReader::startTime() const
{
	return m_d->startTime;
}

ros::Time BagReader::endTime() const
{
	return m_d->endTime;
}

std::size_t BagReader::size() const
{
	return m_d->size;
}

BagReader::Iterator BagReader::begin() const
{
	if(m_d->chunks.empty())
		return {};

	return Iterator{this, 0};
}

BagReader::Iterator BagReader::end() const
{
	return Iterator();
}

BagReader::Iterator BagReader::findTime(const ros::Time& time) const
{
	// Find chunk
	auto chunkIt = std::lower_bound(m_d->chunks.begin(), m_d->chunks.end(), time, [&](Chunk& chunk, const ros::Time& time){
		return chunk.endTime < time;
	});

	if(chunkIt == m_d->chunks.end())
		return {};

	int chunkIdx = chunkIt - m_d->chunks.begin();

	// Find message in chunk
	auto it = Iterator{this, chunkIdx};
	for(; it != end(); ++it)
	{
		if(it->stamp > time)
			return it;
	}

	return it;
}

BagReader::Iterator BagReader::chunkBegin(int chunk) const
{
	if(chunk < 0 || chunk >= static_cast<int>(m_d->chunks.size()))
		return {};

	return Iterator{this, chunk};
}

std::size_t BagReader::numChunks() const
{
	return m_d->chunks.size();
}

}
