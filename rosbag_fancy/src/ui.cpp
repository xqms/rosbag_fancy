// Terminal UI
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "ui.h"
#include "bag_writer.h"
#include "bag_reader.h"

#include <fmt/format.h>
#include <fmt/chrono.h>

#include <ros/node_handle.h>

namespace rosbag_fancy
{

namespace
{
	unsigned int g_statusLines = 2;
	std::string g_windowTitle;

	void cleanup()
	{
		for(unsigned int i = 0; i < g_statusLines+1; ++i)
			printf("\n");

		Terminal term;

		// Switch cursor back on
		term.setCursorVisible();

		// Switch character echo on
		term.setEcho(true);
	}

	std::string memoryToString(uint64_t memory)
	{
		if(memory < static_cast<uint64_t>(1<<10))
			return fmt::format("{}.0   B", memory);
		else if(memory < static_cast<uint64_t>(1<<20))
			return fmt::format("{:.1f} KiB", static_cast<double>(memory) / static_cast<uint64_t>(1<<10));
		else if(memory < static_cast<uint64_t>(1<<30))
			return fmt::format("{:.1f} MiB", static_cast<double>(memory) / static_cast<uint64_t>(1<<20));
		else if(memory < static_cast<uint64_t>(1ull<<40))
			return fmt::format("{:.1f} GiB", static_cast<double>(memory) / static_cast<uint64_t>(1ull<<30));
		else
			return fmt::format("{:.1f} TiB", static_cast<double>(memory) / static_cast<uint64_t>(1ull<<40));
	}

	std::string rateToString(double rate)
	{
		if(rate < 1000.0)
			return fmt::format("{:5.1f}  Hz", rate);
		else if(rate < 1e6)
			return fmt::format("{:5.1f} kHz", rate / 1e3);
		else
			return fmt::format("{:5.1f} MHz", rate / 1e6);
	}

	template<int Columns>
	class TableWriter
	{
	public:
		struct Column
		{
			Column(const std::string& label, unsigned int width = 0, const std::string& format = {})
			 : label(label), width(width), format(format)
			{
				if(width == 0)
					this->width = label.size();

				valueFormatString = fmt::format("{{:{}{}}}", this->width, format);
			}

			std::string label;
			unsigned int width;
			std::string format;

			std::string valueFormatString;
		};

		TableWriter(Terminal& term, const std::array<Column, Columns>& columns)
		 : m_term(term)
		 , m_cols(columns)
		{}

		void printHeader()
		{
			{
				bool first = true;
				for(auto& col : m_cols)
				{
					if(first)
						first = false;
					else
						fmt::print(" │ ");

					std::string format = fmt::format("{{:^{}}}", col.width);
					fmt::print(format, col.label);
				}
				fmt::print("\n");
			}

			printDash();
		}

		void printDash()
		{
			bool first = true;
			for(auto& col : m_cols)
			{
				if(first)
					first = false;
				else
					fmt::print("─┼─");

				for(unsigned int i = 0; i < col.width; ++i)
					fmt::print("─");
			}
			fmt::print("\n");
		}

		void startRow()
		{
			m_currentCol = 0;
		}

		void endRow()
		{
			fmt::print("\n");
		}

		template<class T>
		void printColumn(const T& value, uint32_t color = 0)
		{
			if(m_currentCol != 0)
				fmt::print(" │ ");

			if(color != 0)
				m_term.setForegroundColor(color);

			fmt::print(m_cols[m_currentCol].valueFormatString, value);

			if(color != 0)
				m_term.setStandardColors();

			m_currentCol++;
		}

	private:
		Terminal& m_term;
		std::array<Column, Columns> m_cols;

		unsigned int m_currentCol = 0;
	};
}

UI::UI(TopicManager& config, MessageQueue& queue, BagWriter& writer)
 : m_topicManager{config}
 , m_queue{queue}
 , m_bagWriter{writer}
{
	std::atexit(&cleanup);

	// Switch cursor off
	m_term.setCursorInvisible();

	// Switch character echo off
	m_term.setEcho(false);

	g_statusLines = m_topicManager.topics().size();

	ros::NodeHandle nh;
	m_timer = nh.createSteadyTimer(ros::WallDuration(0.1), boost::bind(&UI::draw, this));
}

template<class... Args>
void UI::printLine(unsigned int& lineCounter, const Args& ... args)
{
	lineCounter++;
	m_term.clearToEndOfLine();
	fmt::print(args...);
	putchar('\n');
}

void UI::draw()
{
	unsigned int cnt = 0;

	ros::WallTime now = ros::WallTime::now();

	printLine(cnt, "");
	printLine(cnt, "");

	uint64_t totalMessages = 0;
	uint64_t totalBytes = 0;
	float totalRate = 0.0f;
	float totalBandwidth = 0.0f;
	bool totalActivity = false;
	unsigned int totalDrops = 0;

	unsigned int maxTopicWidth = 0;
	for(auto& topic : m_topicManager.topics())
		maxTopicWidth = std::max<unsigned int>(maxTopicWidth, topic.name.length());

	TableWriter<6> writer{m_term, {{
		{"Act"},
		{"Topic", maxTopicWidth},
		{"Pub"},
		{"Messages", 22},
		{"Bytes", 25},
		{"Drops"}
	}}};

	writer.printHeader();
	cnt += 2;

	auto& counts = m_bagWriter.messageCounts();
	auto& bytes = m_bagWriter.byteCounts();

	for(auto& topic : m_topicManager.topics())
	{
		float messageRate = topic.messageRateAt(now);
		bool activity = topic.lastMessageTime > m_lastDrawTime;

		writer.startRow();

		if(activity)
		{
			totalActivity = true;
			writer.printColumn(" ▮ ", 0x00FF00);
		}
		else
			writer.printColumn("");

		writer.printColumn(topic.name);
		writer.printColumn(topic.numPublishers, topic.numPublishers == 0 ? 0x0000FF : 0);

		uint32_t messageColor = (topic.totalMessages == 0) ? 0x0000FF : 0;

		uint64_t messageCount = topic.id < counts.size() ? counts[topic.id] : 0;
		uint64_t byteCount = topic.id < bytes.size() ? bytes[topic.id] : 0;

		writer.printColumn(fmt::format("{:10} ({:>8})", messageCount, rateToString(messageRate)), messageColor);
		writer.printColumn(fmt::format("{:>10} ({:>10}/s)", memoryToString(byteCount), memoryToString(topic.bandwidth)));

		writer.printColumn(topic.dropCounter, topic.dropCounter > 0 ? 0x0000FF : 0);

		writer.endRow();
		cnt++;

		totalMessages += messageCount;
		totalBytes += byteCount;
		totalRate += messageRate;
		totalBandwidth += topic.bandwidth;
		totalDrops += topic.dropCounter;

		m_term.setStandardColors();
	}
	writer.printDash();
	cnt++;

	writer.startRow();
	if(totalActivity)
		writer.printColumn(" ▮ ", 0x00FF00);
	else
		writer.printColumn("");
	writer.printColumn("All");
	writer.printColumn("");
	writer.printColumn(fmt::format("{:10} ({:>8})", totalMessages, rateToString(totalRate)));
	writer.printColumn(fmt::format("{:>10} ({:>10}/s)", memoryToString(totalBytes), memoryToString(totalBandwidth)));
	writer.printColumn(totalDrops, totalDrops > 0 ? 0x0000FF : 0);

	writer.endRow();
	cnt++;

	printLine(cnt, "");
	if(m_bagWriter.running())
	{
		m_term.setForegroundColor(0x00FF00);
		printLine(cnt, "Recording...");
		m_term.setStandardColors();
	}
	else
	{
		m_term.setForegroundColor(0x0000FF);
		printLine(cnt, "Paused.");
		m_term.setStandardColors();
	}
	printLine(cnt, "Message queue:  {:10} messages, {:>10}", m_queue.messagesInQueue(), memoryToString(m_queue.bytesInQueue()));

	printLine(cnt, "Compression:    {:>10}", [&](){
		switch(m_bagWriter.compression())
		{
			case rosbag::compression::Uncompressed: return "None";
			case rosbag::compression::BZ2:          return "BZ2";
			case rosbag::compression::LZ4:          return "LZ4";
		}

		return "Unknown";
	}());

	if(m_bagWriter.splitSizeInBytes() != 0)
	{
		printLine(cnt, "Bag size:       {:>10} / {:>10} split size / {:>10} available",
			memoryToString(m_bagWriter.sizeInBytes()),
			memoryToString(m_bagWriter.splitSizeInBytes()),
			memoryToString(m_bagWriter.freeSpace())
		);
	}
	else
	{
		printLine(cnt, "Bag size:       {:>10} / {:>10} available",
			memoryToString(m_bagWriter.sizeInBytes()),
			memoryToString(m_bagWriter.freeSpace())
		);
	}

	if(m_bagWriter.deleteOldAtInBytes() != 0)
	{
		printLine(cnt, "Directory size: {:>10} / {:>10}",
			memoryToString(m_bagWriter.directorySizeInBytes()),
			memoryToString(m_bagWriter.deleteOldAtInBytes())
		);
	}
	else
	{
		printLine(cnt, "Directory size: {:>10}",
			memoryToString(m_bagWriter.directorySizeInBytes())
		);
	}

	g_statusLines = cnt;

	// Move back
	m_term.clearToEndOfLine();
	m_term.moveCursorUp(g_statusLines);
	m_term.moveCursorToStartOfLine();
	fflush(stdout);

	m_lastDrawTime = now;
}



// Playback UI
PlaybackUI::PlaybackUI(TopicManager& topics, BagReader& reader)
 : m_topicManager{topics}
 , m_bagReader{reader}
{
	std::atexit(&cleanup);

	// Switch cursor off
	m_term.setCursorInvisible();

	// Switch character echo off
	m_term.setEcho(false);

	g_statusLines = m_topicManager.topics().size();

	ros::NodeHandle nh;
	m_timer = nh.createSteadyTimer(ros::WallDuration(0.1), boost::bind(&PlaybackUI::draw, this));
}

template<class... Args>
void PlaybackUI::printLine(unsigned int& lineCounter, const Args& ... args)
{
	lineCounter++;
	m_term.clearToEndOfLine();
	fmt::print(args...);
	putchar('\n');
}

void PlaybackUI::draw()
{
	unsigned int cnt = 0;

	ros::WallTime now = ros::WallTime::now();

	printLine(cnt, "");
	printLine(cnt, "");

	float totalRate = 0.0f;
	float totalBandwidth = 0.0f;
	bool totalActivity = false;
	unsigned int totalDrops = 0;

	unsigned int maxTopicWidth = 0;
	for(auto& topic : m_topicManager.topics())
		maxTopicWidth = std::max<unsigned int>(maxTopicWidth, topic.name.length());

	TableWriter<4> writer{m_term, {{
		{"Act"},
		{"Topic", maxTopicWidth},
		{"Messages", 22},
		{"Bytes", 25}
	}}};

	writer.printHeader();
	cnt += 2;

	for(auto& topic : m_topicManager.topics())
	{
		float messageRate = topic.messageRateAt(now);
		bool activity = topic.lastMessageTime > m_lastDrawTime;

		writer.startRow();

		if(activity)
		{
			totalActivity = true;
			writer.printColumn(" ▮ ", 0x00FF00);
		}
		else
			writer.printColumn("");

		writer.printColumn(topic.name);

		uint32_t messageColor = (topic.totalMessages == 0) ? 0x0000FF : 0;

		writer.printColumn(fmt::format("{:>8}", rateToString(messageRate)), messageColor);
		writer.printColumn(fmt::format("{:>10}/s", memoryToString(topic.bandwidth)));

		writer.endRow();
		cnt++;

		totalRate += messageRate;
		totalBandwidth += topic.bandwidth;
		totalDrops += topic.dropCounter;

		m_term.setStandardColors();
	}
	writer.printDash();
	cnt++;

	writer.startRow();
	if(totalActivity)
		writer.printColumn(" ▮ ", 0x00FF00);
	else
		writer.printColumn("");
	writer.printColumn("All");
	writer.printColumn(fmt::format("{:>8}", rateToString(totalRate)));
	writer.printColumn(fmt::format("{:>10}/s", memoryToString(totalBandwidth)));

	writer.endRow();
	cnt++;

	printLine(cnt, "");

	// Size info
	{
		// A lot of std::chrono magic to get local/UTC time
		std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> startTimeC(std::chrono::nanoseconds(m_bagReader.startTime().toNSec()));
		std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> endTimeC(std::chrono::nanoseconds(m_bagReader.endTime().toNSec()));
		std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> currentTimeC(std::chrono::nanoseconds(m_positionInBag.toNSec()));

		std::chrono::seconds startTimeS = std::chrono::duration_cast<std::chrono::seconds>(startTimeC.time_since_epoch());
		std::time_t startTimeSC(startTimeS.count());
		std::tm startTimeB;
		std::tm startTimeBUTC;
		localtime_r(&startTimeSC, &startTimeB);
		gmtime_r(&startTimeSC, &startTimeBUTC);

		std::chrono::seconds endTimeS = std::chrono::duration_cast<std::chrono::seconds>(endTimeC.time_since_epoch());
		std::time_t endTimeSC(endTimeS.count());
		std::tm endTimeB;
		std::tm endTimeBUTC;
		localtime_r(&endTimeSC, &endTimeB);
		gmtime_r(&endTimeSC, &endTimeBUTC);

		std::chrono::seconds currentTimeS = std::chrono::duration_cast<std::chrono::seconds>(currentTimeC.time_since_epoch());
		std::time_t currentTimeSC(currentTimeS.count());
		std::tm currentTimeB;
		std::tm currentTimeBUTC;
		localtime_r(&currentTimeSC, &currentTimeB);
		gmtime_r(&currentTimeSC, &currentTimeBUTC);

		std::chrono::duration<double, std::nano> duration{(m_bagReader.endTime() - m_bagReader.startTime()).toNSec()};
		std::chrono::duration<double, std::nano> positionInBag{(m_positionInBag - m_bagReader.startTime()).toNSec()};

		printLine(cnt, "Start time:     {:%Y-%m-%d %H:%M:%S} ({}) / {:%Y-%m-%d %H:%M:%S} (UTC)", startTimeB, daylight ? tzname[1] : tzname[0], startTimeBUTC);
		printLine(cnt, "End time:       {:%Y-%m-%d %H:%M:%S} ({}) / {:%Y-%m-%d %H:%M:%S} (UTC)", endTimeB, daylight ? tzname[1] : tzname[0], endTimeBUTC);
		printLine(cnt, "Current time:   {:%Y-%m-%d %H:%M:%S} ({}) / {:%Y-%m-%d %H:%M:%S} (UTC)", currentTimeB, daylight ? tzname[1] : tzname[0], currentTimeBUTC);
		printLine(cnt, "Duration:       {:.2%H:%M:%S} ({:7.2f}s)", duration, (m_bagReader.endTime() - m_bagReader.startTime()).toSec());
		printLine(cnt, "Position:       {:.2%H:%M:%S} ({:7.2f}s)", positionInBag, (m_positionInBag - m_bagReader.startTime()).toSec());
		printLine(cnt, "Size:           {}", memoryToString(m_bagReader.size()));
		printLine(cnt, "");
	}

	// Progress bar
	{
		int w = 80, h;
		m_term.getSize(&w, &h);

		w -= 3;
		fmt::print("│");

		int steps = w * 8;
		int pos = (m_positionInBag - m_bagReader.startTime()).toSec() / (m_bagReader.endTime() - m_bagReader.startTime()).toSec() * steps;

		for(int i = 0; i < pos / 8; ++i)
			fmt::print("█");
		switch(pos % 8)
		{
			case 0: fmt::print(" "); break;
			case 1: fmt::print("▏"); break;
			case 2: fmt::print("▎"); break;
			case 3: fmt::print("▍"); break;
			case 4: fmt::print("▌"); break;
			case 5: fmt::print("▋"); break;
			case 6: fmt::print("▊"); break;
			case 7: fmt::print("▉"); break;
		}

		for(int i = 0; i < w - pos/8 - 1; ++i)
			putchar(' ');
		fmt::print("│\n");

		cnt++;
	}

	printLine(cnt, "");
	printLine(cnt, "Hit [space] for pause, [left]/[right] for seeking");

	g_statusLines = cnt;

	// Move back
	m_term.clearToEndOfLine();
	m_term.moveCursorUp(g_statusLines);
	m_term.moveCursorToStartOfLine();
	fflush(stdout);

	m_lastDrawTime = now;
}

void PlaybackUI::setPositionInBag(const ros::Time& stamp)
{
	m_positionInBag = stamp;
}

void PlaybackUI::handleInput()
{
	int c = m_term.readKey();
	if(c < 0)
		return;

	switch(c)
	{
		case Terminal::SK_Left:
			seekBackwardRequested();
			break;
		case Terminal::SK_Right:
			seekForwardRequested();
			break;
		case ' ':
			pauseRequested();
			break;
	}
}

}
