// Terminal UI
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "ui.h"
#include "bag_writer.h"

#include <fmt/format.h>

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

UI::UI(TopicManager& config, MessageQueue& queue, BagWriter& writer, Mode mode)
 : m_topicManager{config}
 , m_queue{queue}
 , m_bagWriter{writer}
 , m_mode{mode}
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

		writer.printColumn(fmt::format("{:10} ({:>8})", counts[topic.id], rateToString(messageRate)), messageColor);
		writer.printColumn(fmt::format("{:>10} ({:>10}/s)", memoryToString(bytes[topic.id]), memoryToString(topic.bandwidth)));

		writer.printColumn(topic.dropCounter, topic.dropCounter > 0 ? 0x0000FF : 0);

		writer.endRow();
		cnt++;

		totalMessages += counts[topic.id];
		totalBytes += bytes[topic.id];
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

}
