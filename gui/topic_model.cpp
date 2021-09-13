// Qt table model for rosbag_fancy messages
// Author: Christian Lenz <lenz@ais.uni-bonn.de>

#include "topic_model.h"

#include <QColor>
#include <QTimer>
#include <rosfmt/full.h>

namespace rosbag_fancy
{

TopicModel::TopicModel(QObject* parent)
 : QAbstractTableModel(parent)
{
	m_timer = new QTimer(this);
	m_timer->setSingleShot(true);
	m_timer->setInterval(3000);

	connect(m_timer, &QTimer::timeout, this, &TopicModel::clear);
}

TopicModel::~TopicModel()
{
}

void TopicModel::clear()
{
	m_valid = false;

	dataChanged(
		index(0,0),
		index(m_status->topics.size()-1, static_cast<int>(Column::ColumnCount)-1)
	);
}

int TopicModel::columnCount(const QModelIndex& parent) const
{
	if(parent.isValid())
		return 0;

	return static_cast<int>(Column::ColumnCount);
}

int TopicModel::rowCount(const QModelIndex& parent) const
{
	if(parent.isValid())
		return 0;

	if(!m_status)
		return 0;

	return m_status->topics.size();
}

QVariant TopicModel::data(const QModelIndex& index, int role) const
{
	if(!m_status)
		return 0;

	if(index.parent().isValid())
		return QVariant();

	int column = index.column();
	if(column < 0 || column >= static_cast<int>(Column::ColumnCount))
		return QVariant();

	Column col = static_cast<Column>(column);

	int row = index.row();
	if(row < 0 || row >= (int)m_status->topics.size())
		return QVariant();

	const auto& topic = m_status->topics[row];
	
	bool active = topic.messages > m_lastMsgCount[row];
				
	switch(role)
	{
		case Qt::ForegroundRole:
			if(m_valid)
			{
				if(topic.publishers < 1 && col == Column::Publisher)
					return QColor(Qt::white);
				else
					return QVariant{};
			}
			else
				return QVariant{};

			break;

		case Qt::BackgroundRole:
			if(m_valid)
			{
				if(active && col == Column::Activity && m_status->status == rosbag_fancy::Status::STATUS_RUNNING)
					return QColor(Qt::green);
				else if(topic.publishers < 1 && col == Column::Publisher)
					return QColor(Qt::red);
				else
					return QVariant{};
			}
			else
				return QColor(Qt::gray);

			break;

		case Qt::DisplayRole:
			switch(col)
			{
				case Column::Activity:
					return QString("");
				case Column::Name:
					return QString::fromStdString(topic.name);
				case Column::Publisher:
					return QString::number(topic.publishers);
				case Column::Messages:
					return QString::number(topic.messages);
				case Column::Rate:
					return rateToString(topic.rate);
				case Column::Bytes:
					return memoryToString(topic.bytes);
				case Column::Bandwidth:
					return memoryToString(topic.bandwidth) + "/s";
				default:
					return QVariant();
			}
			break;

		case Qt::TextAlignmentRole:
			switch(col)
			{
				case Column::Name:
					return int(Qt::AlignLeft | Qt::AlignVCenter);

				default:
					return int(Qt::AlignRight | Qt::AlignVCenter);
			}
			break;
	}

	return QVariant();
}

QVariant TopicModel::headerData(int section, Qt::Orientation orientation, int role) const
{
	if(role != Qt::DisplayRole || orientation != Qt::Horizontal)
		return QVariant();

	if(section < 0 || section > static_cast<int>(Column::ColumnCount))
		return QVariant();

	Column col = static_cast<Column>(section);

	switch(col)
	{
		case Column::Activity:
			return "Act";
		case Column::Name:
			return "Name";
		case Column::Publisher:
			return "Publisher";
		case Column::Messages:
			return "Messages";
		case Column::Rate:
			return "Rate";
		case Column::Bytes:
			return "Bytes";
		case Column::Bandwidth:
			return "Bandwidth";
		default:
			return QVariant();
	}
}

void TopicModel::setState(const rosbag_fancy::StatusConstPtr& state)
{
	if(!m_status || m_status->topics.size() != state->topics.size())
	{
		m_lastMsgCount.clear();
		m_lastMsgCount.reserve(state->topics.size());
		
		for(auto& t : state->topics)
			m_lastMsgCount.push_back(t.messages);
		
		beginResetModel();
		m_status = state;
		endResetModel();
		
		return;
	}

	for(unsigned int i=0;i<m_status->topics.size();i++)
		m_lastMsgCount[i] = m_status->topics[i].messages;
	
	m_status = state;
	m_valid = true;
	dataChanged(
		index(0,0),
		index(m_status->topics.size()-1, static_cast<int>(Column::ColumnCount)-1)
	);

	m_timer->start();
}

QString TopicModel::rateToString(double rate) const
{
	std::string s;
	if(rate < 1000.0)
		s = fmt::format("{:.1f}  Hz", rate);
	else if(rate < 1e6)
		s = fmt::format("{:.1f} kHz", rate / 1e3);
	else
		s = fmt::format("{:.1f} MHz", rate / 1e6);
	
	return QString::fromStdString(s);
}

QString TopicModel::memoryToString(uint64_t memory) const
{
	std::string s;
	if(memory < static_cast<uint64_t>(1<<10))
		s = fmt::format("{}.0   B", memory);
	else if(memory < static_cast<uint64_t>(1<<20))
		s = fmt::format("{:.1f} KiB", static_cast<double>(memory) / static_cast<uint64_t>(1<<10));
	else if(memory < static_cast<uint64_t>(1<<30))
		s = fmt::format("{:.1f} MiB", static_cast<double>(memory) / static_cast<uint64_t>(1<<20));
	else if(memory < static_cast<uint64_t>(1ull<<40))
		s = fmt::format("{:.1f} GiB", static_cast<double>(memory) / static_cast<uint64_t>(1ull<<30));
	else
		s = fmt::format("{:.1f} TiB", static_cast<double>(memory) / static_cast<uint64_t>(1ull<<40));
	
	return QString::fromStdString(s);
}

}
