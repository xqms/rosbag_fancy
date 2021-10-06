// Qt table model for rosbag_fancy messages
// Author: Christian Lenz <lenz@ais.uni-bonn.de>

#ifndef TOPIC_MODEL_H
#define TOPIC_MODEL_H

#include <QAbstractTableModel>

#include <rosbag_fancy_msgs/Status.h>

class QTimer;

namespace rqt_rosbag_fancy
{

class TopicModel : public QAbstractTableModel
{
Q_OBJECT
public:
	enum class Column
	{
		Activity,
		Name,
		Publisher,
		Messages,
		Rate,
		Bytes,
		Bandwidth,

		ColumnCount
	};

	explicit TopicModel(QObject* parent = 0);
	virtual ~TopicModel();

	int columnCount(const QModelIndex& parent) const override;
	int rowCount(const QModelIndex& parent) const override;

	QVariant data(const QModelIndex& index, int role) const override;
	QVariant headerData(int section, Qt::Orientation orientation, int role) const override;
	
public Q_SLOTS:
	void setState(const rosbag_fancy_msgs::StatusConstPtr& status);

private Q_SLOTS:
	void clear();

private:
	rosbag_fancy_msgs::StatusConstPtr m_status;
	bool m_valid = false;
	QTimer* m_timer;
	
	QString rateToString(double rate) const;
	QString memoryToString(uint64_t memory) const;
	
	std::vector<unsigned int> m_lastMsgCount;
};

}

#endif
