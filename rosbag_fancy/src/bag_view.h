// Filtering view on (multiple) bag files
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef ROSBAG_FANCY_BAG_VIEW_H
#define ROSBAG_FANCY_BAG_VIEW_H

#include "bag_reader.h"

namespace rosbag_fancy
{

class BagView
{
public:
	class Iterator
	{
	public:
		using iterator_category = std::input_iterator_tag;
		using value_type        = BagReader::Message;
		using reference         = const BagReader::Message&;
		using pointer           = const BagReader::Message*;

		Iterator() {}
		~Iterator();

		Iterator(const Iterator&) = default;
		Iterator& operator=(const Iterator&) = default;

		reference operator*();
		pointer operator->() { return &(**this); }

		Iterator& operator++();
		Iterator operator++(int) { Iterator tmp = *this; ++(*this); return tmp; }

		friend bool operator== (const Iterator& a, const Iterator& b);
		friend bool operator!= (const Iterator& a, const Iterator& b);

	private:
		friend class BagView;
		class Private;
		std::shared_ptr<Private> m_d;
	};

	BagView();
	~BagView();

	BagView(const BagView&) = delete;
	BagView& operator=(const BagView&) = delete;

	void addBag(BagReader* reader);
	void addBag(BagReader* reader, const std::function<bool(const BagReader::Connection&)>& connectionPredicate);

	Iterator begin();
	Iterator end();
	Iterator findTime(const ros::Time& time);

private:
	friend class Iterator::Private;
	class Private;
	std::unique_ptr<Private> m_d;
};

}

#endif
