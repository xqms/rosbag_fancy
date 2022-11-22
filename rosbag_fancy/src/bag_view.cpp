// Filtering view on (multiple) bag files
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include "bag_view.h"

namespace rosbag_fancy
{

class BagView::Private
{
public:
	void addBag(BagReader* reader, const std::function<bool(const BagReader::Connection&)>& connectionPredicate)
	{
		auto& handle = m_bags.emplace_back();
		handle.reader = reader;
		handle.filtering = true;

		for(auto& conn : reader->connections())
		{
			if(connectionPredicate(conn.second))
				handle.connectionIDs.push_back(conn.second.id);
		}
	}

	void addBag(BagReader* reader)
	{
		auto& handle = m_bags.emplace_back();
		handle.reader = reader;
		handle.filtering = false;
	}

private:
	friend class BagView::Iterator::Private;

	struct BagHandle
	{
		BagReader* reader;
		bool filtering;
		std::vector<uint32_t> connectionIDs;
	};
	std::vector<BagHandle> m_bags;
};

class BagView::Iterator::Private
{
public:
	Private(BagView::Private* view)
	{
		for(auto& handle : view->m_bags)
		{
			auto& state = m_state.emplace_back();
			state.handle = &handle;
			state.it = handle.reader->begin();
		}
	}

	struct BagState
	{
		BagView::Private::BagHandle* handle;
		int chunk = -1;
		BagReader::Iterator it;
	};

	std::vector<BagState> m_state;
	BagState* m_nextBag{};
};

BagView::Iterator::~Iterator()
{}

const BagReader::Message& BagView::Iterator::operator*()
{
	if(!m_d)
		throw std::logic_error{"Attempt to dereference invalid BagView::Iterator"};

	return *m_d->m_nextBag->it;
}

BagView::Iterator& BagView::Iterator::operator++()
{
	if(!m_d)
		return *this;

	if(m_d->m_nextBag)
		++m_d->m_nextBag->it;

	ros::Time earliestStamp;
	m_d->m_nextBag = nullptr;

	for(auto& state : m_d->m_state)
	{
		if(state.it == state.handle->reader->end())
			continue;

		if(!m_d->m_nextBag || state.it->stamp < earliestStamp)
		{
			m_d->m_nextBag = &state;
			earliestStamp = state.it->stamp;
		}
	}

	if(!m_d->m_nextBag)
	{
		// End reached, invalidate
		m_d.reset();
		return *this;
	}

	return *this;
}

BagView::BagView()
 : m_d{std::make_unique<Private>()}
{}

BagView::~BagView()
{}

void BagView::addBag(BagReader* reader, const std::function<bool(const BagReader::Connection&)>& connectionPredicate)
{

}

}
