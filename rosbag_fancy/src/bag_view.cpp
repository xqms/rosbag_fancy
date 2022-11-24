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
			{
				if(handle.connectionIDs.size() <= conn.first)
					handle.connectionIDs.resize(conn.first+1, false);

				handle.connectionIDs[conn.first] = true;
			}
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
		std::vector<bool> connectionIDs;
	};
	std::vector<BagHandle> m_bags;
};

class BagView::Iterator::Private
{
public:
	Private(const BagView::Private* view)
	{
		for(auto& handle : view->m_bags)
		{
			auto& state = m_state.emplace_back();
			state.handle = &handle;
			state.it = handle.reader->begin();
		}
	}

	Private(const BagView::Private* view, const ros::Time& time)
	{
		for(auto& handle : view->m_bags)
		{
			auto& state = m_state.emplace_back();
			state.handle = &handle;
			state.it = handle.reader->findTime(time);
		}
	}

	struct BagState
	{
		const BagView::Private::BagHandle* handle;
		int chunk = -1;
		BagReader::Iterator it;
	};

	std::vector<BagState> m_state;
	BagState* m_nextBag{};
};

BagView::Iterator::Iterator(const BagView* view)
 : m_d{std::make_shared<Private>(view->m_d.get())}
{
}

BagView::Iterator::Iterator(const BagView* view, const ros::Time& time)
 : m_d{std::make_shared<Private>(view->m_d.get(), time)}
{
}

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
	{
		auto* bag = m_d->m_nextBag;

		// Do one increment so we do not check the current message again
		++bag->it;

		// We need to skip to the next valid message in this bag.
		auto messageIsInteresting = [&](const BagReader::Message& msg){
			return bag->handle->connectionIDs[msg.connection->id];
		};

		while(bag->it != bag->handle->reader->end() && !messageIsInteresting(*bag->it))
		{
			if(bag->it.chunk() != bag->chunk)
			{
				// We advanced into the next (or the first) chunk. This is an opportunity to skip a whole chunk ahead!
				auto chunkIsInteresting = [&](const std::vector<BagReader::ConnectionInfo>& connections){
					return std::any_of(connections.begin(), connections.end(), [&](auto& con){
						return con.msgCount != 0 && bag->handle->connectionIDs[con.id];
					});
				};

				int numChunks = bag->handle->reader->numChunks();
				while(true)
				{
					if(chunkIsInteresting(bag->it.currentChunkConnections()))
						break;

					if(bag->it.chunk() >= numChunks-1)
					{
						// Arrived at the end
						bag->it = bag->handle->reader->end();
					}
					else
					{
						// Try next chunk
						bag->it = bag->handle->reader->chunkBegin(bag->it.chunk() + 1);
					}
				}

				bag->chunk = bag->it.chunk();
			}
			else
			{
				// Just increment and look at the next message!
				++bag->it;
			}
		}
	}

	// Figure out the earliest available message from all the bags
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
	m_d->addBag(reader, connectionPredicate);
}

void BagView::addBag(BagReader* reader)
{
	m_d->addBag(reader);
}

BagView::Iterator BagView::begin() const
{
	return BagView::Iterator{this};
}

BagView::Iterator BagView::end() const
{
	return {};
}

BagView::Iterator BagView::findTime(const ros::Time& time) const
{
	return BagView::Iterator{this, time};
}

}
