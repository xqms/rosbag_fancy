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
			state.skipToNext();
		}
	}

	Private(const BagView::Private* view, const ros::Time& time)
	{
		for(auto& handle : view->m_bags)
		{
			auto& state = m_state.emplace_back();
			state.handle = &handle;
			state.it = handle.reader->findTime(time);
			state.skipToNext();
		}
	}

	struct BagState
	{
		const BagView::Private::BagHandle* handle;
		int chunk = -1;
		BagReader::Iterator it;

		void skipToNext()
		{
			// We need to skip to the next valid message in this bag.
			auto messageIsInteresting = [&](const BagReader::Message& msg){
				return handle->connectionIDs[msg.connection->id];
			};

			while(it != handle->reader->end() && !messageIsInteresting(*it))
			{
				if(it.chunk() != chunk)
				{
					// We advanced into the next (or the first) chunk. This is an opportunity to skip a whole chunk ahead!
					auto chunkIsInteresting = [&](const std::vector<BagReader::ConnectionInfo>& connections){
						return std::any_of(connections.begin(), connections.end(), [&](auto& con){
							return con.msgCount != 0 && handle->connectionIDs[con.id];
						});
					};

					int numChunks = handle->reader->numChunks();
					while(true)
					{
						if(chunkIsInteresting(it.currentChunkConnections()))
							break;

						if(it.chunk() >= numChunks-1)
						{
							// Arrived at the end
							it = handle->reader->end();
						}
						else
						{
							// Try next chunk
							it = handle->reader->chunkBegin(it.chunk() + 1);
						}
					}

					chunk = it.chunk();
				}
				else
				{
					// Just increment and look at the next message!
					++it;
				}
			}
		}
	};

	std::vector<BagState> m_state;
	BagState* m_nextBag{};
	MultiBagMessage m_msg;
};

BagView::Iterator::Iterator(const BagView* view)
 : m_d{std::make_shared<Private>(view->m_d.get())}
{
	++(*this);
}

BagView::Iterator::Iterator(const BagView* view, const ros::Time& time)
 : m_d{std::make_shared<Private>(view->m_d.get(), time)}
{
	++(*this);
}

BagView::Iterator::~Iterator()
{}

const BagView::MultiBagMessage& BagView::Iterator::operator*()
{
	if(!m_d)
		throw std::logic_error{"Attempt to dereference invalid BagView::Iterator"};

	return m_d->m_msg;
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
		bag->skipToNext();
	}

	// Figure out the earliest available message from all the bags
	ros::Time earliestStamp;
	m_d->m_nextBag = nullptr;
	std::size_t bagIndex = 0;

	for(std::size_t i = 0; i < m_d->m_state.size(); ++i)
	{
		auto& state = m_d->m_state[i];

		if(state.it == state.handle->reader->end())
			continue;

		if(!m_d->m_nextBag || state.it->stamp < earliestStamp)
		{
			m_d->m_nextBag = &state;
			earliestStamp = state.it->stamp;
			bagIndex = i;
		}
	}

	if(!m_d->m_nextBag)
	{
		// End reached, invalidate
		m_d.reset();
		return *this;
	}

	// Found a message!
	m_d->m_msg.msg = &*m_d->m_nextBag->it;
	m_d->m_msg.bagIndex = bagIndex;

	return *this;
}

bool operator==(const BagView::Iterator& a, const BagView::Iterator& b)
{
	// NOTE: This way view.begin() != view.begin(), but I don't care.
	return a.m_d == b.m_d;
}

bool operator!=(const BagView::Iterator& a, const BagView::Iterator& b)
{
	// NOTE: This way view.begin() != view.begin(), but I don't care.
	return a.m_d != b.m_d;
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
