#include "readings.h"

#include "parameters.h"

namespace virtual_scan
{


const Reading &Readings::back() const
{
	return rbegin()->second;
}


const Reading &Readings::front() const
{
	return begin()->second;
}


void Readings::update(carmen_mapper_virtual_scan_message *message)
{
	// If the whole time window is filled, discard the oldest reading
	// before adding a new one.
	if (size() == T)
		erase(begin());

	// Add a new, more recent reading to the collection.
	emplace_hint(end(), message->timestamp, message);
}


} // namespace virtual_scan
