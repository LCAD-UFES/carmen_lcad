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


void Readings::erase(double timestamp)
{
	for (auto reading = begin(), n = upper_bound(timestamp); reading != n; reading++)
		std::map<double, Reading>::erase(reading);
}


void Readings::update(carmen_mapper_virtual_scan_message *message)
{
	// If the whole time window is filled, discard the oldest reading
	// before adding a new one.
	if (size() == T)
		std::map<double, Reading>::erase(begin());

	// Add a new, more recent reading to the collection.
	emplace_hint(end(), message->timestamp, message);
}


void Readings::update(const Reading &reading)
{
	// If the whole time window is filled, discard the oldest reading
	// before adding a new one.
	if (size() == T)
		std::map<double, Reading>::erase(begin());

	// Add a new, more recent reading to the collection.
	emplace_hint(end(), reading.timestamp, reading);
}


} // namespace virtual_scan
