#include "readings.h"

#include "parameters.h"

namespace virtual_scan
{


Reading &Readings::operator [] (double timestamp)
{
	return readings[timestamp];
}


const Reading &Readings::back() const
{
	return readings.rbegin()->second;
}


const Reading &Readings::front() const
{
	return readings.begin()->second;
}


void Readings::update(carmen_mapper_virtual_scan_message *message)
{
	if (readings.size() == T)
		readings.erase(readings.begin());

	readings.emplace_hint(readings.end(), message->timestamp, message);
}


} // namespace virtual_scan
