#ifndef VIRTUAL_SCAN_READINGS_H
#define VIRTUAL_SCAN_READINGS_H

#include "reading.h"

#include <map>

namespace virtual_scan
{

class Readings
{
	/** @brief Sensor readings indexed by timestamp. */
	std::map<double, Reading> readings;

public:
	/**
	 * @brief Return the sensor reading of given timestamp.
	 */
	Reading &operator [] (double timestamp);

	/**
	 * @brief Return a reference to the latest reading.
	 */
	const Reading &back() const;

	/**
	 * @brief Add the given reading to this sequence, erasing the oldest reading if necessary.
	 */
	void update(carmen_mapper_virtual_scan_message *message);
};

} // namespace virtual_scan

#endif
