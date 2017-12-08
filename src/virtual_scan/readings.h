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
};

} // namespace virtual_scan

#endif
