#ifndef VIRTUAL_SCAN_READINGS_H
#define VIRTUAL_SCAN_READINGS_H

#include "reading.h"

#include <map>

namespace virtual_scan
{

struct Readings: std::map<double, Reading>
{
	/**
	 * @brief Return a reference to the latest reading.
	 */
	const Reading &back() const;

	/**
	 * @brief Return a reference to the oldest reading.
	 */
	const Reading &front() const;

	/**
	 * @brief Add the given reading to this sequence, erasing the oldest reading if necessary.
	 */
	void update(carmen_mapper_virtual_scan_message *message);
};

} // namespace virtual_scan

#endif
