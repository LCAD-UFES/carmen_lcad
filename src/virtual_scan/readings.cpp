#include "readings.h"

namespace virtual_scan
{


Reading &Readings::operator [] (double timestamp)
{
	return readings[timestamp];
};


} // namespace virtual_scan
