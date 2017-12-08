#ifndef VIRTUAL_SCAN_RANDOM_H
#define VIRTUAL_SCAN_RANDOM_H

#include <random>

namespace virtual_scan
{

/** @brief Default random device for the module. */
extern std::random_device RD;

/**
 * @brief Return a random integer in the range `[a, b)`.
 */
int random_int(int a, int b);

template<class T> T &random_choose(std::vector<T> &vector)
{
	return vector[random_int(0, vector.size())];
}

} // namespace virtual_scan

#endif
