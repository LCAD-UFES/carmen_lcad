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

} // namespace virtual_scan

#endif
