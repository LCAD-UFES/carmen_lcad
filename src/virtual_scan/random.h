#ifndef VIRTUAL_SCAN_RANDOM_H
#define VIRTUAL_SCAN_RANDOM_H

#include <deque>
#include <random>
#include <vector>

namespace virtual_scan
{

/** @brief Default random device for the module. */
extern std::random_device RD;

/**
 * @brief Return a random integer in the range `[a, b)`.
 */
int random_int(int a, int b);

/**
 * @brief Return a randomly selected element from the given sequence.
 *
 * The sequence must implement the `[]` operator for random access.
 */
template<class T> typename T::value_type &random_choose(T &sequence)
{
	return sequence[random_int(0, sequence.size())];
}

} // namespace virtual_scan

#endif
