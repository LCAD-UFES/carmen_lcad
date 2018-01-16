#include "random.h"

namespace virtual_scan
{


std::random_device RD;


int random_int(int a, int b)
{
    // If the range has 1 or less elements, return
    // the range start.
    if (b - a <= 1)
        return a;

    // uniform_int_distribution returns an int in the range [a, b], so
    // we need to subtract 1 from b to return an int in the range [a, b).
    // See: http://en.cppreference.com/w/cpp/numeric/random/uniform_int_distribution
	std::uniform_int_distribution <> u(a, b - 1);
	return u(RD);
}


} // namespace virtual_scan
