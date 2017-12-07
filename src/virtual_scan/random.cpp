#include "random.h"

namespace virtual_scan
{


std::random_device RD;


int random_int(int a, int b)
{
	std::uniform_int_distribution <> u(a, b);
	return u(RD);
}


} // namespace virtual_scan
