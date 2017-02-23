#include "primitives.h"

namespace udatmo
{

void *resize(void *buffer, size_t size)
{
	void *resized = realloc(buffer, size);
	return (resized != NULL ? resized : buffer);
}

} // namespace udatmo
