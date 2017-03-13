#include "udatmo_memory.h"


void *udatmo_resize(void *buffer, size_t size)
{
	void *resized = realloc(buffer, size);
	return (resized != NULL ? resized : buffer);
}
