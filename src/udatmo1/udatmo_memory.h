#ifndef _CARMEN_UDATMO_MEMORY_H_
#define _CARMEN_UDATMO_MEMORY_H_


#ifdef __cplusplus
extern "C"
{
#endif


#include <stdlib.h>


void *udatmo_resize(void *buffer, size_t size);


#define CREATE(TYPE) (TYPE*) malloc(sizeof(TYPE))


#define RESIZE(BUFFER, TYPE, N) BUFFER = (TYPE*) udatmo_resize(BUFFER, sizeof(TYPE) * N)


#define DELETE(BUFFER) if (BUFFER != NULL) do {free(BUFFER); BUFFER = NULL;} while (0)


#ifdef __cplusplus
}
#endif


#endif
