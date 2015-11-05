/*
 * ring_buffer.h
 *
 *  Created on: 14/06/2012
 *      Author: filipe
 */

#ifndef RING_BUFFER_H_
#define RING_BUFFER_H_

/*
 * ring_buffer.c
 *
 *  Created on: 14/06/2012
 *      Author: filipe
 */

#include <stdio.h>
#include <malloc.h>

/* Circular buffer object */
typedef struct
{
	int size; /* maximum number of elements */
	int start; /* index of oldest element */
	int end; /* index at which to write new element */
	short *elems;  /* vector of elements */
} CircularBuffer;

void cbInit(CircularBuffer *cb, int size);
void cbFree(CircularBuffer *cb);
void cbRead(CircularBuffer *cb, short *elem);
void cbWrite(CircularBuffer *cb, short elem);
void cbReadBlock(CircularBuffer *cb, short *elem, int length);
void cbWriteBlock(CircularBuffer *cb, short *elem, int length);
int cbAvaiableSpace (CircularBuffer *cb);
int cbIsFull(CircularBuffer *cb);
int cbIsEmpty(CircularBuffer *cb);

#endif /* RING_BUFFER_H_ */
