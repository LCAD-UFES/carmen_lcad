/*
 * ring_buffer.c
 *
 *  Created on: 14/06/2012
 *      Author: filipe
 */

#include <stdio.h>
#include <malloc.h>
#include "ring_buffer.h"

void cbInit(CircularBuffer *cb, int size)
{
	cb->size  = size + 1; /* include empty elem */
	cb->start = 0;
	cb->end   = 0;
	cb->elems = (short *)calloc(cb->size, sizeof(short));
}

void cbFree(CircularBuffer *cb)
{
	free(cb->elems); /* OK if null */
}

int cbIsFull(CircularBuffer *cb)
{
	return (cb->end + 1) % cb->size == cb->start;
}

int cbIsEmpty(CircularBuffer *cb)
{
	return cb->end == cb->start;
}

/* Write an element, overwriting oldest element if buffer is full. App can
   choose to avoid the overwrite by checking cbIsFull(). */
void cbWrite(CircularBuffer *cb, short elem)
{
	cb->elems[cb->end] = elem;
	cb->end = (cb->end + 1) % cb->size;

	if (cb->end == cb->start)
		cb->start = (cb->start + 1) % cb->size; /* full, overwrite */
}

/* Write an element, overwriting oldest element if buffer is full. App can
   choose to avoid the overwrite by checking cbIsFull(). */
void cbWriteBlock(CircularBuffer *cb, short *elem, int length)
{
	int i;

	for(i = 0; i < length; i++)
		cbWrite(cb, elem[i]);
}

/* Read oldest element. App must ensure !cbIsEmpty() first. */
void cbRead(CircularBuffer *cb, short *elem)
{
	*elem = cb->elems[cb->start];
	cb->start = (cb->start + 1) % cb->size;
}

void cbReadBlock(CircularBuffer *cb, short *elem, int length)
{
	int i;

	for(i = 0; i < length; i++)
		cbRead(cb, elem + i);
}

int cbAvaiableSpace (CircularBuffer *cb)
{
	if (cb->end >= cb->start)
		return cb->start + (cb->size - cb->end - 1); // -1 for empty element
	else
		return (cb->start - cb->end);
}
