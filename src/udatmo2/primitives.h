#ifndef CARMEN_UDATMO_PRIMITIVES_H
#define CARMEN_UDATMO_PRIMITIVES_H

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace udatmo
{

/**
 * @brief Compute the slope angle of the segment between points `(x1, y1)` and `(x2, y2)`.
 */
inline double angle(double x1, double y1, double x2, double y2)
{
	return atan2(y2 - y1, x2 - x1);
}

/**
 * @brief Compute the slope angle of the segment between points `a` and `b`.
 *
 * Types `P` and `Q` are expected to have two numeric fields `x` and `y`.
 */
template<class P, class Q> double angle(const P &a, const Q &b)
{
	return angle(a.x, a.y, b.x, b.y);
}

/**
 * @brief Compute the distance between points `(x1, y1)` and `(x2, y2)`.
 */
inline double distance(double x1, double y1, double x2, double y2)
{
	double dx = x1 - x2;
	double dy = y1 - y2;
	return sqrt(dx * dx + dy * dy);
}

/**
 * @brief Compute the distance between points `a` and `b`.
 *
 * Types `P` and `Q` are expected to have two numeric fields `x` and `y`.
 */
template<class P, class Q> double distance(const P &a, const Q &b)
{
	return distance(a.x, a.y, b.x, b.y);
}

/**
 * @brief Return `value` if within the range `[a, b]`, otherwise the appropriate range limit.
 */
template<class T> T truncate(const T &value, const T &a, const T &b)
{
	return std::min(std::max(a, value), b);
}

/**
 * @brief Create a buffer of given type and size (counted in units of the given type).
 */
template<class T> T *create(int count = 1)
{
	return (T*) malloc(sizeof(T) * count);
}

/**
 * @brief Deallocate the given buffer, setting the pointer variable to `NULL`.
 *
 * If the pointer is already null then nothing is done.
 */
template<class T> void destroy(T *&buffer)
{
	if (buffer != NULL)
	{
		free(buffer);
		buffer = NULL;
	}
}

/**
 * @brief Resize the given buffer.
 *
 * This function differs from the standard `realloc()` function in that its return
 * value is always a valid buffer pointer -- either the original argument pointer
 * (if the buffer was shrink or could be expanded) or a new one (if the buffer had
 * to be replaced on expansion).
 *
 * @return Pointer to the (possibly resized) buffer.
 */
template<class T> void resize(T *&buffer, int size)
{
	if (buffer == NULL && size > 0)
		buffer = create<T>(size);
	else if (size == 0)
	{
		if (buffer != NULL)
			destroy(buffer);
	}
	else
	{
		T *resized = (T*) realloc(buffer, sizeof(T) * size);
		if (resized != NULL)
			buffer = resized;
	}
}

};

#endif
