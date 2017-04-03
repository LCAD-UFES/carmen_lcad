#ifndef CARMEN_UDATMO_PRIMITIVES_H
#define CARMEN_UDATMO_PRIMITIVES_H

#include <carmen/carmen.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>


std::ostream &operator << (std::ostream &out, const carmen_position_t &position);


std::ostream &operator << (std::ostream &out, const carmen_point_t &pose);


std::ostream &operator << (std::ostream &out, const carmen_ackerman_traj_point_t &pose);


namespace udatmo
{


template<class P, class Q> P relative_xy(const P &a, const Q &b)
{
	P t = a;
	t.x -= b.x;
	t.y -= b.y;
	return t;
}


template<class P, class Q> P relative_xyt(const P &a, const Q &b)
{
	P t = a;
	t.x -= b.x;
	t.y -= b.y;
	t.theta -= b.theta;
	return t;
}


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
 * @brief Create a buffer of given type.
 */
template<class T> T *create()
{
	return (T*) malloc(sizeof(T));
}

/**
 * @brief Create a buffer of given type.
 *
 * @param init If `true`, the buffer is initialized to all zeros before return.
 */
template<class T> T *create(bool init)
{
	size_t size = sizeof(T);
	return (T*) (init ? calloc(1, size) : malloc(size));
}

/**
 * @brief Create a buffer of given type and size (counted in units of the given type).
 *
 * @param count Size of the number, in units of parameter type `T`.
 */
template<class T> T *create(int count)
{
	return (T*) malloc(sizeof(T) * count);
}

/**
 * @brief Create a buffer of given type and size (counted in units of the given type).
 *
 * @param count Size of the number, in units of parameter type `T`.
 * @param init If `true`, the buffer is initialized to all zeros before return.
 */
template<class T> T *create(int count, bool init)
{
	size_t size = count * sizeof(T);
	return (T*) (init ? calloc(1, size) : malloc(size));
}

/**
 * @brief Clear contents of given buffer.
 */
template<class T> void clear(T *buffer)
{
	memset(buffer, 0, sizeof(T));
}

/**
 * @brief Clear contents of given buffer.
 *
 * @param size Size of the buffer, in units of type `T`.
 */
template<class T> void clear(T *buffer, size_t size)
{
	memset(buffer, 0, sizeof(T) * size);
}

/**
 * @brief Clear contents of given buffer.
 */
template<class T> void clear(T &buffer)
{
	memset(&buffer, 0, sizeof(T));
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
template<class T> void resize(int size, T *&buffer)
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

/**
 * @brief Copy the first `n` entries of the input buffer to the output buffer,
 * optionally resizing it.
 */
template<class T> void copy(bool fit, int n, const T *inputs, T *&outputs)
{
	if (fit)
		resize(n, outputs);

	memcpy(outputs, inputs, sizeof(T) * n);
}

/**
 * @brief Copy the first `n` entries of the input buffer to the output buffer.
 */
template<class T> void copy(int n, const T *inputs, T *&outputs)
{
	copy(false, n, inputs, outputs);
}

};

#endif
