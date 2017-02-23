#ifndef CARMEN_UDATMO_PRIMITIVES_H
#define CARMEN_UDATMO_PRIMITIVES_H

#include <algorithm>
#include <cmath>
#include <cstdlib>

#define CREATE(TYPE) (TYPE*) malloc(sizeof(TYPE))

#define RESIZE(BUFFER, TYPE, N) BUFFER = (TYPE*) udatmo::resize(BUFFER, sizeof(TYPE) * N)

#define DELETE(BUFFER) if (BUFFER != NULL) do {free(BUFFER); BUFFER = NULL;} while (0)

namespace udatmo
{

/**
 * @brief Compute the slope angle of the segment between points `(x1, y1)` and `(x2, y2)`.
 */
inline double angle(double x1, double y1, double x2, double y2)
{
	return atan2(y1 - y2, x1 - x2);
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
 * @brief Resize the given buffer.
 *
 * This function differs from the standard `realloc()` function in that its return
 * value is always a valid buffer pointer -- either the original argument pointer
 * (if the buffer was shrink or could be expanded) or a new one (if the buffer had
 * to be replaced on expansion).
 *
 * @return Pointer to the (possibly resized) buffer.
 */
void *resize(void *buffer, size_t size);

};

#endif
