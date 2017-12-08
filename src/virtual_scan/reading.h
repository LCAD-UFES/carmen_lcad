#ifndef VIRTUAL_SCAN_READING_H
#define VIRTUAL_SCAN_READING_H

#include "obstacle.h"

namespace virtual_scan
{

class Reading
{
public:
	class iterator
	{
	public:
		bool operator != (const iterator &that) const;

		iterator &operator ++ ();

		carmen_point_t &operator * ();
	};

	class const_iterator
	{
	public:
		bool operator != (const const_iterator &that) const;

		const_iterator &operator ++ ();

		const carmen_point_t &operator * () const;
	};

	/** @brief Observer pose at the time this reading was collected. */
	carmen_point_t origin;

	/** @brief Time when this reading was collected. */
	double timestamp;

	/**
	 * @brief Return an iterator at the beggining of this reading.
	 */
	iterator begin();

	/**
	 * @brief Return an iterator at the beggining of this reading.
	 */
	const_iterator begin() const;

	/**
	 * @brief Return the past-the-end iterator.
	 */
	iterator end();

	/**
	 * @brief Return the past-the-end iterator.
	 */
	const_iterator end() const;

	/**
	 * @brief Erase the given point from this reading.
	 */
	void erase(const carmen_point_t &point);

	/**
	 * @brief Add the given point to this reading.
	 */
	void insert(const carmen_point_t &point);

	/**
	 * @brief Generate a reading containing only the rays in the range of the given obstacle.
	 */
	Reading range(const ObstaclePose &pose) const;

	/**
	 * @brief Return the number of rays in this reading.
	 */
	size_t size() const;
};

} // namespace virtual_scan

#endif
