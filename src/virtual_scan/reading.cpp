#include "reading.h"

namespace virtual_scan
{


bool Reading::iterator::operator != (const iterator &that) const
{
	return false;
}


Reading::iterator &Reading::iterator::operator ++ ()
{
	return *this;
}


carmen_point_t &Reading::iterator::operator * ()
{
	static carmen_point_t point;
	return point;
}


bool Reading::const_iterator::operator != (const const_iterator &that) const
{
	return false;
}


Reading::const_iterator &Reading::const_iterator::operator ++ ()
{
	return *this;
}


const carmen_point_t &Reading::const_iterator::operator * () const
{
	static carmen_point_t point;
	return point;
}



Reading::iterator Reading::begin()
{
	return iterator();
}


/**
 * @brief Return an iterator at the beggining of this reading.
 */
Reading::const_iterator Reading::begin() const
{
	return const_iterator();
}


/**
 * @brief Return the past-the-end iterator.
 */
Reading::iterator Reading::end()
{
	return iterator();
}


/**
 * @brief Return the past-the-end iterator.
 */
Reading::const_iterator Reading::end() const
{
	return const_iterator();
}


/**
 * @brief Erase the given point from this reading.
 */
void Reading::erase(const carmen_point_t &point)
{
}


/**
 * @brief Add the given point to this reading.
 */
void Reading::insert(const carmen_point_t &point)
{
}


/**
 * @brief Generate a reading containing only the rays in the range of the given obstacle.
 */
Reading Reading::range(const ObstaclePose &pose) const
{
	return Reading();
}


/**
 * @brief Return the number of rays in this reading.
 */
size_t Reading::size() const
{
	return 0;
}


} // namespace virtual_scan
