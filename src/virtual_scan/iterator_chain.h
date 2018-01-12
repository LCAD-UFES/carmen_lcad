#ifndef VIRTUAL_SCAN_ITERATOR_CHAIN_H
#define VIRTUAL_SCAN_ITERATOR_CHAIN_H

#include <deque>
#include <initializer_list>

namespace virtual_scan
{

/**
 * @brief Iterator over a sequence of `[start, end)` iterator pairs.
 */
template<class T>
class iterator_chain
{
	/** @brief Iterator into the given collection type. */
	typedef typename T::iterator iterator;

	/** @brief Value type of the given collection type. */
	typedef typename T::value_type value_type;

	/** @brief Sequence of chained iterators. */
	typedef std::deque<iterator> Chained;

	/** @brief Sequence of iterators. */
	Chained chained;

	/** @brief Iterator being currently iterated. */
	iterator i;

	/** @brief Past-the-end iterator in the current sequence. */
	iterator n;

	void pop_front()
	{
		i = chained.front();
		chained.pop_front();

		n = chained.front();
		chained.pop_front();
	}

public:
	/**
	 * @brief Default constructor.
	 */
	iterator_chain()
	{
		// Nothing to do.
	}

	/**
	 * @brief Create a new iterator starting from the given iterator sequence.
	 */
	iterator_chain(std::initializer_list<iterator> iterators):
		chained(iterators)
	{
		pop_front();
	}

	/**
	 * @brief Check whether this iterator is different from another.
	 */
	bool operator != (const iterator_chain &that) const
	{
		return (this->i != that.i) || (this->n != that.n);
	}

	/**
	 * @brief Moves this iterator one element forward.
	 */
	iterator_chain &operator ++ ()
	{
		if (i != n)
			++i;
		else
			pop_front();

		return *this;
	}

	/**
	 * @brief Return a reference to the element currently pointed by this iterator.
	 */
	value_type &operator * ()
	{
		return *i;
	}
};

/**
 * @brief Const iterator over a sequence of `[start, end)` iterator pairs.
 */
template<class T>
class const_iterator_chain
{
	/** @brief Iterator into the given collection type. */
	typedef typename T::const_iterator const_iterator;

	/** @brief Value type of the given collection type. */
	typedef typename T::value_type value_type;

	/** @brief Sequence of chained iterators. */
	typedef std::deque<const_iterator> Chained;

	/** @brief Sequence of iterators. */
	Chained chained;

	/** @brief Iterator being currently iterated. */
	const_iterator i;

	/** @brief Past-the-end iterator in the current sequence. */
	const_iterator n;

	void pop_front()
	{
		i = chained.front();
		chained.pop_front();

		n = chained.front();
		chained.pop_front();
	}

public:
	/**
	 * @brief Default constructor.
	 */
	const_iterator_chain()
	{
		// Nothing to do.
	}

	/**
	 * @brief Create a new iterator starting from the given iterator sequence.
	 */
	const_iterator_chain(std::initializer_list<const_iterator> iterators):
		chained(iterators)
	{
		pop_front();
	}

	/**
	 * @brief Check whether this iterator is different from another.
	 */
	bool operator != (const const_iterator_chain &that) const
	{
		return (this->i != that.i) || (this->n != that.n);
	}

	/**
	 * @brief Moves this iterator one element forward.
	 */
	const_iterator_chain &operator ++ ()
	{
		if (i != n)
			++i;
		else
			pop_front();

		return *this;
	}

	/**
	 * @brief Return a reference to the element currently pointed by this iterator.
	 */
	const value_type &operator * ()
	{
		return *i;
	}
};

} // namespace virtual_scan

#endif
