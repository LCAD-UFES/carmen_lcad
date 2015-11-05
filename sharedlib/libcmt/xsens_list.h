#ifndef XSENS_MONOLITHIC
/*! \file
	\brief List class interface for use in CMT

	The implementations of the functions are in \ref Cmtlist.hpp which is automatically
	included so they are inlined.

	This file requires xsens_janitors.h. When a library header is built with this file,
	make sure xsens_janitors.h is included before this file.

	This file requires xsens_math.h if the math switch is set to 1. When a library header
	is built with this file, make sure xsens_math.h is included before this file.

	\section FileCopyright Copyright Notice
	Copyright (C) Xsens Technologies B.V., 2006.  All rights reserved.

	This source code is intended for use only by Xsens Technologies BV and
	those that have explicit written permission to use it from
	Xsens Technologies BV.

	THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
	KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
	IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
	PARTICULAR PURPOSE.

	\section FileSwitches Switches
	\li	\c _XSENS_LIST_WITH_MATH	Check whether to include math files when set to 1.
	\li	\c _XSENS_LIST_IO			Include code for string and file I/O when set to 1.
*/
#endif
#ifndef XSENS_LIST_H
#define XSENS_LIST_H

#ifndef XSENS_MONOLITHIC
#ifndef PSTDINT_H_INCLUDED
#	include "pstdint.h"
#endif
#endif

#ifndef XSENS_EXCEPTION_H
#include "xsens_exception.h"
#endif

#ifndef XSENS_JANITORS_H
#	include "xsens_janitors.h"
#endif


#define XSENS_LIST_NOTFOUND	0xFFFFFFFF

#ifdef _XSENS_LIST_RANGE_CHECKS
#	ifdef _MSC_VER
#		define XSENS_LIST_THROW throw(...)
#	else
#		define XSENS_LIST_THROW
#	endif
#else
#	define XSENS_LIST_THROW
#endif

#include <stdlib.h>
#include <malloc.h>
#include <string.h>

namespace xsens {

#define XSENS_LIST_LINEAR_SEARCH_TRESHOLD	10

	/*! \brief Dynamic list class

		This class can store items of the given type. If the type supports the < operator
		it can also be sorted.
		Items in the list can be accessed through the [] operator or the get() function.

		Do NOT use any item type that requires a constructor to work correctly. Pointers to these
		objects can work though.
	*/
	template <typename T>
	class List
	{
	private:
		void operator = (const List& list);	//!< intentionally NOT implemented due to ambiguous nature
			//! Sorts the list in an ascending order, using the T::< operator.
		void qSort(uint32_t left, uint32_t right);
			//! Sorts the list in an ascending order, using the T::< operator on dereferenced list items.
		void qSortDeref(uint32_t left, uint32_t right);

	protected:
		T* m_data;							//!< The array containing the items
		uint32_t m_max;				//!< The current size of the data array
		uint32_t m_count;				//!< The number of items currently in the list
		JanitorClassFunc<List<T> >* m_jcf;	//!< Used to clean up the list on exit
		bool m_manage;

			//! Construct a list as a reference to a raw list
		List(const uint32_t size, T* src, bool manage);
	public:

			//! A comparison function type, should return -1, 0 or 1 for <, == and >
		typedef int32_t (*cmpFunc) (const T&,const T&);

			//! Standard constructor, creates an empty list with some room for items.
		List();
			//! Construct a list with a capacity of at least the given size.
		List(const uint32_t size);
			//! Construct a list as a direct copy of another list
		List(const List<T>& src);
			//! Construct a list as a copy of a raw list
		List(const uint32_t size, const T* src);
			//! Destroy the list. This does NOT automatically delete items IN the list.
		~List();

			//! Calls delete for all items in the list and then clears the list.
		void deleteAndClear(void);
			//! Calls free for all items in the list and then clears the list.
		void freeAndClear(void);
			//! Clears the list without explicitly deleting anything.
		void clear(void);
			//! Resizes the list to at least the given size.
		void resize(uint32_t newSize);
			//! Adds an item to the end of the list.
		void append(const T& item);
			//! Adds a number of items to the end of the list.
		void appendList(uint32_t count, const T* lst);
			//! Adds the contents of the source list to the end of the list.
		void appendDeepCopy(const List<T>& source);
			//! Adds the contents of the source list to the end of the list.
		void appendShallowCopy(const List<T>& source);
			//! Adds a copy of a referenced item to the end of the list using newItem = new TB(item).
			template <typename TB>
		void appendCopy(const TB& item);
			//! Adds a related item to the end of the list, using the T = TR operator.
			template <typename TR>
		void appendRelated(const TR& item);
			//! Removes an item at the given index in the list.
		void remove(const uint32_t index) XSENS_LIST_THROW;
			//! Swaps two items in the list.
		void swap(const uint32_t i, const uint32_t j) XSENS_LIST_THROW;
			//! Removes an item at the given index in the list.
		void deleteAndRemove(const uint32_t index) XSENS_LIST_THROW;
			//! Removes an item at the given index in the list.
		void freeAndRemove(const uint32_t index) XSENS_LIST_THROW;
			//! Retrieves the last item.
		T& last(void) const XSENS_LIST_THROW;
			//! Retrieves the smallest item, using the T::< operator.
		T& minVal(void) const XSENS_LIST_THROW;
			//! Retrieves the largest item, using the T::< operator.
		T& maxVal(void) const XSENS_LIST_THROW;
			//! Retrieves the item at the given index. An index beyond the end returns the first item.
		T& get(const uint32_t index) const XSENS_LIST_THROW;
			//! Retrieves the item at the given index. An index beyond the end probably causes an exception.
		T& operator [] (const uint32_t index) const XSENS_LIST_THROW;
			//! Inserts an item at the given index, shifting any items below it down one spot.
		void insert(const T& item, const uint32_t index);
			//! Inserts a copy of the referenced item at the given index, shifting any items below it down one spot.
			template <typename TB>
		void insertCopy(const TB& item, const uint32_t index);
			//! Assumes the list is sorted and inserts the item at the appropriate spot.
		uint32_t insertSorted(const T& item);
			//! Assumes the list is sorted by dereferenced values and inserts the item at the appropriate spot.
		uint32_t insertSortedDeref(const T& item);
			//! Assumes the list is sorted and inserts a copy of the referenced item at the appropriate spot.
			template <typename TB>
		uint32_t insertSortedCopy(const TB& item);
			//! Returns the number of items currently in the list.
		uint32_t length(void) const { return m_count; }
			//! Returns the number of items currently in the list.
		uint32_t count(void) const { return m_count; }
			//! Sorts the list in an ascending order, using the T::< operator.
		void sortAscending(void);
			//! Sorts the list in an ascending order, using the T::< operator on dereferenced list items.
		void sortAscendingDeref(void);
			//! Sorts the first list in an ascending order, using the T::< operator, the second list will be updated the same way.
			template <typename T2>
		void twinSortAscending(List<T2>& twin);
			//! Finds an item in an unsorted list (walk over all items) using the T::== operator
			template <typename TB>
		uint32_t find(const TB& item) const;
			//! Finds an item in an unsorted list (walk over all items) using the T::== operator on dereferenced list items
			template <typename TB>
		uint32_t findDeref(const TB& item) const;
			//! Finds an item in a sorted list (binary search) using the T::== and T::< operators
			template <typename TB>
		uint32_t findSorted(const TB& item) const;
			//! Finds an item in a sorted list (binary search) using the T::== and T::< operators on dereferenced list items
			template <typename TB>
		uint32_t findSortedDeref(const TB& item) const;

			//! Finds an item in a sorted list (binary search) using the T::== and T::< operators. If not found, it does not return XSENS_LIST_NOTFOUND but the insert position if this item would be inserted in the list.
			template <typename TB>
		uint32_t findSortedForInsert(const TB& item) const;
			//! Finds an item in a sorted list (binary search) using the T::== and T::< operators on dereferenced list items. If not found, it does not return XSENS_LIST_NOTFOUND but the insert position if this item would be inserted in the list.
			template <typename TB>
		uint32_t findSortedDerefForInsert(const TB& item) const;

			//! Finds an item in an unsorted list (walk over all items) using the T::== operator, starting at the end of the list
			template <typename TB>
		uint32_t reverseFind(const TB& item) const;
			//! Finds an item in an unsorted list (walk over all items) using the T::== operator on dereferenced list items, starting at the end of the list
			template <typename TB>
		uint32_t reverseFindDeref(const TB& item) const;

			//! Reverse the order of the list, useful for sorted lists that are read/created in the reverse order
		void reverse(void);
			//! Removes items from the end of the list.
		void removeTail(const uint32_t count) XSENS_LIST_THROW;
		void deleteAndRemoveTail(const uint32_t count) XSENS_LIST_THROW;
		void freeAndRemoveTail(const uint32_t count) XSENS_LIST_THROW;

			//! Type for an equality compare function, should return true when NOT equal
		typedef int32_t (__cdecl * InequalityFunction)(const T&, const T&);
			//! Finds an item in an unsorted list (walk over all items) using the given inequality function
		uint32_t find(const T item, InequalityFunction fnc) const;

		void deleteItemsOnDestroy(void);
		void freeItemsOnDestroy(void);

			//! Removes any duplicate entries and returns the number of items removed. Items are compared directly.
		uint32_t removeDuplicateEntries(void);
			//! Removes any duplicate entries and returns the number of items removed. Items are compared after dereferencing.
		uint32_t removeDuplicateEntriesDeref(void);

			//! Make a copy of the list, duplicating list items i with: copy[i] = new TB(*source[i])
			template <typename TB>
		void isDeepCopyOf(const List<T>& source);
			//! Overwrites the current list with a direct copy (a=b) of another list.
		void isShallowCopyOf(const List<T>& source);

			//! Returns the start of the linear data buffer
		const T* getBuffer(void) const { return m_data; }
			//! Compare each item of the lists using the T == TB operator. If they're all identical, returns true.
			template <typename TB>
		bool operator == (const List<TB>& lst);
	};


template <typename T>
List<T>::List()
{
	m_max = 16;
	m_count = 0;
	m_data = (T*) malloc(m_max * sizeof(T));
	if (!m_data)
		throw std::bad_alloc();
	m_jcf = NULL;
	m_manage = true;
}

template <typename T>
List<T>::List(uint32_t size)
{
	m_max = size;
	if (m_max == 0)
		m_max = 1;
	m_count = 0;
	m_data = (T*) malloc(m_max * sizeof(T));
	if (!m_data)
		throw std::bad_alloc();
	m_jcf = NULL;
	m_manage = true;
}

template <typename T>
List<T>::List(const List<T>& src)
{
	m_max = src.m_max;
	if (m_max == 0)
		m_max = 1;
	m_count = src.m_count;
	m_data = (T*) malloc(m_max * sizeof(T));
	if (!m_data)
		throw std::bad_alloc();
	m_jcf = NULL;
	if (m_count > 0)
		memcpy(m_data,src.m_data,m_count*sizeof(T));
	m_manage = true;
}

template <typename T>
List<T>::List(const uint32_t size, const T* src)
{
	m_max = size;
	if (m_max == 0)
		m_max = 1;
	m_count = size;
	m_data = (T*) malloc(m_max * sizeof(T));
	if (!m_data)
		throw std::bad_alloc();
	m_jcf = NULL;
	if (m_count > 0)
		memcpy(m_data,src,m_count * sizeof(T));
	m_manage = true;
}

template <typename T>
List<T>::List(const uint32_t size, T* src, bool manage)
{
	m_max = size;
	m_count = size;
	m_data = src;
	m_jcf = NULL;
	m_manage = manage;
}

template <typename T>
List<T>::~List()
{
	if (m_jcf != NULL)
		delete m_jcf;
	if (m_manage && m_data != NULL)
		free(m_data);
	m_jcf = NULL;
	m_data = NULL;
}

template <typename T>
void List<T>::deleteAndClear(void)
{
	for (unsigned i=0;i<m_count;++i)
		delete m_data[i];
	m_count = 0;
}

template <typename T>
void List<T>::freeAndClear(void)
{
	for (unsigned i=0;i<m_count;++i)
		free(m_data[i]);
	m_count = 0;
}

template <typename T>
void List<T>::clear(void)
{
	m_count = 0;
}

template <typename T>
void List<T>::resize(uint32_t newSize)
{
	if (m_manage)
	{
		if (newSize == m_max)
			return;
		if (m_count > newSize)
			m_max = m_count;
		else
			m_max = newSize;
		if (m_max == 0)
			m_max = 1;	// size 0 is not allowed

		m_data = (T*) realloc(m_data,m_max * sizeof(T));
	}
}

template <typename T>
void List<T>::append(const T& item)
{
	if (m_count == m_max)
		resize(m_max + 1 + m_max/2);
	m_data[m_count++] = item;
}

template <typename T>
void List<T>::appendList(uint32_t count, const T* lst)
{
	if (m_count+count > m_max)
		resize(m_max + count + m_max/2);
	for (unsigned i = 0; i < count; ++i)
		m_data[m_count++] = lst[i];
}

template <typename T>
void List<T>::appendDeepCopy(const List<T>& source)
{
	if (m_max < source.m_count + m_count)
		resize(source.m_count + m_count);

	for (uint32_t i = 0;i<source.m_count;++i)
		m_data[m_count++] = new T(*source.m_data[i]);
}

template <typename T>
void List<T>::appendShallowCopy(const List<T>& source)
{
	if (m_max < source.m_count + m_count)
		resize(source.m_count + m_count);

	for (uint32_t i = 0;i<source.m_count;++i)
		m_data[m_count++] = source.m_data[i];
}

template <typename T>
template <typename TB>
void List<T>::appendCopy(const TB& item)
{
	if (m_count == m_max)
		resize(m_max + 1 + m_max/2);
	m_data[m_count++] = new TB(item);
}

template <typename T>
template <typename TR>
void List<T>::appendRelated(const TR& item)
{
	if (m_count == m_max)
		resize(m_max + 1 + m_max/2);
	m_data[m_count++] = item;
}

template <typename T>
T& List<T>::last(void) const XSENS_LIST_THROW
{
	#ifdef _XSENS_LIST_RANGE_CHECKS
		if (m_count == 0)
			throw Exception("List.last: empty list");
	#endif
	return m_data[m_count-1];
}

template <typename T>
T& List<T>::maxVal(void) const XSENS_LIST_THROW
{
	#ifdef _XSENS_LIST_RANGE_CHECKS
		if (m_count == 0)
			throw Exception("List.maxVal: empty list");
	#endif
	T* item = &m_data[0];
	for (uint32_t i = 1; i < m_count; ++i)
		if (*item < m_data[i])
			item = &m_data[i];
	return *item;
}

template <typename T>
T& List<T>::minVal(void) const XSENS_LIST_THROW
{
	#ifdef _XSENS_LIST_RANGE_CHECKS
		if (m_count == 0)
			throw Exception("List.minVal: empty list");
	#endif
	T* item = &m_data[0];
	for (uint32_t i = 1; i < m_count; ++i)
		if (m_data[i] < *item)
			item = &m_data[i];
	return *item;
}

template <typename T>
T& List<T>::get(const uint32_t index) const XSENS_LIST_THROW
{
	#ifdef _XSENS_LIST_RANGE_CHECKS
		if (index >= m_count)
			throw Exception("List.get: index out of bounds");
	#endif
	if (index >= m_count)
		return m_data[m_count-1];
	return m_data[index];
}

template <typename T>
void List<T>::insert(const T& item, const uint32_t index)
{
	if (m_count == m_max)
		resize(1 + m_max + (m_max >> 1));
	for (unsigned i=m_count;i>index;--i)
		m_data[i] = m_data[i-1];
	if (index <= m_count)
		m_data[index] = item;
	else
		m_data[m_count] = item;
	m_count++;
}

template <typename T>
template <typename TB>
void List<T>::insertCopy(const TB& item, const uint32_t index)
{
	if (m_count == m_max)
		resize(1 + m_max + (m_max >> 1));
	for (unsigned i=m_count;i>index;--i)
		m_data[i] = m_data[i-1];
	if (index <= m_count)
		m_data[index] = new TB(item);
	else
		m_data[m_count] = new TB(item);
	m_count++;
}

template <typename T>
T& List<T>::operator [] (const uint32_t index) const XSENS_LIST_THROW
{
	#ifdef _XSENS_LIST_RANGE_CHECKS
		if (index >= m_count)
			throw Exception("List[]: index out of bounds");
	#endif
	return m_data[index];
}


template <typename T>
void List<T>::qSort(uint32_t left, uint32_t right)
{
	uint32_t l_hold, r_hold;
	T pivot;

	l_hold = left;
	r_hold = right;
	pivot = m_data[left];
	while (left < right)
	{
		while (!(m_data[right] < pivot) && (left < right))
			right--;
		if (left != right)
		{
			m_data[left] = m_data[right];
			left++;
		}
		while (!(pivot < m_data[left]) && (left < right))
			left++;
		if (!(left == right))
		{
			m_data[right] = m_data[left];
			right--;
		}
	}
	m_data[left] = pivot;
	if (l_hold < left)
		qSort(l_hold, left-1);
	if (r_hold > left)
		qSort(left+1, r_hold);
}

template <typename T>
void List<T>::qSortDeref(uint32_t left, uint32_t right)
{
	uint32_t l_hold, r_hold;
	T pivot;

	l_hold = left;
	r_hold = right;
	pivot = m_data[left];
	while (left < right)
	{
		while (!(*m_data[right] < *pivot) && (left < right))
			right--;
		if (left != right)
		{
			m_data[left] = m_data[right];
			left++;
		}
		while (!(*pivot < *m_data[left]) && (left < right))
			left++;
		if (!(left == right))
		{
			m_data[right] = m_data[left];
			right--;
		}
	}
	m_data[left] = pivot;
	if (l_hold < left)
		qSortDeref(l_hold, left-1);
	if (r_hold > left)
		qSortDeref(left+1, r_hold);
}

template <typename T>
void List<T>::sortAscending(void)
{
	if (m_count <= 1)
		return;
	struct Linker {
		Linker *prev, *next;
		uint32_t index;

		T item;
	};

	Linker* list = (Linker*) malloc(m_count*sizeof(Linker));
	if (!list)
		throw std::bad_alloc();

	list[0].prev = NULL;
	list[0].next = NULL;
	list[0].index = 0;
	list[0].item = m_data[0];

	Linker* curr = list;

	for (uint32_t i = 1; i < m_count; ++i)
	{
		list[i].index = i;
		list[i].item = m_data[i];
		if (m_data[i] < m_data[curr->index])
		{
			while (curr->prev != NULL)
			{
				curr = curr->prev;
				if (!(m_data[i] < m_data[curr->index]))
				{
					// insert after this
					list[i].next = curr->next;
					list[i].prev = curr;
					curr->next->prev = &list[i];
					curr->next = &list[i];
					curr = &list[i];
					break;
				}
			}
			if (curr != &list[i])
			{
				list[i].prev = NULL;
				list[i].next = curr;
				curr->prev = &list[i];
				curr = &list[i];
			}
		}
		else
		{
			while (curr->next != NULL)
			{
				curr = curr->next;
				if (m_data[i] < m_data[curr->index])
				{
					// insert before this
					list[i].next = curr;
					list[i].prev = curr->prev;
					curr->prev->next = &list[i];
					curr->prev = &list[i];
					curr = &list[i];
					break;
				}
			}
			if (curr != &list[i])
			{
				list[i].prev = curr;
				list[i].next = NULL;
				curr->next = &list[i];
				curr = &list[i];
			}
		}
	}

	// go to start of list
	while (curr->prev != NULL) curr = curr->prev;

	// copy sorted list back
	for (uint32_t i = 0; i < m_count; ++i)
	{
		m_data[i] = curr->item;
		curr = curr->next;
	}

	free(list);
}

template <typename T>
void List<T>::sortAscendingDeref(void)
{
	if (m_count <= 1)
		return;
	//! \todo remove embedded struct definition
	struct Linker {
		Linker *prev, *next;
		uint32_t index;

		T item;
	};

	Linker* list = (Linker*) malloc(m_count*sizeof(Linker));
	if (!list)
		throw std::bad_alloc();

	list[0].prev = NULL;
	list[0].next = NULL;
	list[0].index = 0;
	list[0].item = m_data[0];

	Linker* curr = list;

	for (uint32_t i = 1; i < m_count; ++i)
	{
		list[i].index = i;
		list[i].item = m_data[i];
		if (*m_data[i] < *m_data[curr->index])
		{
			while (curr->prev != NULL)
			{
				curr = curr->prev;
				if (!(*m_data[i] < *m_data[curr->index]))
				{
					// insert after this
					list[i].next = curr->next;
					list[i].prev = curr;
					curr->next->prev = &list[i];
					curr->next = &list[i];
					curr = &list[i];
					break;
				}
			}
			if (curr != &list[i])
			{
				list[i].prev = NULL;
				list[i].next = curr;
				curr->prev = &list[i];
				curr = &list[i];
			}
		}
		else
		{
			while (curr->next != NULL)
			{
				curr = curr->next;
				if (*m_data[i] < *m_data[curr->index])
				{
					// insert before this
					list[i].next = curr;
					list[i].prev = curr->prev;
					curr->prev->next = &list[i];
					curr->prev = &list[i];
					curr = &list[i];
					break;
				}
			}
			if (curr != &list[i])
			{
				list[i].prev = curr;
				list[i].next = NULL;
				curr->next = &list[i];
				curr = &list[i];
			}
		}
	}

	// go to start of list
	while (curr->prev != NULL) curr = curr->prev;

	// copy sorted list back
	for (uint32_t i = 0; i < m_count; ++i)
	{
		m_data[i] = curr->item;
		curr = curr->next;
	}

	free(list);
}

template <typename T>
template <typename T2>
void List<T>::twinSortAscending(List<T2>& twin)
{
	if (m_count <= 1)
		return;

	#ifdef _XSENS_LIST_RANGE_CHECKS
		if (m_count != twin.m_count)
			throw Exception("List.twinSortAscending: sizes do not match");
	#endif
	uint32_t iteration = 0;
	uint32_t mini;
	T tmp;
	T2 tmp2;
	if (m_count > 1)
	while (iteration < m_count-1)
	{
		mini = iteration;
		for (uint32_t i=iteration+1;i<m_count;++i)
		{
			if (m_data[i] < m_data[mini])
				mini = i;
		}
		if (mini != iteration)
		{
			tmp = m_data[mini];
			m_data[mini] = m_data[iteration];
			m_data[iteration] = tmp;

			tmp2 = twin.m_data[mini];
			twin.m_data[mini] = twin.m_data[iteration];
			twin.m_data[iteration] = tmp2;
		}
		++iteration;
	}
}

template <typename T>
void List<T>::remove(const uint32_t index) XSENS_LIST_THROW
{
	#ifdef _XSENS_LIST_RANGE_CHECKS
		if (index >= m_count)
			throw Exception("List.remove: index out of bounds");
	#endif
	if (index == m_count-1)
	{
		--m_count;
		return;
	}
	--m_count;
	for (unsigned i = index;i < m_count;++i)
		m_data[i] = m_data[i+1];
}

template <typename T>
void List<T>::removeTail(const uint32_t count) XSENS_LIST_THROW
{
	#ifdef _XSENS_LIST_RANGE_CHECKS
		if (count > m_count)
			throw Exception("List.removeTail: list size less than remove count");
	#endif
	if (m_count > count)
	{
		m_count -= count;
		return;
	}
	m_count = 0;
}

template <typename T>
void List<T>::deleteAndRemoveTail(const uint32_t count) XSENS_LIST_THROW
{
	#ifdef _XSENS_LIST_RANGE_CHECKS
		if (count > m_count)
			throw Exception("List.deleteAndRemoveTail: list size less than remove count");
	#endif
	if (m_count > count)
	{
		for (unsigned i = 0;i < count;++i)
			delete m_data[--m_count];
		return;
	}
	deleteAndClear();
}

template <typename T>
void List<T>::freeAndRemoveTail(const uint32_t count) XSENS_LIST_THROW
{
	#ifdef _XSENS_LIST_RANGE_CHECKS
		if (count > m_count)
			throw Exception("List.freeAndRemoveTail: list size less than remove count");
	#endif
	if (m_count > count)
	{
		for (unsigned i = 0;i < count;++i)
			free(m_data[--m_count]);
		return;
	}
	freeAndClear();
}

template <typename T>
uint32_t List<T>::removeDuplicateEntries(void)
{
	uint32_t removed = 0;
	for (uint32_t i=0;i < m_count; ++i)
	{
		for (uint32_t j=i+1;j < m_count; ++j)
		{
			if (m_data[i] == m_data[j])
			{
				remove(j);
				++removed;
				--j;
			}
		}
	}
	return removed;
}

template <typename T>
uint32_t List<T>::removeDuplicateEntriesDeref(void)
{
	uint32_t removed = 0;
	for (uint32_t i=0;i < m_count; ++i)
	{
		for (uint32_t j=i+1;j < m_count; ++j)
		{
			if (*(m_data[i]) == *(m_data[j]))
			{
				remove(j);
				++removed;
				--j;
			}
		}
	}
	return removed;
}

template <typename T>
void List<T>::deleteAndRemove(const uint32_t index) XSENS_LIST_THROW
{
	#ifdef _XSENS_LIST_RANGE_CHECKS
		if (index >= m_count)
			throw Exception("List.deleteAndRemove: index out of bounds");
	#endif
	delete m_data[index];
	if (index == m_count-1)
	{
		--m_count;
		return;
	}
	--m_count;
	for (unsigned i = index;i < m_count;++i)
		m_data[i] = m_data[i+1];
}

template <typename T>
void List<T>::freeAndRemove(const uint32_t index) XSENS_LIST_THROW
{
	#ifdef _XSENS_LIST_RANGE_CHECKS
		if (index >= m_count)
			throw Exception("List.freeAndRemove: index out of bounds");
	#endif
	free(m_data[index]);
	if (index == m_count-1)
	{
		--m_count;
		return;
	}
	--m_count;
	for (unsigned i = index;i < m_count;++i)
		m_data[i] = m_data[i+1];
}

template <typename T>
template <typename TB>
uint32_t List<T>::find(const TB& item) const
{
	for (uint32_t i=0;i<m_count;++i)
		if (((const T*)m_data)[i] == item)
			return i;
	return XSENS_LIST_NOTFOUND;
}

template <typename T>
uint32_t List<T>::find(const T item, InequalityFunction fnc) const
{
	for (uint32_t i=0;i<m_count;++i)
		if (!fnc(m_data[i],item))
			return i;
	return XSENS_LIST_NOTFOUND;
}

template <typename T>
template <typename TB>
uint32_t List<T>::findDeref(const TB& item) const
{
	for (uint32_t i=0;i<m_count;++i)
		if (*(m_data[i]) == item)
			return i;
	return XSENS_LIST_NOTFOUND;
}

template <typename T>
template <typename TB>
uint32_t List<T>::reverseFind(const TB& item) const
{
	// comparison is ok due to unsigned-ness of i
	for (uint32_t i=m_count-1 ; i<m_count ; --i)
		if (((const T*)m_data)[i] == item)
			return i;
	return XSENS_LIST_NOTFOUND;
}

template <typename T>
template <typename TB>
uint32_t List<T>::reverseFindDeref(const TB& item) const
{
	// comparison is ok due to unsigned-ness of i
	for (uint32_t i=m_count-1 ; i<m_count ; --i)
		if (*(m_data[i]) == item)
			return i;
	return XSENS_LIST_NOTFOUND;
}

template <typename T>
template <typename TB>
uint32_t List<T>::findSorted(const TB& item) const
{
	if (m_count < XSENS_LIST_LINEAR_SEARCH_TRESHOLD)			// for small lists, it is faster to simply walk the list
		return find(item);

	uint32_t x = m_count;
	uint32_t n = 1;
	uint32_t i;

	while(x >= n)
	{
		i = (x + n) >> 1;

		if (m_data[i-1] == item)
			return i-1;
		if (m_data[i-1] < item)
			n = i+1;
		else
			x = i-1;
	}
	return XSENS_LIST_NOTFOUND;
}

template <typename T>
template <typename TB>
uint32_t List<T>::findSortedDeref(const TB& item) const
{
	if (m_count < XSENS_LIST_LINEAR_SEARCH_TRESHOLD)			// for small lists, it is faster to simply walk the list
		return findDeref(item);

	uint32_t x = m_count;
	uint32_t n = 1;
	uint32_t i;

	while(x >= n)
	{
		i = (x + n) >> 1;

		if (*(m_data[i-1]) < item)
			n = i+1;
		else if (*(m_data[i-1]) == item)
			return i-1;
		else
			x = i-1;
	}
	return XSENS_LIST_NOTFOUND;
}

template <typename T>
template <typename TB>
uint32_t List<T>::findSortedForInsert(const TB& item) const
{
	uint32_t i;
	if (m_count < XSENS_LIST_LINEAR_SEARCH_TRESHOLD)
	{
		for (i=0;i<m_count;++i)
			if (item <= m_data[i])
				return i;
		return m_count;
	}
	else
	{
		uint32_t x = m_count;
		uint32_t n = 1;

		while(x >= n)
		{
			i = (x + n) >> 1;

			if (m_data[i-1] < item)
				n = i+1;
			else if (m_data[i-1] == item)
				return i-1;
			else
				x = i-1;
		}
		return n-1;
	}
}

template <typename T>
template <typename TB>
uint32_t List<T>::findSortedDerefForInsert(const TB& item) const
{
	uint32_t i;
	if (m_count < XSENS_LIST_LINEAR_SEARCH_TRESHOLD)
	{
		for (i=0;i<m_count;++i)
			if (item <= *m_data[i])
				return i;
		return m_count;
	}
	else
	{
		uint32_t x = m_count;
		uint32_t n = 1;

		while(x >= n)
		{
			i = (x + n) >> 1;

			if (*(m_data[i-1]) < item)
				n = i+1;
			else if (*(m_data[i-1]) == item)
				return i-1;
			else
				x = i-1;
		}
		return n-1;
	}
}

template <typename T>
uint32_t List<T>::insertSorted(const T& item)
{
	uint32_t i = findSortedForInsert(item);
	if (i == m_count)
		append(item);
	else if (item == m_data[i])
		m_data[i] = item;
	else
		insert(item,i);
	return i;
}

template <typename T>
uint32_t List<T>::insertSortedDeref(const T& item)
{
	uint32_t i = findSortedDerefForInsert(*item);
	if (i == m_count)
		append(item);
	else if (*item == *m_data[i])
		m_data[i] = item;
	else
		insert(item,i);
	return i;
}

template <typename T>
template <typename TB>
uint32_t List<T>::insertSortedCopy(const TB& item)
{
	uint32_t i = findSortedDerefForInsert(item);
	if (i == m_count)
		appendCopy<TB>(item);
	else if (item == m_data[i])
		m_data[i] = item;
	else
		insertCopy<TB>(item,i);
	return i;
}

template <typename T>
void List<T>::deleteItemsOnDestroy(void)
{
	if (m_jcf != NULL)
	{
		m_jcf->disable();
		delete m_jcf;
	}
	m_jcf = new JanitorClassFunc<List<T>, void>(*this,&List<T>::deleteAndClear);
}

template <typename T>
void List<T>::freeItemsOnDestroy(void)
{
	if (m_jcf != NULL)
	{
		m_jcf->disable();
		delete m_jcf;
	}
	m_jcf = new JanitorClassFunc<List<T>, void>(*this,&List<T>::freeAndClear);
}

template <typename T>
template <typename TB>
void List<T>::isDeepCopyOf(const List<T>& source)
{
	m_count = 0;
	if (m_max < source.m_count)
		resize(source.m_count);
	m_count = source.m_count;
	for (uint32_t i = 0;i<m_count;++i)
		m_data[i] = new TB(*source.m_data[i]);
}

template <typename T>
void List<T>::isShallowCopyOf(const List<T>& x)
{
	m_count = 0;
	if (m_max < x.m_count)
		resize(x.m_count);
	m_count = x.m_count;
	for (uint32_t i = 0;i<m_count;++i)
		m_data[i] = x.m_data[i];
}

template <typename T>
void List<T>::swap(const uint32_t i, const uint32_t j) XSENS_LIST_THROW
{
	#ifdef _XSENS_LIST_RANGE_CHECKS
		if (i >= m_count || j >= m_count)
			throw Exception("List.swap: index out of bounds");
	#endif
	T tmp = m_data[i];
	m_data[i] = m_data[j];
	m_data[j] = tmp;
}

template <typename T>
void List<T>::reverse(void)
{
	uint32_t half = m_count / 2;
	for (uint32_t i = 0, end=m_count-1; i < half; ++i,--end)
	{
		T tmp = m_data[i];
		m_data[i] = m_data[end];
		m_data[end] = tmp;
	}
}

template <typename T>
template <typename TB>
bool List<T>::operator == (const List<TB>& lst)
{
	if (m_count != lst.m_count)
		return false;
	for (uint32_t i = 0; i < m_count; ++i)
		if (!(m_data[i] == lst.m_data[i]))
			return false;
	return true;
}

} // end of xsens namespace

#endif	// XSENS_LIST_H
