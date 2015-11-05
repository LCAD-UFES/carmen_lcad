#ifndef XSENS_MONOLITHIC
/*! \file
	\brief	Contains the FIFO queue interface and implementation (inline)

	\section FileCopyright Copyright Notice 
	Copyright (C) Xsens Technologies B.V., 2006.  All rights reserved.

	This source code is intended for use only by Xsens Technologies BV and
	those that have explicit written permission to use it from
	Xsens Technologies BV.

	THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
	KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
	IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
	PARTICULAR PURPOSE.

	\section FileChangelog	Changelog

	\par 2006-05-03, v0.0.1
	\li Job Mulder:	Created Cmtfifoqueue.h

	\par 2006-07-21, v0.1.0
	\li Job Mulder:	Updated file for release 0.1.0
*/
#endif
#ifndef XSENS_FIFOQUEUE_H
#define XSENS_FIFOQUEUE_H


namespace xsens {

//////////////////////////////////////////////////////////////////////////////////////////

/*! \brief A FIFO queue with limited length (cyclic).

	The class is based on the STL queue class, but has a limited size. If more items are
	inserted than would fit, the oldest item is overwritten. The class can only handle 
	pointer types.
*/
template <class T, bool E=true>
class FifoQueue {
protected:
	size_t m_maxCount;
	size_t m_currentCount;
	size_t m_first;
	bool m_deleteOnOverwrite;

	T*	m_list;
public:
	typedef T		value_type;		//!< The type of the value stored in this queue.
	typedef size_t	size_type;		//!< The type of a 'size' value.

	//! Create an empty queue with capacity size.
	FifoQueue(size_type size=16, bool delOnOverwrite = true)
	{
		if (size > 0)
			m_maxCount = size;
		else
			m_maxCount = 1;
		m_list = new T[m_maxCount];
		m_currentCount = 0;
		m_first = 0;
		m_deleteOnOverwrite = delOnOverwrite;
	}
	
	//! The copy constructor.
	template <bool E2>
	FifoQueue(const FifoQueue<T,E2>& q)
	{
		m_maxCount = q.m_maxCount;
		m_list = new T[m_maxCount];
		m_currentCount = q.m_currentCount;
		m_deleteOnOverwrite = q.m_deleteOnOverwrite;
		m_first = 0;
		for (size_t i = 0;i<m_currentCount;++i)
			m_list[i] = q.m_list[(i+q.m_first) % m_maxCount];
	}
	
	void eraseAndClear(void)
	{
		for (size_t i = 0;i<m_currentCount;++i)
			delete m_list[(i+m_first) % m_maxCount];
		m_currentCount = 0;
		m_first = 0;
	}

	//! The destructor.
	~FifoQueue()
	{
		if (E)
			eraseAndClear();
		m_maxCount = 0;
		delete[] m_list;
	}

	//! The assignment operator.
	template <bool E2>
	FifoQueue<T,E>& operator=(const FifoQueue<T,E2>& q)
	{
		if (m_maxCount != q.m_maxCount)
		{
			delete[] m_list;
			m_maxCount = q.m_maxCount;
			m_list = new T[m_maxCount];
		}
		m_currentCount = q.m_currentCount;
		m_first = 0;
		for (size_t i = 0;i<m_currentCount;++i)
			m_list[i] = q.m_list[(i+q.m_first) % m_maxCount];

		return *this;
	}
	
	//! Resize the queue, note that this function clears the queue.
	void resize(const size_t size)
	{
		if (E)
			eraseAndClear();
		delete[] m_list;
		if (size > 0)
			m_maxCount = size;
		else
			m_maxCount = 1;
		m_list = new T[m_maxCount];
		m_currentCount = 0;
		m_first = 0;
	}

	//! Return true if the queue is empty.
	bool empty() const
	{
		return (m_currentCount == 0);
	}
	
	//! Return the maximum number of elements in the queue.
	size_type size() const
	{
		return m_maxCount;
	}
	
	//! Return the number of elements currnetly in the queue.
	size_type length() const
	{
		return m_currentCount;
	}

	//! Return the oldest element in the queue.
	value_type& front()
	{
		return m_list[m_first];
	}
	
	//! Return the oldest element in the queue.
	const value_type& front() const
	{
		return m_list[m_first];
	}
	
	//! Return the newest element in the queue.
	value_type& back()
	{
		return m_list[(m_first + m_currentCount - 1) % m_maxCount];
	}
	
	//! Return the newest element in the queue.
	const value_type& back() const
	{
		return m_list[(m_first + m_currentCount - 1) % m_maxCount];
	}
	
	//! Insert x at the back of the queue.
	void push(const value_type& x)
	{
		if (m_currentCount == m_maxCount)
		{
			if (m_deleteOnOverwrite)
				delete m_list[m_first];
			
			m_list[m_first] = x;
			m_first = (m_first+1) % m_maxCount;
		}
		else
		{
			m_list[(m_first + m_currentCount++) % m_maxCount] = x;
		}
	}

	//! Remove the element at the front of the queue.
	void pop(void)
	{
		m_first = (m_first+1) % m_maxCount;
		--m_currentCount;
	}

	//! Remove the element at the back of the queue.
	void popBack(void)
	{
		--m_currentCount;
	}

	//! Return the index'th oldest item from the queue
	const value_type& operator[] (size_t index) const
	{
		if (index >= m_currentCount)
			return m_list[(m_first + m_currentCount - 1) % m_maxCount];
		else
			return m_list[(m_first + index) % m_maxCount];
	}

	//! Return the index'th oldest item from the queue
	value_type& operator[] (size_t index)
	{
		if (index >= m_currentCount)
			return m_list[(m_first + m_currentCount - 1) % m_maxCount];
		else
			return m_list[(m_first + index) % m_maxCount];
	}

	void clear(void)
	{
		m_currentCount = 0;
		m_first = 0;
	}

	void remove(size_t index)
	{
		if (index >= m_currentCount)
			return;
		if (index == 0)
			pop();
		else
		{
			--m_currentCount;
			for (size_t i=index;i<m_currentCount;++i)
				m_list[(m_first + i) % m_maxCount] = m_list[(1 + m_first + i) % m_maxCount];
		}
	}
};


//////////////////////////////////////////////////////////////////////////////////////////

/*! \brief A FIFO queue with limited length (cyclic).

	The class is based on the STL queue class, but has a limited size. If more items are
	inserted than would fit, the oldest item is overwritten. The class can only handle 
	non-pointer types.
*/
template <class T>
class FifoQueueBasic {
protected:
	size_t m_maxCount;
	size_t m_currentCount;
	size_t m_first;

	T*	m_list;
public:
	typedef T		value_type;		//!< The type of the value stored in this queue.
	typedef size_t	size_type;		//!< The type of a 'size' value.

	//! Create an empty queue with capacity 'size'.
	FifoQueueBasic(size_type size=16)
	{
		if (size > 0)
			m_maxCount = size;
		else
			m_maxCount = 1;
		m_list = new T[m_maxCount];
		m_currentCount = 0;
		m_first = 0;
	}
	
	//! The copy constructor.
	FifoQueueBasic(const FifoQueueBasic<T>& q)
	{
		m_maxCount = q.m_maxCount;
		m_list = new T[m_maxCount];
		m_currentCount = q.m_currentCount;
		m_first = 0;
		for (size_t i = 0;i<m_currentCount;++i)
			m_list[i] = q.m_list[(i+q.m_first) % m_maxCount];
	}
	
	void eraseAndClear(void)
	{
		for (size_t i = 0;i<m_currentCount;++i)
			delete m_list[(i+m_first) % m_maxCount];
		m_currentCount = 0;
		m_first = 0;
	}

	//! The destructor.
	~FifoQueueBasic()
	{
		m_maxCount = 0;
		delete[] m_list;
	}

	//! The assignment operator.
	FifoQueueBasic<T>& operator=(const FifoQueueBasic<T>& q)
	{
		if (m_maxCount != q.m_maxCount)
		{
			delete[] m_list;
			m_maxCount = q.m_maxCount;
			m_list = new T[m_maxCount];
		}
		m_currentCount = q.m_currentCount;
		m_first = 0;
		for (size_t i = 0;i<m_currentCount;++i)
			m_list[i] = q.m_list[(i+q.m_first) % m_maxCount];

		return *this;
	}
	
	//! Resize the queue, note that this function clears the queue.
	void resize(const size_t size)
	{
		delete[] m_list;
		if (size > 0)
			m_maxCount = size;
		else
			m_maxCount = 1;
		m_list = new T[m_maxCount];
		m_currentCount = 0;
		m_first = 0;
	}

	//! Return true if the queue is empty.
	bool empty() const
	{
		return (m_currentCount == 0);
	}
	
	//! Return the maximum number of elements in the queue.
	size_type size() const
	{
		return m_maxCount;
	}
	
	//! Return the number of elements currently in the queue.
	size_type length() const
	{
		return m_currentCount;
	}

	//! Return the oldest element in the queue.
	value_type& front()
	{
		return m_list[m_first];
	}
	
	//! Return the oldest element in the queue.
	const value_type& front() const
	{
		return m_list[m_first];
	}
	
	//! Return the newest element in the queue.
	value_type& back()
	{
		return m_list[(m_first + m_currentCount - 1) % m_maxCount];
	}
	
	//! Return the newest element in the queue.
	const value_type& back() const
	{
		return m_list[(m_first + m_currentCount - 1) % m_maxCount];
	}
	
	//! Insert x at the back of the queue.
	void push(const value_type& x)
	{
		if (m_currentCount == m_maxCount)
		{
			m_list[m_first] = x;
			m_first = (m_first+1) % m_maxCount;
		}
		else
		{
			m_list[(m_first + m_currentCount++) % m_maxCount] = x;
		}
	}

	//! Insert x at the front of the queue (LIFO operation).
	void push_front(const value_type& x)
	{
		m_first = (m_first+m_maxCount-1)%m_maxCount;
		if (m_currentCount == 0)
			m_first = 0;
		m_list[m_first] = x;
		if (m_currentCount < m_maxCount)
			++m_currentCount;
	}

	//! Remove the element at the front of the queue.
	void pop(void)
	{
		if (m_currentCount > 0)
		{
			m_first = (m_first+1) % m_maxCount;
			--m_currentCount;
		}
	}

	//! Remove the element at the back of the queue.
	void popBack(void)
	{
		if (m_currentCount > 0)
			--m_currentCount;
	}

	//! Return the index'th oldest item from the queue
	const value_type& operator[] (size_t index) const
	{
		if (index >= m_currentCount)
			return m_list[(m_first + m_currentCount - 1) % m_maxCount];
		else
			return m_list[(m_first + index) % m_maxCount];
	}

	//! Return the index'th oldest item from the queue
	value_type& operator[] (size_t index)
	{
		if (index >= m_currentCount)
			return m_list[(m_first + m_currentCount - 1) % m_maxCount];
		else
			return m_list[(m_first + index) % m_maxCount];
	}

	void clear(void)
	{
		m_currentCount = 0;
		m_first = 0;
	}

	void remove(size_t index)
	{
		if (index >= m_currentCount)
			return;
		if (index == 0)
			pop();
		else
		{
			--m_currentCount;
			for (size_t i=index;i<m_currentCount;++i)
				m_list[(m_first + i) % m_maxCount] = m_list[(1 + m_first + i) % m_maxCount];
		}
	}
};

} // end of xsens namespace

#endif	// XSENS_FIFOQUEUE_H
