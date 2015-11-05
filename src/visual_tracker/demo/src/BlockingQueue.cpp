/*
 * Copyright © 2011 Paul Nader 
 *
 * This file is part of QOpenTLD.
 *
 * This program is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 * 
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 * 
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "BlockingQueue.h"

BlockingQueue::BlockingQueue(unsigned bufferSize)
: count(0)
, m_bufferSize(bufferSize)
{
}

BlockingQueue::~BlockingQueue()
{
	waitRoomAvailable.wakeAll();
	waitDataAvailable.wakeAll();
}

void BlockingQueue::enqueue(IplImage* &t)
{
	m_queueAccess.lock();
	if ((unsigned) queue.size() >= m_bufferSize)
	{
		m_queueAccess.unlock();
		emit status(QString("FD"));
		enqueueWaitConditionLocker.lock();
		waitRoomAvailable.wait(&enqueueWaitConditionLocker);
		enqueueWaitConditionLocker.unlock();

		m_queueAccess.lock();
	}
	queue.enqueue(t);
	emit status(QString("BS: %1").arg(queue.size()));
	m_queueAccess.unlock();
	cMutex.lock();
	count++;
	cMutex.unlock();
	waitDataAvailable.wakeAll();
}

IplImage* BlockingQueue::dequeue()
{
	//emit status(QString("BS: %1").arg(queue.size()));
	m_queueAccess.lock();
	bool isQueueEmpty = queue.isEmpty();
	m_queueAccess.unlock();

	if (isQueueEmpty)
	{
		cMutex.lock();
		if (count == 0)
		{
			cMutex.unlock();
			//emit status(QString("In waitBlockingQueueAccess.wait"));
			enqueueWaitConditionLocker.lock();
			waitDataAvailable.wait(&enqueueWaitConditionLocker);
			enqueueWaitConditionLocker.unlock();

			cMutex.lock();
		}
		cMutex.unlock();
	}

	m_queueAccess.lock();
	if (queue.empty()) return NULL;
	IplImage* returnVal = queue.dequeue();
	cMutex.lock();
	count--;
	cMutex.unlock();
	m_queueAccess.unlock();
	waitRoomAvailable.wakeAll();
	return returnVal;
}

size_t BlockingQueue::size()
{ 
	QMutexLocker locker(&m_queueAccess);
	return queue.size();
}
