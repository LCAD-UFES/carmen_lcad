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

#ifndef BLOCKINGQUEUE_H_
#define BLOCKINGQUEUE_H_

#include <QObject>
#include <QMutex>
#include <QQueue>
#include <QWaitCondition>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#define FRAME_BUFFER_SIZE 30

class BlockingQueue : public QObject
{
	Q_OBJECT

public:
	BlockingQueue(unsigned bufferSize = FRAME_BUFFER_SIZE);
	~BlockingQueue();
	void setBufferSize(unsigned bufferSize) { m_bufferSize = bufferSize; };
	void enqueue(IplImage* &t);
	size_t size();
	IplImage* dequeue();

signals:
	void status(const QString message);

private:
	int count;
	unsigned m_bufferSize;
	QQueue<IplImage*> queue;
	QMutex m_queueAccess, dequeueWaitConditionLocker, enqueueWaitConditionLocker, cMutex;
	QWaitCondition waitDataAvailable, waitRoomAvailable;
};

#endif /* BLOCKINGQUEUE_H_ */
