/*
 * Copyright � 2011 Paul Nader
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

#ifndef CAPTUREDEVICE_H_
#define CAPTUREDEVICE_H_

#include <QThread>
#include <QDir>
#include <QStringList>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "BlockingQueue.h"
#include <carmen/carmen.h>

class CaptureDevice : public QThread
{
	Q_OBJECT

public:
	CaptureDevice(QObject* parent = 0);
	virtual ~CaptureDevice();

public:
	virtual bool open() = 0;
	void run() = 0;
	IplImage* read();
	void write(IplImage *frame);
	virtual void close() = 0;

	signals:
	void status(const QString message);

protected:
	int m_count;
	BlockingQueue m_frameQueue;
};

/////////////////////////////////////////////////////////////////////////

class CameraCaptureDevice : public CaptureDevice
{
	Q_OBJECT

public:
	CameraCaptureDevice(int index = 0);
	virtual ~CameraCaptureDevice();

public:
	bool open();
	void close();
	void run();

private:
	int m_index;
	CvCapture* m_device;
};

/////////////////////////////////////////////////////////////////////////

class DirectoryCaptureDevice : public CaptureDevice
{
public:
	DirectoryCaptureDevice(QDir directory);
	virtual ~DirectoryCaptureDevice();

public:
	bool open();
	void close();
	void run();

private:
	QDir m_directory;
	QStringList m_inputFiles;
};

/////////////////////////////////////////////////////////////////////////

class FileCaptureDevice : public CaptureDevice
{
public:
	FileCaptureDevice(QString fileName);
	virtual ~FileCaptureDevice();

public:
	bool open();
	void close();
	void run();

private:
	QString m_filename;
	CvCapture* m_capture;
};

#endif /* CAPTUREDEVICE_H_ */

