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

#include "CaptureDevice.h"
#include <carmen/visual_tracker_interface.h>
#include <carmen/visual_tracker_messages.h>
#include <carmen/visual_tracker_util.h>
#include <carmen/global_graphics_qt.h>

CaptureDevice::CaptureDevice(QObject* parent)
: QThread(parent)
, m_count(0)
{
}

CaptureDevice::~CaptureDevice()
{
}

IplImage* CaptureDevice::read()
{
	if (isRunning())
	{
		IplImage* frame = m_frameQueue.dequeue();
		return frame;
	}
	return NULL;
}

void CaptureDevice::write(IplImage *frame)
{
	m_frameQueue.enqueue(frame);
}

CameraCaptureDevice::CameraCaptureDevice(int index)
: CaptureDevice()
, m_index(index)
, m_device(NULL)
{
}

CameraCaptureDevice::~CameraCaptureDevice()
{
	close();
}

bool CameraCaptureDevice::open()
{
	close();
	m_device = cvCreateCameraCapture(m_index);
	if (!m_device)
	{
		status("Failed to open video camera source");
		return false;
	}
	cvSetCaptureProperty(m_device, CV_CAP_PROP_FRAME_WIDTH, 320);
	cvSetCaptureProperty(m_device, CV_CAP_PROP_FRAME_HEIGHT, 240);
	connect(&m_frameQueue, SIGNAL(status(const QString)), this, SIGNAL(status(const QString)));
	return true;
}

void CameraCaptureDevice::close()
{
	if (isRunning())
	{
		terminate();
		wait();
	}
	if (m_device)
	{
		cvReleaseCapture(&m_device);
		m_device = NULL;
	}
}

void CameraCaptureDevice::run()
{
	if (!m_device)
	{
		status("Device not opened");
		return;
	}

	while (true)
	{
		IplImage* frame = cvQueryFrame(m_device);
		write(frame);
	}
}

////////////////////////////////////////////////////////////////////////////

DirectoryCaptureDevice::DirectoryCaptureDevice(QDir directory)
: CaptureDevice()
, m_directory(directory)
{
}

DirectoryCaptureDevice::~DirectoryCaptureDevice()
{
	close();
}

bool DirectoryCaptureDevice::open()
{
	close();
	m_inputFiles = m_directory.entryList(QDir::Files|QDir::NoDotAndDotDot, QDir::Name);
	if (m_inputFiles.empty())
	{
		status("Capture directory is empty");
		return false;
	}
	connect(&m_frameQueue, SIGNAL(status(const QString)), this, SIGNAL(status(const QString)));
	return true;
}

void DirectoryCaptureDevice::close()
{
	if (isRunning())
	{
		terminate();
		wait();
	}
}

void DirectoryCaptureDevice::run()
{
	if (m_inputFiles.empty())
	{
		status("Device not opened");
		return;
	}

	while (!m_inputFiles.empty())
	{
		QString filename = m_directory.absoluteFilePath(m_inputFiles.front());
		m_inputFiles.pop_front();
		IplImage* frame = cvLoadImage(filename.toStdString().c_str());
		write(frame);
	}
}

//////////////////////////////////////////////////////////////////////////////////

FileCaptureDevice::FileCaptureDevice(QString filename)
: CaptureDevice()
, m_filename(filename)
, m_capture(NULL)
{
}

FileCaptureDevice::~FileCaptureDevice()
{
	close();
	m_capture = NULL;
}

bool FileCaptureDevice::open()
{
	close();
	m_capture = cvCreateFileCapture(m_filename.toStdString().c_str());
	if (!m_capture)
	{
		status("Failed to open video file source");
		return false;
	}
	connect(&m_frameQueue, SIGNAL(status(const QString)), this, SIGNAL(status(const QString)));
	return true;
}

void FileCaptureDevice::close()
{
	if (isRunning())
	{
		terminate();
		wait();
	}

	if (m_capture)
	{
		cvReleaseCapture(&m_capture);
		m_capture = NULL;
	}
}

void FileCaptureDevice::run()
{
	if (!m_capture)
	{
		status("Video file not opened");
		return;
	}

	while (true)
	{
		IplImage* frame = cvQueryFrame(m_capture);
		if (!frame) break;
		write(frame);
	}
}
