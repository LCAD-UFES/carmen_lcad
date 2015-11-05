/*
 * DetectorObserver.cpp
 *
 *  Created on: 29/11/2011
 *      Author: avelino
 */

#include <carmen/carmen.h>
#include <carmen/visual_tracker_interface.h>
#include <carmen/visual_tracker_messages.h>
#include <carmen/visual_tracker_util.h>
#include "DetectorObserver.h"
#include <QRectF>

DetectorObserver::~DetectorObserver()
{
}

DetectorObserver::DetectorObserver()
{
	m_train.image = NULL;
}

void DetectorObserver::signalBoundingBox(bool val)
{
        val = val;
}

void DetectorObserver::signalPoints(bool val)
{
        val = val;
}

void DetectorObserver::signalConfidence(bool val)
{
        val = val;
}

void DetectorObserver::signalDetections(bool val)
{
        val = val;
}

void DetectorObserver::signalPositive(bool val)
{
        val = val;
}

void DetectorObserver::signalNegative(bool val)
{
        val = val;
}

void DetectorObserver::flipImage(bool val)
{
        val = val;
}

void DetectorObserver::train(IplImage *frame, QRectF selection)
{
    if (frame)
    {
        m_train.host = carmen_get_host();
        m_train.timestamp = carmen_get_time();

        m_train.channels = frame->nChannels;
        m_train.height = frame->height;
        m_train.width = frame->width;

        m_train.size = frame->height * frame->width * frame->nChannels;

        m_train.image = (unsigned char *) malloc(m_train.size * sizeof(unsigned char *));

        copy_BGR_image_to_RGB_image(frame, m_train.image, frame->nChannels);

        m_train.rect.x = (int)selection.x();
        m_train.rect.y = (int)selection.y();
        m_train.rect.width = (int)selection.width();
        m_train.rect.height = (int)selection.height();

        IPC_RETURN_TYPE err;
        err = IPC_publishData(CARMEN_VISUAL_TRACKER_TRAIN_MESSAGE_NAME, &m_train);
        carmen_test_ipc_exit(err, "Could not publish", CARMEN_VISUAL_TRACKER_TRAIN_MESSAGE_NAME);

        free(m_train.image);
    }
}

void DetectorObserver::test(IplImage *frame)
{
    if (frame)
    {
        m_test.host = carmen_get_host();
        m_test.timestamp = carmen_get_time();

        m_test.channels = frame->nChannels;
        m_test.height = frame->height;
        m_test.width = frame->width;

        m_test.size = frame->height * frame->width * frame->nChannels;

        m_test.image = (unsigned char *) malloc(m_test.size * sizeof(unsigned char *));

        copy_BGR_image_to_RGB_image(frame, m_test.image, frame->nChannels);

        IPC_RETURN_TYPE err;
        err = IPC_publishData(CARMEN_VISUAL_TRACKER_TEST_MESSAGE_NAME, &m_test);
        carmen_test_ipc_exit(err, "Could not publish", CARMEN_VISUAL_TRACKER_TEST_MESSAGE_NAME);

        free(m_test.image);

        emit newFrame(frame);

    }

}

void DetectorObserver::display(IplImage *frame)
{
    if (frame)
        emit newFrame(frame);
}

void DetectorObserver::display(QRectF region)
{
    emit boundingRect(region);
}

bool DetectorObserver::trained()
{
    return m_train.image != NULL;
}
