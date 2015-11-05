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

#include <carmen/carmen.h>
#include <carmen/global_graphics_qt.h>
#include <carmen/visual_tracker_interface.h>
#include <carmen/visual_tracker_messages.h>
#include <carmen/visual_tracker_util.h>
#include "MainWindow.h"
#include <QApplication>
#include <QRectF>

QApplication *a = NULL;
MainWindow *w = NULL;

IplImage *
QImage2IplImage(const QImage *qImage)
{
    QImage::Format imageType = qImage->format();

    if(imageType == QImage::Format_RGB888)
    {
        IplImage *image3channels = cvCreateImageHeader( cvSize(qImage->width(), qImage->height()), IPL_DEPTH_8U, 3);
        image3channels->imageData = (char*) qImage->bits();
        uchar* newdata = (uchar*) malloc(sizeof(uchar) * qImage->byteCount());
        memcpy(newdata, qImage->bits(), qImage->byteCount());
        image3channels->imageData = (char*) newdata;
        cvCvtColor(image3channels, image3channels, CV_RGB2BGR);
        return image3channels;
    }
    else if(imageType == QImage::Format_RGB32)
    {
        IplImage *image4channels = cvCreateImageHeader( cvSize(qImage->width(), qImage->height()), IPL_DEPTH_8U, 4);
        image4channels->imageData = (char*) qImage->bits();

        IplImage *image3channels = cvCreateImage( cvGetSize(image4channels), IPL_DEPTH_8U, 3 );
        cvConvertImage(image4channels, image3channels, 0);

        cvReleaseImageHeader(&image4channels);

        return image3channels;
    }
    else if (imageType == QImage::Format_Indexed8)
    {
        IplImage *image1channel = cvCreateImageHeader( cvSize(qImage->width(), qImage->height()), IPL_DEPTH_8U, 1);
        image1channel->imageData = (char*) qImage->bits();
        uchar* newdata = (uchar*) malloc(sizeof(uchar) * qImage->byteCount());
        memcpy(newdata, qImage->bits(), qImage->byteCount());
        image1channel->imageData = (char*) newdata;
        return image1channel;
    }
    else
        return NULL;
}

void visual_tracker_output_message_handler(carmen_visual_tracker_output_message *message)
{
    if(w != NULL)
    {
        QRectF bb(message->rect.x, message->rect.y, message->rect.width, message->rect.height);
        w->display(bb);
    }
}

void bumblebee_message_handler(carmen_bumblebee_basic_stereoimage_message *message)
{
    if(w != NULL)
    {
        CvSize size = cvSize(message->width, message->height);
        IplImage *frame_left = cvCreateImage(size, IPL_DEPTH_8U, 3);
        copy_RGB_image_to_BGR_image(message->raw_left, frame_left, 3);
        w->display(frame_left);
    }
}

void kinect_message_handler(carmen_kinect_video_message *message)
{
    if(w != NULL)
    {
//        QImage qtImage = QImage(message->video, message->width, message->height, QImage::Format_RGB888);
//        IplImage *frame_left = QImage2IplImage(&qtImage);
        CvSize size = cvSize(message->width, message->height);
        IplImage *frame_left = cvCreateImage(size, IPL_DEPTH_8U, 3);
        copy_RGB_image_to_BGR_image(message->video, frame_left, 3);
        w->display(frame_left);
    }
}

void shutdown_module(int signo)
{
  if(signo == SIGINT)
  {
     carmen_ipc_disconnect();
     printf("Visual Tracker View: disconnected.\n");
     exit(0);
  }
}

int main(int argc, char **argv)
{
        a = new QApplication(argc, argv);
        w = new MainWindow();

        /* Do Carmen Initialization*/
        carmen_ipc_initialize(argc, argv);

        /* Check the param server version */
        carmen_param_check_version(argv[0]);

        /* Register shutdown cleaner handler */
        signal(SIGINT, shutdown_module);

        /* Register Carmen Callbacks to Qt interface */
        carmen_graphics_update_ipc_callbacks_qt(w->getVideoWindow(), SLOT(updateIPC(int)));

        /* Define published messages by your module */
        carmen_visual_tracker_define_message_train();

        carmen_visual_tracker_define_message_test();

        /* Subscribe to sensor messages */
        carmen_visual_tracker_subscribe_output(NULL, (carmen_handler_t) visual_tracker_output_message_handler, CARMEN_SUBSCRIBE_LATEST);

        carmen_bumblebee_basic_subscribe_stereoimage1(NULL, (carmen_handler_t) bumblebee_message_handler, CARMEN_SUBSCRIBE_LATEST);

        carmen_kinect_subscribe_video_message(0, NULL, (carmen_handler_t) kinect_message_handler, CARMEN_SUBSCRIBE_LATEST);

        a->setWindowIcon(QIcon("resources/QOpenTLD.ico"));
        w->show();

        return a->exec();
}
