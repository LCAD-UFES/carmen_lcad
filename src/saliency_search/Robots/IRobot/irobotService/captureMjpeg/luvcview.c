/*******************************************************************************
#                 luvcview: Sdl video Usb Video Class grabber           .        #
#This package work with the Logitech UVC based webcams with the mjpeg feature. #
#All the decoding is in user space with the embedded jpeg decoder              #
#.                                                                             #
#                 Copyright (C) 2005 2006 Laurent Pinchart &&  Michel Xhaard     #
#                                                                              #
# This program is free software; you can redistribute it and/or modify         #
# it under the terms of the GNU General Public License as published by         #
# the Free Software Foundation; either version 2 of the License, or            #
# (at your option) any later version.                                          #
#                                                                              #
# This program is distributed in the hope that it will be useful,              #
# but WITHOUT ANY WARRANTY; without even the implied warranty of               #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                #
# GNU General Public License for more details.                                 #
#                                                                              #
# You should have received a copy of the GNU General Public License            #
# along with this program; if not, write to the Free Software                  #
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA    #
#                                                                              #
*******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/file.h>
#include <string.h>
#include <pthread.h>
#include <linux/videodev.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <errno.h>
#include <fcntl.h>
#include <time.h>
#include <sys/time.h>
#include <signal.h>
#include "v4l2uvc.h"
#include "gui.h"
#include "utils.h"
#include "color.h"
/* Fixed point arithmetic */
#define FIXED Sint32
#define FIXED_BITS 16
#define TO_FIXED(X) (((Sint32)(X))<<(FIXED_BITS))
#define FROM_FIXED(X) (((Sint32)(X))>>(FIXED_BITS))

#define INCPANTILT 64 // 1°

static const char version[] = VERSION;
struct vdIn *videoIn;

int main(int argc, char *argv[])
{

        int status;
        int currtime;
        int lasttime;
        unsigned char *p = NULL;
        int hwaccel = 0;
        const char *videodevice = NULL;
        const char *mode = NULL;
        //int format = V4L2_PIX_FMT_MJPEG;
        int format = V4L2_PIX_FMT_YUYV;
        int i;
        int grabmethod = 1;
        int width = 640;
        int height = 480;
        int fps = 30;
        unsigned char frmrate = 0;
        char *avifilename = NULL;
        int queryformats = 0;
        int querycontrols = 0;
        int readconfigfile = 0;
        char *separateur;
        char *sizestring = NULL;
        char *fpsstring  = NULL;
        int enableRawStreamCapture = 0;
        int enableRawFrameCapture = 0;

        printf("luvcview %s\n\n", version);

  videodevice = "/dev/video0";

        videoIn = (struct vdIn *) calloc(1, sizeof(struct vdIn));
        if ( queryformats ) {
                /* if we're supposed to list the video formats, do that now and go out */
                check_videoIn(videoIn,(char *) videodevice);
                free(videoIn);
                exit(1);
        }

        if (init_videoIn
                        (videoIn, (char *) videodevice, width, height, fps, format,
                         grabmethod, avifilename) < 0)
                exit(1);

        /* if we're supposed to list the controls, do that now */
        if ( querycontrols )
                enum_controls(videoIn->fd);

  /* if we're supposed to read the control settings from a configfile, do that now */
  if ( readconfigfile )
    load_controls(videoIn->fd);


  if (enableRawFrameCapture)
    videoIn->rawFrameCapture = enableRawFrameCapture;

        //initLut();

        /* main big loop */
        while (videoIn->signalquit) {
                //currtime = SDL_GetTicks();
                //if (currtime - lasttime > 0) {
                //        frmrate = 1000/(currtime - lasttime);
                //}
                lasttime = currtime;
                if (uvcGrab(videoIn) < 0) {
                        printf("Error grabbing\n");
                        break;
                }

                /* if we're grabbing video, show the frame rate */
    printf("frame rate: %d     \n",frmrate);

                ////videoIn->framebuffer, videoIn->width * (videoIn->height) * 2);
    //
    //videoIn->getPict=1;
                if (videoIn->getPict) {
                        switch(videoIn->formatIn){
                                case V4L2_PIX_FMT_MJPEG:
                                        get_picture(videoIn->tmpbuffer,videoIn->buf.bytesused);
                                        break;
                                case V4L2_PIX_FMT_YUYV:
                                        get_pictureYV2(videoIn->framebuffer,videoIn->width,videoIn->height);
                                        break;
                                default:
                                        break;
                        }
                        videoIn->getPict = 0;
                        printf("get picture !\n");
                }

        }

        close_v4l2(videoIn);
        free(videoIn);
        //freeLut();
        printf("Cleanup done. Exiting ...\n");
}

