/*******************************************************************************
#                 uvcview: Sdl video Usb Video Class grabber           .         #
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

#include "gui.h"
#include "utils.h"
#include "button.h"

#define ADDRESSE(x,y,w) (((y)*(w))+(x))

unsigned char *YUYVbutt = NULL;

typedef struct YUYV {
    unsigned char Y0;
    unsigned char U0;
} YUYV;

/* Each pixels in the resulting figure need to be set. For each one take the nearest available in the original surface
If the last Chroma component is U take a V else take U by moving the index in the nearest pixel from the left
This routine only deal with X axis you need to make the original picture with the good height */

static int resize(unsigned char *INbuff, unsigned char *OUTbuff,
                  int Owidth, int Oheight, int width, int height)
{
    int rx;
    int xscale;
    int x, y;
    int Cc, lastCc;
    YUYV *input = (YUYV *) INbuff;
    YUYV *output = (YUYV *) OUTbuff;
    if (!width || !height)
        return -1;
    /* at start Cc mean a U component so LastCc should be a V */
    lastCc = 1;
    xscale = (Owidth << 16) / width;
    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            rx = x * xscale >> 16;
            if (((2 * rx + 1) & 0x03) == 3)
                Cc = 1;                // find V component
            else
                Cc = 0;
            if (lastCc == Cc) {
                /* no Chroma interleave correct by moving the index */
                rx -= 1;
                Cc = !Cc;
            }
            memcpy(output++, &input[ADDRESSE((int) rx, (int) y, Owidth)],
                   sizeof(YUYV));
            lastCc = Cc;
        }
    }
    return 0;
}

int creatButt(int width, int height)
{
    int wOrg = 0;
    int hOrg = 0;
    jpeg_decode(&YUYVbuttOrg, bouttons, &wOrg, &hOrg);
    if (wOrg != BUTTWIDTH || hOrg != BUTTHEIGHT) {
        printf(" alloc jpeg Button fail !!\n");
        goto err;
    }
    YUYVbutt = (unsigned char *) calloc(1, width * height << 1);
    if (!YUYVbutt) {
        printf(" alloc Button fail !!\n");
        goto err;
    }
    if (resize(YUYVbuttOrg, YUYVbutt, BUTTWIDTH, BUTTHEIGHT, width, height)
        < 0) {
        printf(" resize Button fail !!\n");
        goto err;
    }
    return 0;
  err:
    exit(0);
}
int destroyButt(void)
{
    free(YUYVbutt);
    YUYVbutt = NULL;
    free(YUYVbuttOrg);
    YUYVbuttOrg = NULL;
}
