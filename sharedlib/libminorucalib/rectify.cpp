/*
 Rectify images
 Copyright 2010. All rights reserved.
 Bob Mottram <fuzzgun@gmail.com>

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "rectify.h"

Rectify::Rectify()
{
    homography = cvCreateMat(3,3,CV_64F);
    temp_img=NULL;
    rectify=false;
}

Rectify::~Rectify()
{
    cvReleaseMat (&homography);

    if (temp_img != NULL) {
        cvReleaseImage(&l);
        cvReleaseImage(&temp_img);
    }
}

void Rectify::WarpShift(
    IplImage* src,
    IplImage* dest,
    int shiftx,
    int shifty)
{
    IplImage* _dest;
    if (src==dest) {
        _dest=cvCreateImage(cvGetSize(src),src->depth,src->nChannels);
    }
    else {
        _dest = dest;
    }
    cvZero(_dest);

    if (shiftx>=0 &&shifty>=0) {
        cvSetImageROI( src, cvRect( 0, 0, src->width-shiftx, src->height-shifty ) );
        cvSetImageROI( _dest, cvRect( shiftx, shifty, src->width, src->height ) );
    }
    else if(shiftx>=0 &&shifty<0) {
        cvSetImageROI( src, cvRect( 0, -shifty, src->width-shiftx, src->height ) );
        cvSetImageROI( _dest, cvRect( shiftx, 0, src->width, src->height+shifty ) );
    }
    else if(shiftx<0 &&shifty<0) {
        cvSetImageROI( src, cvRect( -shiftx, -shifty, src->width, src->height ) );
        cvSetImageROI( _dest, cvRect( 0, 0, src->width+shiftx, src->height+shifty ) );
    }
    else if(shiftx<0 &&shifty>=0) {
        cvSetImageROI( src, cvRect(-shiftx, 0, src->width, src->height-shifty ) );
        cvSetImageROI( _dest, cvRect( 0, shifty, src->width+shiftx, src->height ) );
    }

    cvCopy(src, _dest);

    cvResetImageROI( src );
    cvResetImageROI( _dest );

    if (src==dest) {
        cvCopy(_dest,dest);
        cvReleaseImage(&_dest);
    }
}

void Rectify::update(
    int image_width,
    int image_height,
    unsigned char * image_data,
    int v_shift)
{
    unsigned char * img;

    if (rectify) {
        if (temp_img == NULL) {
            l = cvCreateImage(cvSize(image_width, image_height), 8, 3);
            temp_img = cvCreateImage(cvSize(image_width, image_height), 8, 3);
        }

        img = (unsigned char *)l->imageData;
        memcpy((void*)img, (void*)image_data, image_width*image_height*3);
        cvWarpPerspective(l, temp_img, homography);
        WarpShift(temp_img, l, 0, v_shift);
        memcpy((void*)image_data, (void*)img, image_width*image_height*3);
    }
}

void Rectify::update(
    int image_width,
    int image_height,
    int16_t *image_data,
    int v_shift)
{
    int i;
    unsigned char * img;

    if (rectify) {

        if (temp_img == NULL) {
            l = cvCreateImage(cvSize(image_width, image_height), 8, 3);
            temp_img = cvCreateImage(cvSize(image_width, image_height), 8, 3);
        }

        img = (unsigned char *)l->imageData;

        for (i = 0; i < image_width*image_height; i++) {
            img[i*3] = image_data[i]/3;
            img[i*3+1] = img[i*3];
            img[i*3+2] = img[i*3];
        }

        cvWarpPerspective(l, temp_img, homography);
        WarpShift(temp_img, l, 0, v_shift);

        for (i = 0; i < image_width*image_height; i++) {
            image_data[i] = img[i*3]+img[i*3+1]+img[i*3+2];
        }
    }
}

int Rectify::Parse(
    char * rectification_str)
{
    char str[256];
    double params[3*3];
    int i=0,index=0,p=0,success=0;
    while (rectification_str[i]!=0) {
        if ((index > 0) &&
            (rectification_str[i]==' ')) {
            str[index]=0;
            params[p++] = atof(str);
            index=0;   
        }
        else {
            str[index++] = rectification_str[i];
        }
        if (i==255) break;
        i++;
    }
    if (index > 0) {
        str[index]=0;
        params[p++] = atof(str);
    }
    if (p==9) {

        i=0;
        for (int y = 0; y < 3; y++) {
            for (int x = 0; x < 3; x++, i++) {
                cvmSet(homography, y, x, params[i]);
            }
        }
        rectify=true;
        success=1;
    }
    return success;
}

void Rectify::Set(double * m)
{
    int i=0;
    for (int y = 0; y < 3; y++) {
        for (int x = 0; x < 3; x++, i++) {
            cvmSet(homography, y, x, m[i]);
        }
    }
    rectify=true;
}

