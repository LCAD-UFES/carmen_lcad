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

#ifndef __RECTIFY_H__
#define __RECTIFY_H__

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <vector>
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;

class Rectify {
private:
    CvMat *homography;
    IplImage *l;
    IplImage *temp_img;
    bool rectify;

    void WarpShift(
        IplImage* src,
        IplImage* dest,
        int shiftx,
        int shifty);

public:
    int Parse(char * rectification_str);

    void Set(double * m);

    void update(
        int image_width,
        int image_height,
        unsigned char * image_data,
        int v_shift);

    void update(
        int image_width,
        int image_height,
        int16_t *image_data,
        int v_shift);

    Rectify();
    ~Rectify();
};

#endif

