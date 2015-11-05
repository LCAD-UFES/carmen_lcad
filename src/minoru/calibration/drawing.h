/*
    functions useful for drawing within images
    Copyright (C) 2009 Bob Mottram
    fuzzgun@gmail.com

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

#ifndef drawing_h
#define drawing_h

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string>
#include <assert.h>

#ifndef ABS
    #define ABS(a) (((a) < 0) ? -(a) : (a))
#endif

#ifndef PI
    #define PI 3.14159265358979323846264338327950288419716939937510
#endif

class drawing
{
    public:
        static void drawBox(unsigned char* img, int img_width, int img_height, int x, int y, int radius, int r, int g, int b, int line_width);
        static void drawBox(unsigned char* img, int img_width, int img_height, int x, int y, int box_width, int box_height, float rotation, int r, int g, int b, int line_width);
        static void drawCross(unsigned char* img, int img_width, int img_height, int x, int y, int radius, int r, int g, int b, int line_width);
        static void drawCircle(unsigned char* img, int img_width, int img_height, int x, int y, int radius, int r, int g, int b, int line_width);
        static void drawCircle(unsigned char* img, int img_width, int img_height, float x, float y, float radius, int r, int g, int b, int line_width);
        static void drawSpot(unsigned char* img, int img_width, int img_height, int x, int y, int radius, int r, int g, int b);
        static void drawBlendedSpot(unsigned char* img, int img_width, int img_height, int x, int y, int radius, int r, int g, int b);
        static void drawGrid(unsigned char* img, int img_width, int img_height, int centre_x, int centre_y, float rotation, float size_width, float size_height, int columns, int rows, int r, int g, int b, int linewidth);
        static void drawLine(unsigned char* img, int img_width, int img_height, int x1, int y1, int x2, int y2, int r, int g, int b, int linewidth, bool overwrite);
};

#endif
