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

#include "drawing.h"

// ********** public methods **********

/*!
 * \brief drawBox
 * \param img
 * \param img_width
 * \param img_height
 * \param x
 * \param y
 * \param radius
 * \param r
 * \param g
 * \param b
 * \param line_width
 */
void drawing::drawBox(
    unsigned char* img,
    int img_width,
    int img_height,
    int x,
    int y,
    int radius,
    int r,
    int g,
    int b,
    int line_width)
{
    int radius_y = radius * img_width / img_height;
    int tx = x - radius;
    int ty = y - radius_y;
    int bx = x + radius;
    int by = y + radius_y;
    drawLine(img, img_width, img_height, tx, ty, bx, ty, r, g, b, line_width, false);
    drawLine(img, img_width, img_height, bx, ty, bx, by, r, g, b, line_width, false);
    drawLine(img, img_width, img_height, tx, by, bx, by, r, g, b, line_width, false);
    drawLine(img, img_width, img_height, tx, by, tx, ty, r, g, b, line_width, false);
}

/*!
 * \brief draw a rotated box
 * \param img
 * \param img_width
 * \param img_height
 * \param x
 * \param y
 * \param box_width
 * \param box_height
 * \param rotation
 * \param r
 * \param g
 * \param b
 * \param line_width
 */
void drawing::drawBox(
    unsigned char* img,
    int img_width,
    int img_height,
    int x,
    int y,
    int box_width,
    int box_height,
    float rotation,
    int r,
    int g,
    int b,
    int line_width)
{
    int tx = -box_width/2;
    int ty = -box_height/2;
    int bx = box_width/2;
    int by = box_height/2;
    int* xx = new int[4];
    int* yy = new int[4];
    assert(xx != NULL);
    assert(yy != NULL);
    xx[0] = tx;
    yy[0] = ty;
    xx[1] = bx;
    yy[1] = ty;
    xx[2] = bx;
    yy[2] = by;
    xx[3] = tx;
    yy[3] = by;

    int prev_x2 = 0, prev_y2 = 0;
    for (int i = 0; i < 5; i++)
    {
        int index = i;
        if (i >= 4) index = 0;
        float dist = (float)sqrt((xx[index] * xx[index]) + (yy[index] * yy[index]));
        float angle = (float)acos(xx[index] / dist);
        if (yy[index] < 0) angle = ((float)PI * 2) - angle;

        int x2 = x + (int)(dist * (float)sin(angle + rotation));
        int y2 = y + (int)(dist * (float)cos(angle + rotation));
        if (i > 0)
            drawLine(img, img_width, img_height, x2, y2, prev_x2, prev_y2, r, g, b, line_width, false);
        prev_x2 = x2;
        prev_y2 = y2;
    }
    delete[] xx;
    delete[] yy;
}

/*!
 * \brief drawCross
 * \param img image data
 * \param img_width image width
 * \param img_height image height
 * \param x x coordinate
 * \param y y coordinate
 * \param radius radius of the cross
 * \param r red
 * \param g green
 * \param b blue
 * \param line_width width of lines
 */
void drawing::drawCross(
    unsigned char* img,
    int img_width,
    int img_height,
    int x,
    int y,
    int radius,
    int r,
    int g,
    int b,
    int line_width)
{
    int radius_y = radius * img_width / img_height;
    int tx = x - radius;
    int ty = y - radius_y;
    int bx = x + radius;
    int by = y + radius_y;
    drawLine(img, img_width, img_height, x, ty, x, by, r, g, b, line_width, false);
    drawLine(img, img_width, img_height, tx, y, bx, y, r, g, b, line_width, false);
}

/*!
 * \brief drawCircle
 * \param img image data
 * \param img_width image width
 * \param img_height image height
 * \param x x coordinate
 * \param y y coordinate
 * \param radius radius of the circle
 * \param r red
 * \param g green
 * \param b blue
 * \param line_width width of the line
 */
void drawing::drawCircle(
    unsigned char* img,
    int img_width,
    int img_height,
    int x,
    int y,
    int radius,
    int r,
    int g,
    int b,
    int line_width)
{
    drawCircle(img, img_width, img_height, (float)x, (float)y, (float)radius, r, g, b, line_width);
}

/*!
 * \brief drawCircle
 * \param img
 * \param img_width
 * \param img_height
 * \param x
 * \param y
 * \param radius
 * \param r
 * \param g
 * \param b
 * \param line_width
 */
void drawing::drawCircle(
    unsigned char* img,
    int img_width,
    int img_height,
    float x,
    float y,
    float radius,
    int r,
    int g,
    int b,
    int line_width)
{
    int points = 20;
    int prev_xx = 0, prev_yy = 0;
    for (int i = 0; i < points+1; i++)
    {
        float angle = i * 2 * (float)PI / points;
        int xx = (int)(x + 0.5f + (radius * sin(angle)));
        int yy = (int)(y + 0.5f + (radius * cos(angle)));

        if (i > 0)
            drawLine(img, img_width, img_height, prev_xx, prev_yy, xx, yy, r, g, b, line_width, false);
        prev_xx = xx;
        prev_yy = yy;
    }
}

/*!
 * \brief drawSpot
 * \param img
 * \param img_width
 * \param img_height
 * \param x
 * \param y
 * \param radius
 * \param r
 * \param g
 * \param b
 */
void drawing::drawSpot(
    unsigned char* img,
    int img_width,
    int img_height,
    int x,
    int y,
    int radius,
    int r,
    int g,
    int b)
{
    for (int rr = 1; rr <= radius; rr++)
        drawCircle(img, img_width, img_height, x, y, rr, r, g, b, 1);
}

/*!
 * \brief draws a spot blended with the background
 * \param img
 * \param img_width
 * \param img_height
 * \param x
 * \param y
 * \param radius
 * \param r
 * \param g
 * \param b
 */
void drawing::drawBlendedSpot(
    unsigned char* img,
    int img_width,
    int img_height,
    int x,
    int y,
    int radius,
    int r,
    int g,
    int b)
{
	int dx, dy, xx, yy, dist_sqr, n;
	int radius_sqr = radius*radius;

	for (yy = y - radius; yy <= y + radius; yy++)
	{
		if ((yy > -1) && (yy < img_height))
		{
			dy = yy - y;
		    for (xx = x - radius; xx <= x + radius; xx++)
		    {
		    	if ((xx > -1) && (xx < img_width))
		    	{
		    		dx = xx - x;
		    		dist_sqr = dx*dx + dy*dy;
		    		if (dist_sqr <= radius_sqr)
		    		{
		    			n = ((yy * img_width) + xx) * 3;
		    			img[n] = (unsigned char)((img[n] + b) / 2);
		    			img[n+1] = (unsigned char)((img[n + 1] + g) / 2);
		    			img[n+2] = (unsigned char)((img[n + 2] + r) / 2);
		    		}
		    	}
		    }
		}
	}
}


/*!
 * \brief draw a grid within the given image
 * \param img image to be returned
 * \param img_width width of the image
 * \param img_height height of the image
 * \param centre_x x centre point of the grid
 * \param centre_y y centre point of the grid
 * \param rotation rotation angle of the grid  radians
 * \param columns number of grid columns
 * \param rows number of grid rows
 * \param r red
 * \param g green
 * \param b blue
 * \param linewidth line width
 */
void drawing::drawGrid(
    unsigned char* img,
    int img_width,
    int img_height,
    int centre_x,
    int centre_y,
    float rotation,
    float size_width,
    float size_height,
    int columns,
    int rows,
    int r,
    int g,
    int b,
    int linewidth)
{
    // draw the columns
    for (int col = 0; col <= columns; col++)
    {
        float grid_x = ((col * size_width) / (float)columns) - (size_width/2);
        int prev_x = 0;
        int prev_y = 0;
        for (int row = 0; row <= rows; row += rows)
        {
            float grid_y = ((row * size_height) / (float)rows) - (size_height/2);
            float hyp = (float)sqrt((grid_x*grid_x) + (grid_y*grid_y));
            float angle = 0;
            if (hyp > 0)
            {
                angle = (float)asin(grid_x / hyp);
                if (grid_y < 0) angle = (float)(PI*2)-angle;
            }
            angle += rotation;

            int x = (int)(centre_x + (hyp * sin(angle)));
            int y = (int)(centre_y + (hyp * cos(angle)));

            if (row > 0)
            {
                drawLine(img, img_width, img_height, prev_x, prev_y, x, y,
                         r, g, b, linewidth, false);
            }

            prev_x = x;
            prev_y = y;
        }
    }

    // draw the rows
    for (int row = 0; row <= rows; row ++)
    {
        float grid_y = ((row * size_height) / (float)rows) - (size_height/2);
        int prev_x = 0;
        int prev_y = 0;
        for (int col = 0; col <= columns; col += columns)
        {
            float grid_x = ((col * size_width) / (float)columns) - (size_width/2);
            float hyp = (float)sqrt((grid_x*grid_x) + (grid_y*grid_y));
            float angle = 0;
            if (hyp > 0)
            {
                angle = (float)asin(grid_x / hyp);
                if (grid_y < 0) angle = (float)(PI*2)-angle;
            }
            angle += rotation;

            int x = (int)(centre_x + (hyp * sin(angle)));
            int y = (int)(centre_y + (hyp * cos(angle)));

            if (col > 0)
            {
                drawLine(img, img_width, img_height, prev_x, prev_y, x, y,
                         r, g, b, linewidth, false);
            }

            prev_x = x;
            prev_y = y;
        }
    }

}

/*!
 * \brief draw a line within the given image
 * \param img image to be returned
 * \param img_width width of the image
 * \param img_height height of the image
 * \param x1 top x
 * \param y1 top y
 * \param x2 bottom x
 * \param y2 bottom y
 * \param r red
 * \param g green
 * \param b blue
 * \param linewidth line width
 */
void drawing::drawLine(
    unsigned char* img,
    int img_width,
    int img_height,
    int x1,
    int y1,
    int x2,
    int y2,
    int r,
    int g,
    int b,
    int linewidth,
    bool overwrite)
{
    //printf("line: (%d,%d)-(%d,%d)\n",x1,y1,x2,y2);

    if (img != NULL)
    {
        int w, h, x, y, step_x, step_y, dx, dy, xx2, yy2;
        float m;

        dx = x2 - x1;
        dy = y2 - y1;
        w = ABS(dx);
        h = ABS(dy);
        if (x2 >= x1) step_x = 1; else step_x = -1;
        if (y2 >= y1) step_y = 1; else step_y = -1;

        if ((w < img_width + 2000) && (h < img_height + 2000))
        {

            if (w > h)
            {
                if (dx != 0)
                {
                    m = dy / (float)dx;
                    x = x1;
                    int s = 0;
                    while (s * ABS(step_x) <= ABS(dx))
                    {
                        y = (int)(m * (x - x1)) + y1;

                        for (xx2 = x - linewidth; xx2 <= x + linewidth; xx2++)
                            for (yy2 = y - linewidth; yy2 <= y + linewidth; yy2++)
                            {
                                if ((xx2 >= 0) && (xx2 < img_width) && (yy2 >= 0) && (yy2 < img_height))
                                {
                                    int n = ((img_width * yy2) + xx2) * 3;
                                    if ((img[n] == 0) || (!overwrite))
                                    {
                                        img[n] = (unsigned char)b;
                                        img[n + 1] = (unsigned char)g;
                                        img[n + 2] = (unsigned char)r;
                                    }
                                    else
                                    {
                                        img[n] = (unsigned char)((img[n] + b) / 2);
                                        img[n + 1] = (unsigned char)((img[n] + g) / 2);
                                        img[n + 2] = (unsigned char)((img[n] + r) / 2);
                                    }
                                }
                            }

                        x += step_x;
                        s++;
                    }
                }
            }
            else
            {
                if (dy != 0)
                {
                    m = dx / (float)dy;
                    y = y1;
                    int s = 0;
                    while (s * ABS(step_y) <= ABS(dy))
                    {
                        x = (int)(m * (y - y1)) + x1;
                        for (xx2 = x - linewidth; xx2 <= x + linewidth; xx2++)
                            for (yy2 = y - linewidth; yy2 <= y + linewidth; yy2++)
                            {
                                if ((xx2 >= 0) && (xx2 < img_width) && (yy2 >= 0) && (yy2 < img_height))
                                {
                                    int n = ((img_width * yy2) + xx2) * 3;
                                    if ((img[n] == 0) || (!overwrite))
                                    {
                                        img[n] = (unsigned char)b;
                                        img[n + 1] = (unsigned char)g;
                                        img[n + 2] = (unsigned char)r;
                                    }
                                    else
                                    {
                                        img[n] = (unsigned char)((img[n] + b) / 2);
                                        img[n + 1] = (unsigned char)((img[n] + g) / 2);
                                        img[n + 2] = (unsigned char)((img[n] + r) / 2);
                                    }
                                }
                            }

                        y += step_y;
                        s++;
                    }
                }
            }
        }
    }
}
