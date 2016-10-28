/*  Copyright 2011 AIT Austrian Institute of Technology
*
*   This file is part of OpenTLD.
*
*   OpenTLD is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   OpenTLD is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with OpenTLD.  If not, see <http://www.gnu.org/licenses/>.
*
*/

/*
 * gui.cpp
 *
 *  Created on: Oct 18, 2011
 *      Author: clemensk
 */

#include "gui.h"

#include <string>

using std::string;

static string window_name;
static cv::Mat *img0 = 0;
static cv::Mat *img1 = 0;
static cv::Point clicked_point;
static CvRect *bb;
static int drag = 0;

static void mouseHandler(int event, int x, int y, int flags, void *param)
{
	// just to avoid the unused warnings
	(void) flags;
	(void) param;

	/* user press left button */
	if(event == CV_EVENT_LBUTTONDOWN && !drag)
	{
		clicked_point = cv::Point(x, y);
		drag = 1;
	}

	/* user drag the mouse */
	if(event == CV_EVENT_MOUSEMOVE && drag)
	{
		//img1 = (IplImage *) cvClone(img0);
		memcpy(img1->data, img0->data, img0->rows * img0->cols * img0->channels() * sizeof(uchar));
		cv::rectangle(*img1, clicked_point, cv::Point(x, y), cv::Scalar(255, 0, 0), 1, 8, 0);
		cv::imshow(window_name.c_str(), *img1);
	}

	/* user release left button */
	if(event == CV_EVENT_LBUTTONUP && drag)
	{
		*bb = cvRect(clicked_point.x, clicked_point.y, x - clicked_point.x, y - clicked_point.y);
		drag = 0;
	}
}

// TODO: member of Gui
// --> problem: callback function mouseHandler as member!
int getBBFromUser(cv::Mat *img, CvRect &rect, string windowName)
{
	if (img0 == 0)
	{
		img0 = new cv::Mat(cv::Size(img->cols, img->rows), CV_8UC3);
		img1 = new cv::Mat(cv::Size(img->cols, img->rows), CV_8UC3);
	}

	memcpy(img0->data, img->data, img->rows * img->cols * img->channels() * sizeof(uchar));
    //img0 = (IplImage *) cvClone(img);

	window_name = windowName;
    rect = cvRect(-1, -1, -1, -1);
    bb = &rect;
    bool correctBB = false;
    cv::setMouseCallback(window_name.c_str(), mouseHandler, NULL);
    cv::putText(*img0, "Draw a bounding box and press Enter", cv::Point(0, 60),
    		cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar::all(255), 1, 8);
    cv::imshow(window_name.c_str(), *img0);

    while(!correctBB)
    {
        char key = cv::waitKey(0);

        if(tolower(key) == 'q')
        {
            return 0;
        }

        if(((key == '\n') || (key == '\r') /*|| (key == '\r\n')*/) && (bb->x != -1) && (bb->y != -1))
        {
            correctBB = true;
        }
    }

    if(rect.width < 0)
    {
        rect.x += rect.width;
        rect.width = abs(rect.width);
    }

    if(rect.height < 0)
    {
        rect.y += rect.height;
        rect.height = abs(rect.height);
    }

//    cvSetMouseCallback(window_name.c_str(), NULL, NULL);
//    cvReleaseImage(&img0);
//    cvReleaseImage(&img1);

    return 1;
}
