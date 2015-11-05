/*
 * Copyright (C) 2009 Giacomo Spigler
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
 */

//2 Webcams!

#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdio.h>

#include "libcam.h"

using namespace std;


int main()
{
  int ww=640;
  int hh=480;

  Camera c("/dev/video0", ww, hh, 15);
  Camera c2("/dev/video1", ww, hh, 15);

  cvNamedWindow("l", CV_WINDOW_AUTOSIZE);
  cvNamedWindow("r", CV_WINDOW_AUTOSIZE);

  IplImage *l=cvCreateImage(cvSize(ww, hh), 8, 3);
  IplImage *r=cvCreateImage(cvSize(ww, hh), 8, 3);

  while(1)
  {
    c.GrabFrame();
    c2.GrabFrame();

    c.RetrieveFrame();
    c2.RetrieveFrame();

    c.toIplImage(l);
    c2.toIplImage(r);

    cvShowImage("l", l);
    cvShowImage("r", r);

    if( (cvWaitKey(10) & 255) == 27 )
    	break;
  }

  cvDestroyWindow("l");
  cvReleaseImage(&l);
  cvDestroyWindow("r");
  cvReleaseImage(&r);

  return 0;
}





