/*
 * Copyright (C) 2009 Giacomo Spigler
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
 */

//Single webcam demo

#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdio.h>

#include "libcam.h"

using namespace std;


int main(int argc, char **args) {
  int ww=640;
  int hh=480;
  int fps=30;

  const char *dev="/dev/video1";


  printf("Usage is:\n    %s -w width -h height -d device -f fps\n\n", args[0]);


  //Processing arguments
  for(int i=1; i<argc-1; i++) {
    string a=args[i];
    if(a=="-w") {
      ww=atoi(args[i+1]);
    } else if(a=="-h") {
      hh=atoi(args[i+1]);
    } else if(a=="-d") {
      dev=args[i+1];
    } else if(a=="-f") {
      fps=atoi(args[i+1]);
    }

  }


  // 1) Instance a Camera object
  Camera c(dev, ww, hh, fps);



  cvNamedWindow("l", CV_WINDOW_AUTOSIZE);

  IplImage *l=cvCreateImage(cvSize(ww, hh), 8, 3);


  while(1){
    // 2) Grab next frame
    c.GrabFrame();
    c.RetrieveFrame();


    // 3) Convert to OpenCV format  (default is YUYV, stored into c.data[] )
    c.toIplImage(l);


    cvShowImage("l", l);

    if( (cvWaitKey(10) & 255) == 27 ) break;
  }



  cvDestroyWindow("l");
  cvReleaseImage(&l);


  // 4) Automatic cleanup is done when the app terminates

  return 0;
}





