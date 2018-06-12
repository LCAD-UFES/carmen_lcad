#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netdb.h>
#include <string.h>
#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <raspicam/raspicam.h>


using namespace std;
using namespace cv;


typedef struct _CameraParameters
{
	double width, height;
	double fx, fy;
	double cx, cy;
	double k1, k2, k3;
	double p1, p2;
}CameraParameters;
