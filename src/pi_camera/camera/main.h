#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <raspicam/raspicam.h>
#include <iostream>
#include <unistd.h>

typedef struct _CameraParameters
{
	double width, height;
	double fx, fy;
	double cx, cy;
	double k1, k2, k3;
	double p1, p2;
}CameraParameters;
