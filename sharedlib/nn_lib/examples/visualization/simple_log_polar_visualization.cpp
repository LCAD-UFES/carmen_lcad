/*
 * simple_log_polar_visualization.cpp
 *
 *  Created on: 19/10/2012
 *      Author: _filipe
 */

#include "cell.h"
#include "neuron.h"
#include "util.h"

#include <stdio.h>
#include <string>
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace cv;
using namespace std;
using namespace nn_lib::util;

string image_filename = "../../data/images/lena.jpg";

IplImage *reference_image, *showed_image;
CvFont font;


void
init_font()
{
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0, 0, 2, CV_AA);
}


void
load_image()
{
	reference_image = cvLoadImage(image_filename.c_str(), CV_LOAD_IMAGE_COLOR);
	showed_image = cvLoadImage(image_filename.c_str(), CV_LOAD_IMAGE_COLOR);

	if (!reference_image || !showed_image)
		exit(printf("Error: image '%s' not found\n", image_filename.c_str()));

	cvPutText(showed_image, "Clique em um ponto", cvPoint(30, 30), &font, cvScalar(255, 255, 255, 255));
	cvPutText(showed_image, "da imagem", cvPoint(50, 70), &font, cvScalar(255, 255, 255, 255));
}


void
perform_log_polar_transformation(int x, int y)
{
	LogPolarReceptor receptor;
	receptor.transform(reference_image, showed_image, x, y);
}


void
mouse_callback_function(int event, int x, int y, int flags, void *param)
{
	switch(event)
	{
		case CV_EVENT_LBUTTONDOWN:
			memset(showed_image->imageData, 0, reference_image->imageSize);

			perform_log_polar_transformation(x, y);

			cvPutText(showed_image, "Pressione space ", cvPoint(30, 30), &font, cvScalar(255, 255, 255, 255));
			cvPutText(showed_image, "para reiniciar", cvPoint(70, 70), &font, cvScalar(255, 255, 255, 255));

			break;
	}
}


void
reinit_process()
{
	memcpy(showed_image->imageData, reference_image->imageData, reference_image->imageSize);

	cvPutText(showed_image, "Clique em um ponto", cvPoint(30, 30), &font, cvScalar(255, 255, 255, 255));
	cvPutText(showed_image, "da imagem", cvPoint(50, 70), &font, cvScalar(255, 255, 255, 255));
}


int
main(int argc, char** argv)
{
	int first = 1;

	// init_font();
	load_image();

	cvNamedWindow("image", 1);
	cvNamedWindow("image log polar", 1);
	//cvSetMouseCallback("image", mouse_callback_function, (void*) showed_image);

	CvCapture* capture = cvCaptureFromCAM(0);

	if (!capture)
		exit(printf("Error: Capture initialization failed! Please check if the camera is turned on!\n"));

	LogPolarReceptor receptor;

	while(1)
	{
		reference_image = cvQueryFrame(capture);

		if (first)
		{
			showed_image = cvCreateImage(cvSize(reference_image->width, reference_image->height), reference_image->depth, reference_image->nChannels);
			first = 0;
		}

		receptor.transform(reference_image, showed_image, reference_image->width/2, reference_image->height/2);

		cvLine(
			reference_image,
			cvPoint(reference_image->width/2 - 10, reference_image->height/2),
			cvPoint(reference_image->width/2 + 10, reference_image->height/2),
			cvScalar(0, 255, 0, 0),
			2, 1, 0
		);

		cvLine(
			reference_image,
			cvPoint(reference_image->width/2, reference_image->height/2 - 10),
			cvPoint(reference_image->width/2, reference_image->height/2 + 10),
			cvScalar(0, 255, 0, 0),
			2, 1, 0
		);

		cvShowImage("image", reference_image);
		cvShowImage("image log polar", showed_image);

//		key = cvWaitKey(10);
//
//		if (key == ' ') reinit_process();
//		else if ((key & 255) == 27) break;

		if ((cvWaitKey(10) & 255) == 27) break;
	}

	cvDestroyWindow("image");
	return 0;
}
