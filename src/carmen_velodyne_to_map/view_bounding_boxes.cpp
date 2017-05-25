/*
 * view_bounding_boxes.cpp
 *
 *  Created on: May 24, 2017
 *      Author: luan
 */

#include <stdio.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


int
main(int argc, char **argv)
{
	setlocale(LC_ALL, "C");

	if (argc < 2)
	{
		printf("Usage: %s timestamps.txt\n", argv[0]);
		return (1);
	}

	FILE *labels;

	char imagename[256];
	char labelname[256];

	double timestamp;

	FILE *timestamps = fopen("/dados/dataset/timestamps.txt","r");

	cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
	double xt, yt, xb, yb;
	char classe[10];

	setlocale(LC_ALL, "C");
	while (!feof(timestamps))
	{
		//setlocale(LC_ALL, "C");
		fscanf(timestamps, "%lf\n", &timestamp);
		sprintf(imagename, "/dados/dataset/map/%lf.png", timestamp);
		sprintf(labelname, "/dados/dataset/labels/%lf.txt", timestamp);

		labels = fopen(labelname, "r");
		cv::Mat image;
		image = cv::imread(imagename, CV_LOAD_IMAGE_COLOR);   // open image

		if (labels)
		{
			while (!feof(labels))
			{
				fscanf(labels, "%s 0.00 0 0.00 %lf %lf %lf %lf 0.00 0.00 0.00 0.00 0.00 0.00 0\n", classe, &xt, &yt, &xb, &yb);
				cv::rectangle(image, cv::Point(xt,yt), cv::Point(xb,yb), cv::Scalar(0,255,0), 3);
			}

			fclose(labels);
		}

		cv::imshow( "Display window", image );                   // Show our image inside it.
		cv::waitKey(0);

	}

	return (0);
}
