/*
 * simple_michinton_cell_visualization.cpp
 *
 *  Created on: 19/10/2012
 *      Author: _filipe
 */

#include "cell.h"
#include <stdio.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace cv;
using namespace nn_lib::cell;

int
main(void)
{
	const char *filename = "../../data/images/lena.jpg";
	MinchintonCell cell_center, cell_corner;

	IplImage *img = cvLoadImage (filename, CV_LOAD_IMAGE_COLOR);

	if (img == NULL)
		exit(printf("Image '%s' not found\n", filename));

	// set the attention point of the cell to the middle of the image
	cell_center.set_attention_point_coordinates(img->width / 2, img->height / 2);
	// set image coordinates
	cell_center.set_image_size(img->height, img->width);
	// create the cell stochastic gaussian distribution with 20 pixels of radius and 500 points
	cell_center.generate_gaussian_distribution(20, 500);
	// draw the gaussian distribution in the image
	cell_center.draw(img);

	// set the attention point of the cell to the upper left point of the image
	cell_corner.set_attention_point_coordinates(0, 0);
	// set image coordinates
	cell_corner.set_image_size(img->height, img->width);
	// create the cell stochastic gaussian distribution with 30 pixels of radius and 700 points
	cell_corner.generate_gaussian_distribution(30, 700);
	// draw the gaussian distribution in the image
	cell_corner.draw(img);

	cvShowImage("gaussian distribution", img);
	cvWaitKey(-1);

	cvReleaseImage(&img);

	printf("Terminou!\n");
	return 0;
}
