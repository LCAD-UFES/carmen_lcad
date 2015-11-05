/*
 * simple_face_parts_recognition.cpp
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
using namespace nn_lib::neuron;
using namespace nn_lib::cell;
using namespace nn_lib::util;

const char *image_filename = "../../data/images/lena.jpg";

int num_sinapsis = 500;
int gaussian_radius = 50;

NeuronVGRAM<string> neuron;
MinchintonCell cell;

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
	reference_image = cvLoadImage(image_filename, CV_LOAD_IMAGE_COLOR);
	showed_image = cvLoadImage(image_filename, CV_LOAD_IMAGE_COLOR);

	if (!reference_image || !showed_image)
		exit(printf("Error: image '%s' not found\n", image_filename));

	cvPutText(showed_image, "Passe o mouse", cvPoint(30, 30), &font, cvScalar(255, 255, 255, 255));
	cvPutText(showed_image, "sobre a imagem", cvPoint(50, 70), &font, cvScalar(255, 255, 255, 255));
}


void
initialize_neuron_and_cell(int image_width, int image_height)
{
	cell.set_image_size(reference_image->height, reference_image->width);
	cell.generate_gaussian_distribution(20, 300);
}


void
perform_train ()
{
	// train eye
	cell.set_attention_point_coordinates(270, 270);
	neuron.train(cell.filter(reference_image), "olho");

	// train mouth
	cell.set_attention_point_coordinates(299, 350);
	neuron.train(cell.filter(reference_image), "boca");

	// train noose
	cell.set_attention_point_coordinates(301, 314);
	neuron.train(cell.filter(reference_image), "nariz");

	// train hat
	cell.set_attention_point_coordinates(210, 119);
	neuron.train(cell.filter(reference_image), "chapeu");

	// train background
	cell.set_attention_point_coordinates(57, 55);
	neuron.train(cell.filter(reference_image), "background");
	cell.set_attention_point_coordinates(440, 210);
	neuron.train(cell.filter(reference_image), "background");
}


string
perform_test(int x, int y)
{
	cell.set_attention_point_coordinates(x, y);

	cell.draw(showed_image);
	return neuron.test(cell.filter(reference_image));
}


void
mouse_callback_function(int event, int x, int y, int flags, void *param)
{
	memcpy(showed_image->imageData, reference_image->imageData, reference_image->imageSize);

	string clicked_point_class = perform_test(x, y);
	cvPutText(showed_image, clicked_point_class.c_str(), cvPoint(x, y), &font, cvScalar(0, 255, 0, 0));
}


int
main(int argc, char** argv)
{
	init_font();
	load_image();
	initialize_neuron_and_cell(reference_image->width, reference_image->height);
	perform_train();

	cvNamedWindow("image", 1);
	cvSetMouseCallback("image", mouse_callback_function, (void*) showed_image);

	while(1)
	{
		cvShowImage("image", showed_image);
		if ((cvWaitKey(10) & 255) == 27) break;
	}

	cvDestroyWindow("image");
	return 0;
}
