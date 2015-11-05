/*
 * ocr_main.cpp
 *
 *  Created on: 31/10/2012
 *      Author: filipe
 */

#include <stdio.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>

#include "../../nn_lib/cell.h"
#include "../../nn_lib/neuron.h"


using namespace std;
using namespace nn_lib::cell;
using namespace nn_lib::neuron;


/********************/
/**    NN Vars     **/
/********************/

int num_neurons = 5;
int cell_width = 100;
int cell_height = 100;
vector<NeuronVGRAM<char> > neurons;
vector<MinchintonCell> neuron_input_cells;

/********************/
/** Interface vars **/
/********************/

int program_interface_width = 500;
int program_interface_height = 500;

IplImage *program_interface_image = 0;

CvFont font;

int DRAWING_INACTIVE = 0;
int DRAWING_ACTIVE = 1;

int drawing_state = DRAWING_INACTIVE;


void
init_font()
{
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 2, CV_AA);
}


void
check_image_boudaries(IplImage *img, CvPoint a, CvPoint b)
{
	if (a.x < 0 || a.y < 0 || b.x < 0 || b.y < 0)
		exit(printf("Error: trying to draw rectangles with negative points\n"));

	if (a.x > img->width || b.x > img->width || a.y > img->height || b.y > img->height)
		exit(printf("Error: trying to draw rectangles with out of bounds points\n"));
}


void
draw_filled_rectangle(IplImage *img, CvPoint a, CvPoint b, CvScalar color)
{
	int i, j, k, img_pos;

	check_image_boudaries(img, a, b);

	for(i = a.y; i < b.y; i++)
	{
		for(j = a.x; j < b.x; j++)
		{
			img_pos = img->nChannels * (i * img->width + j);

			for(k = 0; k < img->nChannels; k++)
			{
				img->imageData[img_pos + k] = color.val[k];
			}
		}
	}
}


void
initialize_program_interface()
{
	program_interface_image = cvCreateImage(cvSize(program_interface_width, program_interface_height), IPL_DEPTH_8U, 3);

	if (program_interface_image == NULL)
		exit(printf("Error: program_interface_image could not be created!\n"));

	draw_filled_rectangle(program_interface_image, cvPoint(0, 0), cvPoint(program_interface_width, program_interface_height), cvScalar(255, 255, 255, 255));
	draw_filled_rectangle(program_interface_image, cvPoint(100, 20), cvPoint(program_interface_width - 100, 70), cvScalar(255, 0, 0, 0));
	draw_filled_rectangle(program_interface_image, cvPoint(100, program_interface_height - 80), cvPoint(program_interface_width - 100, program_interface_height - 30), cvScalar(255, 255, 0, 0));

	cvRectangle(program_interface_image, cvPoint(100, 100), cvPoint(program_interface_width - 100, program_interface_height - 100), cvScalar(0, 0, 255, 0), 1, 1, 0);

	cvPutText(program_interface_image, "Draw a char in the box", cvPoint(170, 50), &font, cvScalar(255, 255, 255, 255));
	cvPutText(program_interface_image, "Classify", cvPoint(230, program_interface_height - 50), &font, cvScalar(255, 255, 255, 255));
}


void
clean_drawing_area()
{
	draw_filled_rectangle(program_interface_image, cvPoint(101, 101), cvPoint(program_interface_width - 100 - 1, program_interface_height - 100 - 1), cvScalar(255, 255, 255, 255));
}


int
find_top_boundary()
{
	int i, j, p, color_r, color_g, color_b;

	for(i = 101; i < (program_interface_height - 100); i++)
	{
		for(j = 101; j < (program_interface_width - 100); j++)
		{
			p = 3 * (i * program_interface_width + j);

			color_b = program_interface_image->imageData[p + 0];
			color_g = program_interface_image->imageData[p + 1];
			color_r = program_interface_image->imageData[p + 2];

			if (color_b == 0 || color_r == 0 || color_g == 255)
				return i;
		}
	}

	return -1;
}


int
find_bottom_boundary()
{
	int i, j, p, color_r, color_g, color_b;

	for(i = (program_interface_height - 101); i > 100; i--)
	{
		for(j = 101; j < (program_interface_width - 100); j++)
		{
			p = 3 * (i * program_interface_width + j);

			color_b = program_interface_image->imageData[p + 0];
			color_g = program_interface_image->imageData[p + 1];
			color_r = program_interface_image->imageData[p + 2];

			if (color_b == 0 || color_r == 0 || color_g == 255)
				return i;
		}
	}

	return -1;
}


int
find_left_boundary()
{
	int i, j, p, color_r, color_g, color_b;

	for(j = 101; j < (program_interface_width - 100); j++)
	{
		for(i = 101; i < (program_interface_height - 100); i++)
		{
			p = 3 * (i * program_interface_width + j);

			color_b = program_interface_image->imageData[p + 0];
			color_g = program_interface_image->imageData[p + 1];
			color_r = program_interface_image->imageData[p + 2];

			if (color_b == 0 || color_r == 0 || color_g == 255)
				return j;
		}
	}

	return -1;
}


int
find_right_boundary()
{
	int i, j, p, color_r, color_g, color_b;

	for(j = (program_interface_width - 101); j > 100; j--)
	{
		for(i = 101; i < (program_interface_height - 100); i++)
		{
			p = 3 * (i * program_interface_width + j);

			color_b = program_interface_image->imageData[p + 0];
			color_g = program_interface_image->imageData[p + 1];
			color_r = program_interface_image->imageData[p + 2];

			if (color_b == 0 || color_r == 0 || color_g == 255)
				return j;
		}
	}

	return -1;
}


CvRect
detect_char_area()
{
	CvRect rect;
	int p_top, p_bottom, p_left, p_right;

	p_top = find_top_boundary();
	p_left = find_left_boundary();
	p_right = find_right_boundary();
	p_bottom = find_bottom_boundary();

	cvRectangle(program_interface_image, cvPoint(p_left - 5, p_top - 5), cvPoint(p_right + 5, p_bottom + 5), cvScalar(0, 0, 0, 0), 1, 1, 0);

	rect.y = p_top;
	rect.x = p_left;
	rect.width = p_right - p_left;
	rect.height = p_bottom - p_top;

	return rect;
}


char
classify_char(Rect rect)
{
	int i, max_voted;
	char result, max_voted_char;
	map<char, int> neuron_votes;
	map<char, int>::iterator it;

	IplImage *resized_img = cvCreateImage(cvSize(cell_width, cell_height), IPL_DEPTH_8U, 3),
		*gray = cvCreateImage(cvSize(cell_width, cell_height), IPL_DEPTH_8U, 1);

	cvSetImageROI(program_interface_image, rect);
	cvResize(program_interface_image, resized_img, CV_INTER_CUBIC);
	cvCvtColor(resized_img, gray, CV_BGR2GRAY);
	cvResetImageROI(program_interface_image);

//	IplImage *test_gaussian_image = cvCreateImage(cvSize(resized_img->width, resized_img->width), IPL_DEPTH_8U, 3);

	for(i = 0; i < num_neurons; i++)
	{
//		memcpy(test_gaussian_image->imageData, resized_img->imageData, resized_img->imageSize * sizeof(char));
//		neuron_input_cells[i].draw(test_gaussian_image);
//		cvShowImage("test", test_gaussian_image);
//		cvWaitKey(-1);

		result = neurons[i].test(neuron_input_cells[i].filter(gray));

		printf("neuron %d said: %c\n", i, result);

		if (neuron_votes.find(result) == neuron_votes.end())
			neuron_votes[result] = 1;
		else
			neuron_votes[result]++;
	}

//	exit(0);

	max_voted_char = ' ';
	max_voted = 0;

	for(it = neuron_votes.begin(); it != neuron_votes.end(); it++)
	{
		if (it->second > max_voted)
		{
			max_voted = it->second;
			max_voted_char = it->first;
		}
	}

	cvReleaseImage(&resized_img);
	cvReleaseImage(&gray);

	return max_voted_char;
}


void
write_result_in_interface(char result)
{
	char message[256];

	sprintf(message, "recognized char: %c", result);

	draw_filled_rectangle(program_interface_image, cvPoint(100, 20), cvPoint(program_interface_width - 100, 70), cvScalar(255, 0, 0, 0));
	cvPutText(program_interface_image, message, cvPoint(170, 50), &font, cvScalar(255, 255, 255, 255));
}


void
perform_classify()
{
	Rect rect;
	char result = ' ';

	rect = detect_char_area();
	result = classify_char(rect);
	write_result_in_interface(result);
}


void
mouse_callback_function(int event, int x, int y, int flags, void *param)
{
	if (y < 100)
		return;

	switch(event)
	{
		case CV_EVENT_LBUTTONDOWN:
		{
			if (x >= 100 && x <= program_interface_width - 100 && y >= 100 && y <= program_interface_height - 100)
			{
				drawing_state = DRAWING_ACTIVE;
			}
			else if (x >= 100 && x <= program_interface_width - 100 && y >= program_interface_height - 80 && y <= program_interface_height - 30)
			{
				perform_classify();
				clean_drawing_area();
			}
			break;
		}
		case CV_EVENT_LBUTTONUP:
			drawing_state = DRAWING_INACTIVE;
			break;
		default:
			break;
	}

	if (drawing_state == DRAWING_ACTIVE)
		draw_filled_rectangle(program_interface_image, cvPoint(x - 5, y - 5), cvPoint(x + 5, y + 5), cvScalar(0, 0, 0, 0));
}


void
initialize_neurons_and_cells()
{
	int i;

	for(i = 0; i < num_neurons; i++)
	{
		NeuronVGRAM<char> neuron;
		MinchintonCell cell;

		cell.set_image_size(cell_height, cell_width);
		cell.set_attention_point_coordinates(cell_height / 2, cell_width / 2);
		cell.generate_gaussian_distribution(30, 4096);

		neurons.push_back(neuron);
		neuron_input_cells.push_back(cell);
	}
}


void
train(char *filename)
{
	initialize_neurons_and_cells();

	int i;
	IplImage *img,
		*gray = cvCreateImage(cvSize(cell_width, cell_height), IPL_DEPTH_8U, 1),
		*resized_img = cvCreateImage(cvSize(cell_width, cell_height), IPL_DEPTH_8U, 3);
	char char_name, char_image_name[256];

	FILE *f = fopen(filename, "r");

	if (f == NULL)
		exit(printf("Error: train data file '%s' not found\n", filename));

	while(!feof(f))
	{
		fscanf(f, "\n%c \t%s\n", &char_name, char_image_name);

		img = cvLoadImage(char_image_name, CV_LOAD_IMAGE_COLOR);

		if (img == NULL)
			exit(printf("Error: train image '%s' not found!\n", char_image_name));

		cvResize(img, resized_img);
		cvCvtColor(resized_img, gray, CV_BGR2GRAY);

		printf("trainning %s with %c\n", char_image_name, char_name);

		for(i = 0; i < num_neurons; i++)
			neurons[i].train(neuron_input_cells[i].filter(gray), char_name);

		cvReleaseImage(&img);
	}

	fclose(f);

	cvReleaseImage(&resized_img);
	cvReleaseImage(&gray);
}


int
main(int argc, char **argv)
{
	if (argc < 2)
		exit(printf("Use %s <train-data-file>\n", argv[0]));

	train(argv[1]);

	init_font();
	initialize_program_interface();

	cvNamedWindow("ocr", 1);
	cvSetMouseCallback("ocr", mouse_callback_function, (void*) program_interface_image);

	while(1)
	{
		cvShowImage("ocr", program_interface_image);
		if ((cvWaitKey(10) & 255) == 27) break;
	}

	printf("Terminou!\n");
	return 0;
}
