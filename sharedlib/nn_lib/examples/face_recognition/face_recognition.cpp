/*
 * facedetect.cpp
 *
 *  Created on: 29/10/2012
 *      Author: _filipe
 */

#include <iostream>
#include <cstdio>
#include <vector>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <cell.h>
#include <neuron.h>
#include <util.h>

using namespace std;
using namespace cv;
using namespace nn_lib::neuron;
using namespace nn_lib::cell;


// ***************************************************
// Default Parameters from OpenCV Face Detect. Example
// ***************************************************
String cascadeName = "../../data/haarcascades/haarcascade_frontalface_alt.xml";
String nestedCascadeName = "../../data/haarcascades/haarcascade_eye_tree_eyeglasses.xml";
double scale = 1.3;


// ***********
// Color Table
// ***********
const static Scalar colors[] =
{
	CV_RGB(0,0,255),
	CV_RGB(0,128,255),
	CV_RGB(0,255,255),
	CV_RGB(0,255,0),
	CV_RGB(255,128,0),
	CV_RGB(255,255,0),
	CV_RGB(255,0,0),
	CV_RGB(255,0,255)
};


CvFont font;

void
init_font()
{
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0, 0, 0, CV_AA);
}


void
write_on_image(IplImage *img, char *text, CvPoint position, CvScalar color)
{
	cvPutText(img, text, position, &font, color);
}

void
draw_faces(vector<Rect> faces, Mat &img)
{
	int i = 0;
	Point center;

	for(vector<Rect>::const_iterator r = faces.begin(); r != faces.end(); r++, i++)
	{
		Scalar color = colors[i % 8];
		Rect face_container = Rect(r->x * scale, r->y * scale, r->width * scale, r->height * scale);
		rectangle(img, face_container, color, 1, 1, 0);
	}
}


void
detect_and_draw(IplImage* img_in, CascadeClassifier &cascade, double scale, vector<Rect> &faces)
{
	double t = 0;

	Mat img = img_in;
	Mat smallImg(cvRound (img.rows / scale), cvRound(img.cols / scale), CV_8UC1);

	resize(img, smallImg, smallImg.size(), 0, 0, INTER_LINEAR);
	equalizeHist(smallImg, smallImg);

	t = (double) cvGetTickCount();

	cascade.detectMultiScale(
		smallImg, faces,
		1.1, 2, 0
		//|CV_HAAR_FIND_BIGGEST_OBJECT,
		//|CV_HAAR_DO_ROUGH_SEARCH,
		|CV_HAAR_SCALE_IMAGE,
		Size(30, 30)
	);

	t = (double) cvGetTickCount() - t;
	printf("detection time = %g ms\n", t / ((double) cvGetTickFrequency() * 1000.));
}


string
select_the_most_voted_class(map<string, int> &classes_votation)
{
	int is_first_iteraction, num_votes, max_num_votes;
	map<string, int>::iterator it, it_max;
	
	it_max = classes_votation.begin();
	is_first_iteraction = 1; 
	max_num_votes = 0;
	
	for(it = classes_votation.begin(); it != classes_votation.end(); it++)
	{
		num_votes = it->second;
		
		if ((num_votes > max_num_votes) || is_first_iteraction)
		{
			it_max = it;
			max_num_votes = num_votes;
			
			is_first_iteraction = 0;
		}
	}
	
	if (it_max == classes_votation.end())
		exit(printf("Error: empty votation map\n"));
	
	return it_max->first;
}


void
recog_and_label_faces(IplImage *frame, vector<Rect> faces, vector<NeuronVGRAM<string> > &neuron_layer, MinchintonCell &cell, double scale)
{
	int i, y, x, mean_x, mean_y;
	IplImage *test_img = cvCreateImage(cvSize(100, 200), IPL_DEPTH_8U, 1);
	map<string, int> classes_votation;
	string face_class;
	Rect face;
	
	for(i = 0; i < (int) faces.size(); i++)
	{
		face = cvRect(faces[i].x * scale, faces[i].y * scale, faces[i].width * scale, faces[i].height * scale);
		
		cvSetImageROI(frame, face);
		cvResize(frame, test_img);
		cvResetImageROI(frame);

		mean_x = (face.x + face.width) / 2;
		mean_y = (face.y + face.height) / 2;

		for(y = 0; y < test_img->height; y++)
		{
			for(x = 0; x < test_img->width; x++)
			{
				cell.set_attention_point_coordinates(x, y);
				face_class = neuron_layer[y * test_img->width + x].test(cell.filter(test_img));
				
				if (classes_votation.find(face_class) == classes_votation.end())
					classes_votation[face_class] = 1;
				else
					classes_votation[face_class]++;
			}
		}
		
		face_class = select_the_most_voted_class(classes_votation);
		
		write_on_image(frame, (char*) face_class.c_str(), cvPoint(mean_x * scale + 100, mean_y * scale + 70), cvScalar(255, 255, 255, 255));
	}
}


void
load_and_filter_image(char *face_img_path, IplImage *face_img)
{
	IplImage *img = cvLoadImage (face_img_path, CV_LOAD_IMAGE_GRAYSCALE);
	
	if (img == NULL)
		exit(printf("Error: Image file '%s' not found\n", face_img_path));
	
	cvResize(img, face_img);
}


void
load_data_and_train_neuron(char *filename, vector<NeuronVGRAM<string> > &neuron_layer, MinchintonCell &cell)
{
	int i, j;
	char face_class[256], face_img_path[256];
	
	FILE* f = fopen(filename, "r");

	if (f == NULL)
		exit(printf("Error: train list file '%s' not found!\n", filename));
	
	IplImage *face_img = cvCreateImage(cvSize(100, 200), IPL_DEPTH_8U, 1);
	
	while(!feof(f))
	{
		fscanf(f, "%s %s", face_class, face_img_path);
		load_and_filter_image(face_img_path, face_img);
		
		for(i = 0; i < face_img->height; i++)
		{
			for(j = 0; j < face_img->width; j++)
			{
				cell.set_attention_point_coordinates(j, i);
				neuron_layer[i * face_img->width + j].train(cell.filter(face_img), face_class);
			}
		}
	}
	
	fclose(f);
	cvReleaseImage(&face_img);
}


void
initialize_cell(MinchintonCell &cell, int cell_height, int cell_width, int num_synapsis, int gaussian_radius)
{
	cell.set_image_size(cell_height, cell_width);
	cell.generate_gaussian_distribution(gaussian_radius, num_synapsis);
	cell.set_attention_point_coordinates(cell_width / 2, cell_height / 2);
}


void
initialize_neuron_layer(vector<NeuronVGRAM<string> > &neuron_layer, int h, int w)
{
	int i;
	int n = (w * h);
	
	for(i = 0; i < n; i++)
		neuron_layer.push_back(NeuronVGRAM<string>());
	
}


int main(int argc, const char** argv)
{
	vector<Rect> faces;
	CvCapture* capture = 0;
	IplImage *frame, *gray = 0;
	CascadeClassifier cascade;
	vector<NeuronVGRAM<string> > neuron_layer;
	MinchintonCell cell;

	if (argc < 2)
		exit(printf("Use %s <training-list>\n", argv[0]));

	capture = cvCaptureFromCAM(0);

	if (!capture)
		exit(printf("Error: capture initialization error. Please check if the camera is turned on\n"));


	if(!cascade.load(cascadeName))
		exit(printf("Error: cascade file '%s' not found!\n", cascadeName.c_str()));


	initialize_cell(cell, 200, 100, 512, 5);
	initialize_neuron_layer(neuron_layer, 200, 100);
	
	load_data_and_train_neuron((char*) argv[1], neuron_layer, cell);
	
	init_font();
	cvNamedWindow("result", 1);

	while(1)
	{
		frame = cvQueryFrame(capture);

		if (gray == 0)
		{
			gray = cvCreateImage(cvSize(frame->width, frame->height), IPL_DEPTH_8U, 1);
		}
			
		cvCvtColor(frame, gray, CV_BGR2GRAY);
		detect_and_draw(gray, cascade, scale, faces);
		recog_and_label_faces(gray, faces, neuron_layer, cell, scale);

		cvShowImage("result", gray);

		if ((cvWaitKey(10) & 255) == 27)
			break;
	}

	cvReleaseCapture(&capture);
	cvDestroyWindow("result");

	return 0;
}

