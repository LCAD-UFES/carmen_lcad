/*******************GENERATE_GT***********************

 Compile:
 g++ -o generate_gt generate_gt_new.cpp -W -Wall `pkg-config --cflags opencv` -O4 `pkg-config --libs opencv`

 *************************************************/


#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/legacy/legacy.hpp>

#include <stdio.h>
#include <sys/io.h>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <dirent.h>
#include <sys/types.h>

#include "spline.h"

using namespace std;
using namespace cv;

Mat image, image2;
string window_name1 = "IMG1";
string window_name2 = "IMG2";

string
IntToString(int num)
{
	ostringstream myStream;
	myStream << num << flush;

	return (myStream.str());
}

void
on_mouse(int event, int x, int y, int flag, void *param)
{
	flag = flag;
	param = param;

	if (event == CV_EVENT_LBUTTONDOWN)
	{
//		Point c = Point(x, y);
//		g_points_to_spline[i].push_back(c);
//		cv::circle(image, c, 100, Scalar(255, 0, 0));
	}
}


void
could_not_open_error_message(string type, string name)
{
	cerr << "\n" <<
	"------------------------------------------------------------------------------------------" << endl <<
	"Failed! COLD NOT OPEN " << type << ": " << name << endl <<
	"------------------------------------------------------------------------------------------" << "\n\n";
}


void
write_marked_images_to_file(vector<string> images, string file_name)
{
	ofstream file;
	file.open(file_name.c_str(), ofstream::out | ofstream::out);

	for (unsigned int i = 0; i < images.size(); i++)
	{
		file << images[i];
	}

	file.close();
}

void thresh_callback(cv::Mat src_gray)
{
	RNG rng(12345);
	int thresh = 100;
	int max_thresh = 255;

	Mat canny_output;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	/// Detect edges using canny
	cv::Canny( src_gray, canny_output, thresh, thresh*2, 3 );
	/// Find contours
	findContours( canny_output, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point(0, 0) );

	/// Draw contours
	Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
	cout << contours.size() << endl;
	cout << hierarchy.size() << endl;

	namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
	for( int i = 0; i< contours.size(); i++ )
	{
		Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		drawContours( drawing, contours, i, color, 1, 8/*, hierarchy, 0, Point() */);
	}
	/// Show in a window
	imshow( "Contours", drawing );
	//cv::waitKey(0);
}

int
main(int argc, char** argv)
{
	int iKey = -1;
	unsigned int i, actual_position = 0;
	string s, line, image_name, image_name2, input_file, strPostfix = "";
	vector<string> strings;
	ifstream input;
	bool first_rect_drawn = false , going_back = false;

	if (argc != 2)
	{
		cerr << argv[0] << " input_ImageList.txt" << endl;
		return -1;
	}

	input_file = argv[1];

	input.open(input_file.c_str());

	namedWindow(window_name1, 1);

	if (input.is_open())
	{
		getline(input, line);

		while (!input.eof())
		{
			if (line == "") break;
			istringstream iss(line);
			for (i = 0; getline(iss, s, ' '); i++)
			{
				strings.push_back(s);
			}
			image_name = strings[0];
			image_name2 = strings[1];

			cout << "Loading image :" << image_name;
			image = imread(image_name, IMREAD_COLOR);
			image2 = imread(image_name2, IMREAD_COLOR);

			if (!image.empty())
			{
				// do stuff
				imshow(window_name2, image2);

				cv::Mat channels[3];

				cv::split(image2, channels);
				//imshow("BLUE", channels[0]);
				//imshow("GREEN", channels[1]);
				//imshow("RED", channels[2]);

//				int r, c;
//				for (r = 0; r < channels[1].rows; r++)
//				{
//					for (c = 0; c < channels[1].cols; c++)
//					{
//						if (channels[1].at<uchar>(r * channels[1].cols + c) > 127)
//						{
//							cv::Point p;
//							p.x = c;
//							p.y = r;
//							cv::circle(image, p, 1, Scalar(255, 0, 0));
//						}
//					}
//				}

				thresh_callback(image2);

				setMouseCallback(window_name2, on_mouse, NULL);

				imshow(window_name1, image);
				cv::waitKey(0);
				image.release();
			}
			else
			{
				could_not_open_error_message("IMAGE", image_name);
			}
			strings.clear();
			strPostfix.empty();

			getline(input, line);
		}
	}
	else
	{
		could_not_open_error_message("FILE", input_file);
	}

	printf("AE\n");
	image.release();
	printf("AE\n");
	image2.release();
	printf("AE\n");
	input.close();
	printf("AE\n");
	//destroyWindow(window_name1);
	printf("AE\n");

	return EXIT_SUCCESS;
}
