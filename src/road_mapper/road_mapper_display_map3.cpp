#include <iostream>
#include <stdio.h>
#include <sys/io.h>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <dirent.h>
#include <sys/types.h>
#include <math.h>

#include <opencv2/core/version.hpp>
#if CV_MAJOR_VERSION == 3
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#else
#include <opencv/cv.h>
#include <opencv/highgui.h>
#endif

#define PI	3.14159265

enum lane_marking_type { NO_MARKING, BROKEN_WHITE, SOLID_WHITE, BROKEN_YELLOW, SOLID_YELLOW,
                         DOUBLE_BROKEN_YELLOW, DOUBLE_SOLID_YELLOW };

struct pixel_str			/* Probabilities of a pixel in the lane map */
{
	unsigned short off_road;			/* Probability of a pixel off road */
	unsigned short solid_marking;	/* Probability of pixel in the lane's solid marking */
	unsigned short broken_marking;	/* Probability of pixel in the lane's broken marking */
	unsigned short lane_center;		/* Probability of pixel in lane center */
};

union lane_map_union
{
	pixel_str pixel;
	char pixel_data[8];
} 	lane_map;

std::string window_name1 = "map probabilities";
#define	width	350
#define height	350
#define MAX_PROB (pow(2.0, 16) - 1.0)

cv::Mat image1(height, width, CV_8UC3, cv::Scalar::all(0));

int main(int argc, char** argv)
{
	std::string input_file;
	std::ifstream input;

	if (argc != 2)
	{
		std::cerr << argv[0] << " <road_map>.map" << std::endl;
		return -1;
	}

	input_file = argv[1];

	input.open(input_file.c_str());

	if (!input.is_open())
	{
		std::cerr << "\n" <<
		"------------------------------------------------------------------------------------------" << std::endl <<
		"Failed! COULD NOT OPEN FILE: " << input_file.c_str() << std::endl <<
		"------------------------------------------------------------------------------------------" << "\n\n";
		return -1;
	}

	cv::namedWindow(window_name1, 1);
    cv::moveWindow(window_name1, 78 + width, 10);

	int x = 0, y = 0;

	input.read(lane_map.pixel_data, 8);

	while (input.gcount() == 8)
	{
		if (lane_map.pixel.off_road == 0)
		{
            uchar blue = (uchar) round(255.0 * lane_map.pixel.broken_marking / MAX_PROB);
            uchar green = (uchar) round(255.0 * lane_map.pixel.lane_center / MAX_PROB);
            uchar red = (uchar) round(255.0 * lane_map.pixel.solid_marking / MAX_PROB);

			cv::Vec3b color;
			color[0] = blue;
			color[1] = green;
			color[2] = red;
			image1.at<cv::Vec3b>(height - 1 - y, x) = color;
		}
		x++;
		if (x == width)
		{
			x = 0;
			y++;
			if (y == height)
				break;
		}
		input.read(lane_map.pixel_data, 8);
	}
	if (y == height)
		std::cout << "File successfully read: " << width << " * " << height << "\n";
	else
		std::cout << "File partially read: x = " << x << " , y = " << y << "\n";
	input.close();
    cv::imshow(window_name1, image1);
	std::cout << "\nPress \"Esc\" key to continue...\n";
	while(cv::waitKey() != 27);
	image1.release();
	cv::destroyWindow(window_name1);

	return 0;
}
