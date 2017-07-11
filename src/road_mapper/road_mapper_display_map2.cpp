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

struct pixel_str			/* Attributes of a pixel inside a road lane */
{
	short distance_center;	/* Distance in millimeters from the pixel to the center of the lane.
                               Positive value if the pixel is on the right hand.
                               Negative value if the pixel is on the left hand. */
	short x_orientation;	/* X component of lane orientation vector */
	short y_orientation;	/* Y component of lane orientation vector */
	char left_marking;		/* Lane marking type on the left hand */
	char right_marking;		/* Lane marking type on the right hand */
};

union lane_map_union
{
	pixel_str pixel;
	char pixel_data[8];
	long off_road;			/* If pixel is off the road = (-1); otherwise pixel is inside a road lane  */
} 	lane_map;

std::string window_name1 = "distance to center of lane";
std::string window_name2 = "lane orientation";
#define	width	350
#define height	350

cv::Mat image1(height, width, CV_8UC3, cv::Scalar::all(0));
cv::Mat image2(height, width, CV_8UC3, cv::Scalar::all(0));

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
	cv::namedWindow(window_name2, 1);
    cv::moveWindow(window_name1, 78 + width, 10);
    cv::moveWindow(window_name2, 78 + width, 128 + height);

	int x = 0, y = 0;

	lane_map.off_road = -1;
	input.read(lane_map.pixel_data, 8);

	while (input.gcount() == 8)
	{
		if (lane_map.off_road != -1)
		{
			cv::Vec3b color;
			color[0] = lane_map.pixel.right_marking + lane_map.pixel.left_marking * 10; // blue = lane markings
			int distance = lane_map.pixel.distance_center / 10.0 + 0.5; // distance in centimeters
			color[1] = (!std::signbit(distance)) * distance; // green = positive degrees
			color[2] = std::signbit(distance) * abs(distance); // red = negative degrees
			image1.at<cv::Vec3b>(height - 1 - y, x) = color;
            float orientation = atan2(lane_map.pixel.y_orientation, lane_map.pixel.x_orientation);
            int degrees = orientation / PI * 180 + 0.5; // orientation in range (-180, 180) degrees
            color[1] = (!std::signbit(degrees)) * degrees; // green = positive degrees
            color[2] = std::signbit(degrees) * abs(degrees); // red = negative degrees
            image2.at<cv::Vec3b>(height - 1 - y, x) = color;
		}
		x++;
		if (x == width)
		{
			x = 0;
			y++;
			if (y == height)
				break;
		}
		lane_map.off_road = -1;
		input.read(lane_map.pixel_data, 8);
	}
	if (y == height)
		std::cout << "File successfully read: " << width << " * " << height << "\n";
	else
		std::cout << "File partially read: x = " << x << " , y = " << y << "\n";
	input.close();
    cv::imshow(window_name1, image1);
    cv::imshow(window_name2, image2);
	std::cout << "\nPress \"Esc\" key to continue...\n";
	while(cv::waitKey() != 27);
	image1.~Mat();
	image2.~Mat();
	cv::destroyWindow(window_name1);
	cv::destroyWindow(window_name2);
	return 0;
}
