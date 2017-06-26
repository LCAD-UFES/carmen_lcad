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
#include <math.h>

#define PI	3.14159265

#include "spline.h"


using namespace std;
using namespace cv;

enum lane_marking_type { NO_MARKING, BROKEN_WHITE, SOLID_WHITE,
                         BROKEN_YELLOW, SOLID_YELLOW,
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
	long off_road;			/* If pixel is off the road: (-1); otherwise pixel is inside a road lane  */
	char pixel_data[8];
	pixel_str pixel;
} 	lane_map;

string window_name1 = "distance to center of lane";
string window_name2 = "lane orientation";
#define	width	350
#define height	350

Mat image1(height, width, CV_8UC3, Scalar::all(0));
Mat image2(height, width, CV_8UC3, Scalar::all(0));

int main(int argc, char** argv)
{
	string input_file;
	ifstream input;

	if (argc != 2)
	{
		cerr << argv[0] << " <road_map>.map" << endl;
		return -1;
	}

	input_file = argv[1];

	input.open(input_file.c_str());

	if (!input.is_open())
	{
		cerr << "\n" <<
		"------------------------------------------------------------------------------------------" << endl <<
		"Failed! COULD NOT OPEN FILE: " << input_file.c_str() << endl <<
		"------------------------------------------------------------------------------------------" << "\n\n";
		return -1;
	}

	namedWindow(window_name1, 1);
	namedWindow(window_name2, 1);
    moveWindow(window_name1, 78 + width, 10);
    moveWindow(window_name2, 78 + width, 128 + height);

	int x = 0, y = 0;

	lane_map.off_road = -1;
	input.read(lane_map.pixel_data, 8);

	while (input.gcount() == 8)
	{
		if (lane_map.off_road != -1)
		{
			Vec3b color;
			uchar blue, green, red;
			color[0] = (float) lane_map.pixel.distance_center / 10.0 + 0.5; // blue = distance in centimeters
			color[1] = lane_map.pixel.right_marking;
			color[2] = lane_map.pixel.left_marking;
			image1.at<Vec3b>(height - 1 - y, x) = color;
			float orientation;
			int degrees;
            orientation = atan2(lane_map.pixel.y_orientation, lane_map.pixel.x_orientation);
            degrees = fabs(orientation) / PI * 180 + 0.5; // orientation in range (0, 180) degrees
            color[0] = 0;
            color[1] = (!signbit(orientation)) * degrees; // green = positive degrees
            color[2] = signbit(orientation) * degrees; // red = negative degrees
            image2.at<Vec3b>(height - 1 - y, x) = color;
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
		cout << "File successfully read: " << width << " * " << height << "\n";
	else
		cout << "File partially read: x = " << x << " , y = " << y << "\n";
	input.close();
    imshow(window_name1, image1);
    imshow(window_name2, image2);
	cout << "\nPress \"Esc\" key to continue...\n";
	while(waitKey() != 27);
	image1.~Mat();
	image2.~Mat();
	destroyWindow(window_name1);
	destroyWindow(window_name2);
	return 0;
}
