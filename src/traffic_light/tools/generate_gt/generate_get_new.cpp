/*************************************************

Execution:
  ./view_gt input_ImageList.txt"

 *************************************************/

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/legacy/legacy.hpp>

#include <stdio.h>
#include <sys/io.h>
#include <string>
#include <fstream>
#include <sstream>
#include <dirent.h>
#include <sys/types.h>

using namespace std;
using namespace cv;

Mat image;
Mat image2;
int roi_x0 = 0;
int roi_y0 = 0;
int roi_x1 = 0;
int roi_y1 = 0;
int startDraw = 0;
string window_name = "<SPACE>add, <G>green, <Y>Yellow, <R>Red, <O>Off, <N>next <ESC>exit";

string
IntToString(int num)
{
	ostringstream myStream; //creates an ostringstream object
	myStream << num << flush;
	/*
	 * outputs the number into the string stream and then flushes
	 * the buffer (makes sure the output is put into the stream)
	 */
	return (myStream.str()); //returns the string form of the stringstream object
}

void
on_mouse(int event, int x, int y, int flag, void *param)
{
	flag = flag;
	param = param;
	if (event == CV_EVENT_LBUTTONDOWN)
	{
		if (!startDraw)
		{
			roi_x0 = x;
			roi_y0 = y;
			startDraw = 1;
		}
		else
		{
			roi_x1 = x;
			roi_y1 = y;
			startDraw = 0;
		}
	}
	if (event == CV_EVENT_MOUSEMOVE && startDraw)
	{
		//redraw ROI selection
		image2 = image.clone();
		rectangle(image2, cvPoint(roi_x0, roi_y0), cvPoint(x, y), CV_RGB(255, 0, 255), 1);
		imshow(window_name, image2);
		image2.~Mat();
	}

}

bool
add_new_rectangle(string& strPostfix, string line)
{
	if (roi_x0 < roi_x1 && roi_y0 < roi_y1)
	{
		strPostfix += line + " " + IntToString(roi_x0) + " " + IntToString(roi_y0) + " " + IntToString(roi_x1) + " " + IntToString(roi_y1) + "\n";
		roi_x0 = 0.0;
		roi_x1 = 0.0;
		roi_y0 = 0.0;
		roi_y1 = 0.0;
	}
	else
	{
		cout << " There is no rectangle drawn!" << endl;
		return false;
	}
	return true;
}

int
main(int argc, char** argv)
{
	int iKey = 0;
	string strPostfix = "";
	string input_file;
	ifstream input;
	ofstream green, yellow, red, off;
	bool add_on = false;

	string s, line;
	vector<string> strings;

	if (argc != 2)
	{
		cerr << argv[0] << " input_ImageList.txt" << endl;
		return -1;
	}

	input_file = argv[1];

	cerr << "Opening input_file with names of images" << endl;
	input.open(input_file.c_str());
	cerr << "Done." << endl;

	namedWindow(window_name, 1);
	setMouseCallback(window_name, on_mouse, NULL);

	green.open("green_gt.txt", ofstream::out | ofstream::app);
	yellow.open("yellow_gt.txt", ofstream::out | ofstream::app);
	red.open("red_gt.txt", ofstream::out | ofstream::app);
	off.open("off_gt.txt", ofstream::out | ofstream::app);

	if (green.is_open() && yellow.is_open() && red.is_open() && off.is_open() && input.is_open())
	{
		getline(input, line);

		istringstream iss(line);
		while (getline(iss, s, ' '))
		{
			cout << "POS: " << s << endl;
			strings.push_back(s);
		}

		cout << "Loading image :" << strings.at(0) << endl;

		while (!input.eof())
		{
			image = imread(strings.at(0), 1);

			if (!image.empty())
			{
				iKey = -1;
				while ((iKey != 110 && iKey != 78))
				{
					cout << "x1 " << strings.at(1) << endl;
					int x1 = atoi(strings.at(1).c_str());
					cout << "y1 " << strings.at(2) << endl;
					int y1 = atoi(strings.at(2).c_str());
					cout << "x2 " << strings.at(3) << endl;
					int x2 = atoi(strings.at(3).c_str());
					cout << "y2 " << strings.at(4) << endl;
					int y2 = atoi(strings.at(4).c_str());

					Rect myROI(cvPoint(x1, y1), cvPoint(x2, y2));
					cv::Mat croppedImage;

					cv::Mat(image, myROI).copyTo(croppedImage);
					//num++;
					ostringstream myStream;
					//myStream << num << flush;
					imwrite("positives/image" + myStream.str() + ".png", croppedImage);

					rectangle(image, cvPoint(x1, y1), cvPoint(x2, y2), CV_RGB(255, 0, 255), 1);

					imshow(window_name, image);

					//  any other key clears rectangle drawing only
					iKey = waitKey(0);

					switch (iKey)
					{

					case 27:                      // ESC -> key 27 Exit program
						image.release();
						destroyWindow(window_name);
						return EXIT_SUCCESS;

					case 32:                      // SPACE key 32 Add image name and rectangle position to string
						add_on = add_new_rectangle(strPostfix, line);
						break;

					case 103:                     // G key 103 Save added rectangles to GREEN file
						if (!add_on)
						{
							add_on = add_new_rectangle(strPostfix, line);
						}
						if (!strPostfix.empty())
						{
							cout << "Saving to green_gt.txt  " << strPostfix << endl;
							green << strPostfix;
							green.flush();
							strPostfix.clear();
						}
						break;

					case 121:                      // Y  key 121 Save added rectangles to YELOW file
						if (!add_on)
						{
							add_on = add_new_rectangle(strPostfix, line);
						}
						if (!strPostfix.empty())
						{
							cout << "Saving to yellow_gt.txt  " << strPostfix << endl;
							yellow << strPostfix;
							yellow.flush();
							strPostfix.clear();
						}
						break;

					case 114:                      // R key 114 Save added rectangles to RED file
						if (!add_on)
						{
							add_on = add_new_rectangle(strPostfix, line);
						}
						if (!strPostfix.empty())
						{
							cout << "Saving to red_gt.txt  " << strPostfix << endl;
							red << strPostfix;
							red.flush();
							strPostfix.clear();
						}
						break;

					case 111:                       // O key 111 Save added rectangles to OFF file
						if (!add_on)
						{
							add_on = add_new_rectangle(strPostfix, line);
						}
						if (!strPostfix.empty())
						{
							cout << "Saving to off_gt.txt  " << strPostfix << endl;
							off << strPostfix;
							off.flush();
							strPostfix.clear();
						}
						break;
					}
					//Go to NEXT image without annotation
					if ((iKey == 103 || iKey == 121 || iKey == 114 || iKey == 111) )
					{
						add_on = false;
						break;
					}
				}
				image.~Mat();
			}
			else
			{
				cerr <<
				"------------------------------------------------------------------------------------------" << endl <<
				"Failed! COLD NOT OPEN IMAGE: " << line << endl <<
				"------------------------------------------------------------------------------------------" << "\n\n\n";
			}
			getline(input, line);
		}
	}
	else
	{
		cerr << "Failed to open: " << input_file << endl;
	}

	image.~Mat();
	image2.~Mat();
	input.close();
	green.close();
	red.close();
	yellow.close();
	off.close();
	destroyWindow(window_name);

	return EXIT_SUCCESS;
}
