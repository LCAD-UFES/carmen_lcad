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


using namespace std;
using namespace cv;


Mat image;
Mat image2;
int roi_x0 = 0;
int roi_y0 = 0;
int roi_x1 = 0;
int roi_y1 = 0;
int startDraw = 0;
string window_name = "<SPACE>Add  <G>Green  <Y>Yellow  <R>Red  <O>Off  <N>Next  <B>Back  <ESC>Exit";


string
IntToString(int num)
{
	ostringstream myStream;
	myStream << num << flush;

	return (myStream.str());
}


void
draw_rectangle(int x0, int y0, int x1, int y1)
{
	//cout << x0 << " " << y0 << " " << x1 << " " << y1 << endl;

	image2 = image.clone();
	rectangle(image2, cvPoint(x0, y0), cvPoint(x1, y1), CV_RGB(255, 0, 255), 1);
	imshow(window_name, image2);
	image2.~Mat();
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
//		image2 = image.clone();
//		rectangle(image2, cvPoint(roi_x0, roi_y0), cvPoint(x, y), CV_RGB(255, 0, 255), 1);
//		imshow(window_name, image2);
//		image2.~Mat();

		draw_rectangle(roi_x0, roi_y0, x, y);
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


void
aply_rectangles_to_file(bool &first_rect_drawn, string &str, vector<string> &images_vector, string image_name, string file_name)
{
	if (!first_rect_drawn || (roi_x0 != 0 && roi_x1 != 0 && roi_y0 != 0 && roi_y1 != 0))
	{
		first_rect_drawn = add_new_rectangle(str, image_name);
	}
	if (!str.empty())
	{
		cout << "Saving to " << file_name << str << endl;
		images_vector.push_back(str);
		write_marked_images_to_file(images_vector, "red_gt.txt");
		str.clear();
	}
}


int
main(int argc, char** argv)
{
	int iKey = -1, i;
	string s, line, image_name, input_file, strPostfix = "";
	vector<string> strings, input_images, red_images, yellow_images, green_images, off_images;
	ifstream input;
	ofstream green, yellow, red, off;
	bool first_rect_drawn = false;

	if (argc != 2)
	{
		cerr << argv[0] << " input_ImageList.txt" << endl;
		return -1;
	}

	input_file = argv[1];

	input.open(input_file.c_str());

	namedWindow(window_name, 1);
	setMouseCallback(window_name, on_mouse, NULL);


	if (input.is_open())
	{
		getline(input, line);

		while (!input.eof())
		{
			istringstream iss(line);
			for (i = 0; getline(iss, s, ' '); i++)
			{
				//cout << "Pose: " << s << endl;
				strings.push_back(s);
			}
			image_name = strings[0];
			cout << "Loading image :" << image_name << endl;
			image = imread(image_name, 1);


			if (!image.empty())
			{
				input_images.push_back(strings[0]);

				if (i > 1) // So the image is already marked and the rectangle should be drawn
				{
					roi_x0 = atoi(strings[1].c_str());
					roi_y0 = atoi(strings[2].c_str());
					roi_x1 = atoi(strings[3].c_str());
					roi_y1 = atoi(strings[4].c_str());

					draw_rectangle(roi_x0, roi_y0, roi_x1, roi_y1);
				}
				else
				{
					imshow(window_name, image);
				}

				iKey = -1;

				while (iKey != 110) // && iKey != 78) 78 ????
				{
					//  any other key clears rectangle drawing only
					iKey = waitKey(0);
					switch (iKey)
					{

					case 32:                      // SPACE key 32 Add image name and rectangle position to string
						first_rect_drawn = add_new_rectangle(strPostfix, line);
						break;

					case 114:                      // R key 114 Save added rectangles to RED file
						if (!first_rect_drawn || (roi_x0 != 0 && roi_x1 != 0 && roi_y0 != 0 && roi_y1 != 0))
						{
							first_rect_drawn = add_new_rectangle(strPostfix, line);
						}
						if (!strPostfix.empty())
						{
							cout << "Saving to red_gt.txt  " << strPostfix << endl;
							//red << strPostfix;
							//red.flush();

							red_images.push_back(strPostfix);
							write_marked_images_to_file(red_images, "red_gt.txt");
							strPostfix.clear();
						}
						break;

					case 121:                      // Y  key 121 Save added rectangles to YELOW file
						if (!first_rect_drawn || (roi_x0 != 0 && roi_x1 != 0 && roi_y0 != 0 && roi_y1 != 0))
						{
							first_rect_drawn = add_new_rectangle(strPostfix, line);
						}
						if (!strPostfix.empty())
						{
							cout << "Saving to yellow_gt.txt  " << strPostfix << endl;
							yellow_images.push_back(strPostfix);
							write_marked_images_to_file(red_images, "yellow_gt.txt");
							strPostfix.clear();
						}
						break;

					case 103:                     // G key 103 Save added rectangles to GREEN file
						if (!first_rect_drawn || (roi_x0 != 0 && roi_x1 != 0 && roi_y0 != 0 && roi_y1 != 0))
						{
							first_rect_drawn = add_new_rectangle(strPostfix, line);
						}
						if (!strPostfix.empty())
						{
							cout << "Saving to green_gt.txt  " << strPostfix << endl;
							green_images.push_back(strPostfix);
							write_marked_images_to_file(red_images, "green_gt.txt");
							strPostfix.clear();
						}
						break;

					case 111:                       // O key 111 Save added rectangles to OFF file
						if (!first_rect_drawn || (roi_x0 != 0 && roi_x1 != 0 && roi_y0 != 0 && roi_y1 != 0))
						{
							first_rect_drawn = add_new_rectangle(strPostfix, line);
						}
						if (!strPostfix.empty())
						{
							cout << "Saving to off_gt.txt  " << strPostfix << endl;
							off_images.push_back(strPostfix);
							write_marked_images_to_file(red_images, "off_gt.txt");
							strPostfix.clear();
						}
						break;

					case 27:                      // ESC -> key 27 Exit program
						image.release();
						destroyWindow(window_name);
						cout << "PROGRAM EXITED BY PRESSING <ESC> KEY!" << "\n\n";
						return EXIT_SUCCESS;
					}
					//cout << "IKEY " << iKey << endl;
					if ((iKey == 103 || iKey == 121 || iKey == 114 || iKey == 111) && first_rect_drawn == true)
					{
						first_rect_drawn = false;
						break;
					}
				}
				image.~Mat();
			}
			else
			{
				could_not_open_error_message("IMAGE", line);
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
