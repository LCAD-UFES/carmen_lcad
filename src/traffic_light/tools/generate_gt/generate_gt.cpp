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


Mat image, image2;
int startDraw = 0, rect_x0 = 0, rect_y0 = 0, rect_x1 = 0, rect_y1 = 0, prev_rect_x0 = 0, prev_rect_y0 = 0, prev_rect_x1 = 0, prev_rect_y1 = 0;
string window_name = "<SPACE>Add  <G>Green  <Y>Yellow  <R>Red  <O>Off  <D>Discard <B>RepeatPrevious  <N>Next  <B>Back  <ESC>Exit";


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
	image2 = image.clone();
	rectangle(image2, cvPoint(x0, y0), cvPoint(x1, y1), CV_RGB(0, 255, 255), 1);
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
			rect_x0 = x;
			rect_y0 = y;
			startDraw = 1;
		}
		else
		{
			rect_x1 = x;
			rect_y1 = y;
			startDraw = 0;
		}
	}
	if (event == CV_EVENT_MOUSEMOVE && startDraw)
	{
		draw_rectangle(rect_x0, rect_y0, x, y);
	}
}


bool
add_new_rectangle(string& strPostfix, string line)
{
	if (rect_x0 < rect_x1 && rect_y0 < rect_y1)
	{

		prev_rect_x0 = rect_x0;
		prev_rect_y0 = rect_y0;
		prev_rect_x1 = rect_x1;
		prev_rect_y1 = rect_y1;

		strPostfix += line + " " + IntToString(rect_x0) + " " + IntToString(rect_y0) + " " + IntToString(rect_x1) + " " + IntToString(rect_y1) + "\n";
		rect_x0 = 0.0;
		rect_x1 = 0.0;
		rect_y0 = 0.0;
		rect_y1 = 0.0;
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
aply_rectangles_to_file(bool &first_rect_drawn, string &str, vector<string> &images_vector, string image_name, string file_name, vector<vector<string>*> &last_saved_files)//, int file_code)
{
	if (!first_rect_drawn || (rect_x0 != 0 && rect_x1 != 0 && rect_y0 != 0 && rect_y1 != 0))
	{
		first_rect_drawn = add_new_rectangle(str, image_name);
	}
	if (!str.empty())
	{
		cout << "  --  Saving to " << file_name << endl;
		images_vector.push_back(str);
		write_marked_images_to_file(images_vector, file_name);
		str.clear();
		last_saved_files.push_back(&images_vector);
	}
}


void
save_discarded(string image_name, vector<string> &discarded_images_vector, vector<vector<string>*> &last_saved_files)
{
	cout << "  --  Saving to " << "discarded_gt.txt" << endl;
	image_name += "\n";
	discarded_images_vector.push_back(image_name);
	write_marked_images_to_file(discarded_images_vector, "discarded_gt.txt");
	last_saved_files.push_back(&discarded_images_vector);
}


void
set_previous_mark_to_image()
{
	rect_x0 = prev_rect_x0;
	rect_y0 = prev_rect_y0;
	rect_x1 = prev_rect_x1;
	rect_y1 = prev_rect_y1;
	draw_rectangle(rect_x0, rect_y0, rect_x1, rect_y1);
}


int
main(int argc, char** argv)
{
	int iKey = -1;
	unsigned int i, actual_position = 0;
	string s, line, image_name, input_file, strPostfix = "";
	vector<string> strings, input_image_names, files, red_images, yellow_images, green_images, off_images, discarded_images;
	vector<vector<string>*> last_saved_files;
	ifstream input;
	bool first_rect_drawn = false , going_back = false;

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
				strings.push_back(s);
			}
			image_name = strings[0];
			cout << "Loading image :" << image_name;
			image = imread(image_name, 1);


			if (!image.empty())
			{
				if (!going_back)
					input_image_names.push_back(line);

				if (i > 1) // So the image is already marked and the rectangle should be drawn
				{
					rect_x0 = atoi(strings[1].c_str());
					rect_y0 = atoi(strings[2].c_str());
					rect_x1 = atoi(strings[3].c_str());
					rect_y1 = atoi(strings[4].c_str());

					prev_rect_x0 = rect_x0;
					prev_rect_y0 = rect_y0;
					prev_rect_x1 = rect_x1;
					prev_rect_y1 = rect_y1;

					draw_rectangle(rect_x0, rect_y0, rect_x1, rect_y1);
				}
				else
				{
					imshow(window_name, image);
				}

				iKey = -1;
				while (iKey != 110 && iKey != 98 && iKey != 100) //&&iKey!=78)? // <N> key 110 go to next image saving nothing in no file
				{																//<D> key 100 Save image to DISCARDED file
					iKey = waitKey(0);
					switch (iKey)
					{

					case 32:                      // SPACE key 32 Add image name and rectangle position to string
						first_rect_drawn = add_new_rectangle(strPostfix, image_name);
						break;

					case 114:                      // R key 114 Save added rectangles to RED file

						aply_rectangles_to_file(first_rect_drawn, strPostfix, red_images, image_name, "red_gt.txt", last_saved_files);//, 1);
						break;

					case 121:                      // Y  key 121 Save added rectangles to YELOW file
						aply_rectangles_to_file(first_rect_drawn, strPostfix, yellow_images, image_name, "yellow_gt.txt", last_saved_files);//, 2);
						break;

					case 103:                     // G key 103 Save added rectangles to GREEN file
						aply_rectangles_to_file(first_rect_drawn, strPostfix, green_images, image_name, "green_gt.txt", last_saved_files);//, 3);

						break;
					case 111:                       // O key 111 Save added rectangles to OFF file
						aply_rectangles_to_file(first_rect_drawn, strPostfix, off_images, image_name, "off_gt.txt", last_saved_files);//, 4);
						break;

					case 100:                       // <D> key 100 Save image to DISCARDED file
						save_discarded(line, discarded_images, last_saved_files);
						break;

					case 112:                       // <P> key 112 set the previous mark to the current image, it does not work if the image is already marked
						set_previous_mark_to_image();
						break;

					case 98:                        // B key 98 Go back one image in the list
						if (input_image_names.size() < 2)
							break;

						if (going_back)
						{
							if (actual_position > 0)
								actual_position--;
							else
								break;
						}
						else
						{
							going_back = true;
							actual_position = input_image_names.size() - 2;
						}
						last_saved_files.back()->pop_back();
						last_saved_files.pop_back();
						break;

					case 27:                      // ESC -> key 27 Exit program
						image.release();
						destroyWindow(window_name);
						cout << "\nPROGRAM EXITED BY PRESSING <ESC> KEY!" << "\n\n";
						return EXIT_SUCCESS;
					}
					// cout << "IKEY " << iKey << endl;
					if ((iKey == 103 || iKey == 121 || iKey == 114 || iKey == 111) && first_rect_drawn == true) //mudar isso para dentro do while
					{
						if (going_back && actual_position < (input_image_names.size() - 1))
						{
							actual_position++;
						}
						else if (going_back && actual_position >= (input_image_names.size() - 1))
						{
							going_back = false;
							write_marked_images_to_file(red_images, "red_gt.txt");
							write_marked_images_to_file(yellow_images, "yellow_gt.txt");
							write_marked_images_to_file(green_images, "green_gt.txt");
							write_marked_images_to_file(off_images, "off_gt.txt");
							write_marked_images_to_file(discarded_images, "discarded_gt.txt");
						}
						first_rect_drawn = false;
						break;
					}
				}
				if (iKey == 110)
					cout << endl;

				image.~Mat();
			}
			else
			{
				could_not_open_error_message("IMAGE", image_name);
			}
			strings.clear();
			strPostfix.empty();

			if (going_back)
				line = input_image_names[actual_position];
			else
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
	destroyWindow(window_name);

	return EXIT_SUCCESS;
}
