/*******************GENERATE_GT***********************

 Compile:
 g++ -std=c++0x -o generate_gt generate_gt_kitti.cpp -W -Wall `pkg-config --cflags opencv` -O4 `pkg-config --libs opencv`

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
#include <iostream>
#include <dirent.h>
#include <sys/types.h>


using namespace std;
using namespace cv;


typedef struct
{
	int x0;
	int y0;
	int x1;
	int y1;
	string state;
} BBOX;


Mat image, image2;
BBOX global_bbox;
vector<BBOX> bbox_vector, previous_bbox_vector;
string window_name = "<G>Green  <Y>Yellow  <R>Red  <O>Off  <N>Next  <B>Back  <P>Reuse_Previous_Mark  <ESC>Exit";


void
drawing_current_bbox(int x0, int y0, int x1, int y1)
{
	Mat img;

	if (image2.empty())
		img = image.clone();
	else
		img = image2.clone();

	rectangle(img, cvPoint(x0, y0), cvPoint(x1, y1), CV_RGB(0, 255, 255), 1);
	imshow(window_name, img);
	img.~Mat();
}


void
drawl_all_bbox()
{
	int r = 0, g = 0, b = 0;

	if (!image2.empty())
		image2.~Mat();

	image2 = image.clone();

	for (unsigned int i = 0; i < bbox_vector.size(); i++)
	{
		if (bbox_vector[i].state.compare("RedTrafficLight") == 0)
		{
			r = 255; g = 0; b = 0;
		}
		else if (bbox_vector[i].state.compare("YellowTrafficLight") == 0)
		{
			r = 200; g = 200; b = 0;
		}
		else if (bbox_vector[i].state.compare("GreeTrafficLight") == 0)
		{
			r = 20; g = 150; b = 50;
		}
		else if (bbox_vector[i].state.compare("OffTrafficLight") == 0)
		{
			r = 200; g = 200; b = 200;
		}
		rectangle(image2, cvPoint(bbox_vector[i].x0, bbox_vector[i].y0), cvPoint(bbox_vector[i].x1, bbox_vector[i].y1), CV_RGB(r, g, b), 1);
	}
	imshow(window_name, image2);
}


void
move_bbox(int x0_displacement, int y0_displacement, int x1_displacement, int y1_displacement)
{
	global_bbox.x0 += x0_displacement;
	global_bbox.y0 += y0_displacement;
	global_bbox.x1 += x1_displacement;
	global_bbox.y1 += y1_displacement;

	drawing_current_bbox(global_bbox.x0, global_bbox.y0, global_bbox.x1, global_bbox.y1);
}


void
resize_bbox(int zoom)
{
	global_bbox.x0 -= zoom;
	global_bbox.y0 -= zoom;
	global_bbox.x1 += zoom;
	global_bbox.y1 += zoom;

	drawing_current_bbox(global_bbox.x0, global_bbox.y0, global_bbox.x1, global_bbox.y1);
}


bool
click_is_inside_bbox(int x, int y)
{
	bool control = false;
	vector<BBOX> aux;

	for (unsigned int i = 0; i < bbox_vector.size(); i++)
	{
		if ((x > bbox_vector[i].x0) && (x < bbox_vector[i].x1) && (y > bbox_vector[i].y0) && (y < bbox_vector[i].y1))
		{
			global_bbox.x0 = bbox_vector[i].x0;
			global_bbox.y0 = bbox_vector[i].y0;
			global_bbox.x1 = bbox_vector[i].x1;
			global_bbox.y1 = bbox_vector[i].y1;
			control = true;
		}
		else
		{
			aux.push_back(bbox_vector[i]);
		}
	}
	bbox_vector.clear();
	bbox_vector = aux;
	aux.clear();

	return control;
}


void
on_mouse(int event, int x, int y, int, void*)
{
	static bool startDraw = false;

	if (event == CV_EVENT_LBUTTONDOWN)
	{
		if (!startDraw)
		{
			if (click_is_inside_bbox(x, y))
			{
				drawl_all_bbox();
				drawing_current_bbox(global_bbox.x0, global_bbox.y0, global_bbox.x1, global_bbox.y1);
			}
			else
			{
				global_bbox.x0 = x;
				global_bbox.y0 = y;
				startDraw = true;
			}
		}
		else
		{
			global_bbox.x1 = x;
			global_bbox.y1 = y;
			startDraw = false;
		}
	}
	if (event == CV_EVENT_MOUSEMOVE && startDraw)
	{
		drawing_current_bbox(global_bbox.x0, global_bbox.y0, x, y);
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


vector<string>
open_file_read_all_image_names(string filename)
{
	ifstream input;
	string line;
	vector<string> image_name_vector;

	input.open(filename.c_str());

	if (input.is_open())
	{
		getline(input, line);
		while (!input.eof())
		{
			image_name_vector.push_back(line);
			getline(input, line);
		}
		input.close();

		return image_name_vector;
	}
	else
	{
		could_not_open_error_message("FILE", filename);
		exit(0);
	}
}


void
open_image_label_file(string path, string image_name)
{
	ifstream current_label_file;
	string line, aux;
	BBOX bbox;
	vector<string> line_vector;

	image_name.replace(image_name.size()-3, 3, "txt");
	path = path + "labels/" + image_name;

	current_label_file.open(path.c_str());
	if (!current_label_file.is_open())
		return;

	getline(current_label_file, line);
	while (!current_label_file.eof())
	{
		istringstream iss(line);
		for (unsigned int i = 0; getline(iss, aux, ' '); i++)
		{
			line_vector.push_back(aux);
		}

		if (line_vector[0].compare("DontCare"))      // If this is not a DontCare line
		{
			bbox.x0 = stoi(line_vector[4]);
			bbox.y0 = stoi(line_vector[5]);
			bbox.x1 = stoi(line_vector[6]);
			bbox.y1 = stoi(line_vector[7]);
			bbox.state = line_vector[0];

			bbox_vector.push_back(bbox);
		}
		line_vector.clear();
		getline(current_label_file, line);
	}
}

void
consolidate_marked_bbox(string state)
{
	int aux;

	global_bbox.state = state;

	if (global_bbox.x0 == global_bbox.x1 || global_bbox.y0 == global_bbox.y1)
				return;

	if (global_bbox.x0 > global_bbox.x1 || global_bbox.y0 > global_bbox.y1)
	{
		aux = global_bbox.x0;
		global_bbox.x0 = global_bbox.x1;
		global_bbox.x1 = aux;

		aux = global_bbox.y0;
		global_bbox.y0 = global_bbox.y1;
		global_bbox.y1 = aux;
	}

	bbox_vector.push_back(global_bbox);
	drawl_all_bbox();
}


void
set_previous_mark_to_image()
{
	bbox_vector = previous_bbox_vector;
	drawl_all_bbox();
}


void
save_to_file(string path, string image_name)
{
	ofstream current_label_file;

	image_name.replace(image_name.size()-3, 3, "txt");
	path = path + "labels/" + image_name;

	current_label_file.open(path.c_str());

	for (unsigned int i = 0; i < bbox_vector.size(); i++)
	{
		current_label_file << bbox_vector[i].state + ' ' + "0.00" + ' ' + "0" + ' ' + "0.00" + ' ' + to_string(bbox_vector[i].x0) + ".00" + ' ' + to_string(bbox_vector[i].y0) + ".00" + ' ' + to_string(bbox_vector[i].x1) +
				".00" + ' '	+ to_string(bbox_vector[i].y1) + ".00" + ' ' + "0.00" + ' ' + "0.00" + ' ' + "0.00" + ' ' + "0.00" + ' ' + "0.00" + ' ' + "0.00" + ' ' + "0" + "\n";
	}

	previous_bbox_vector = bbox_vector;
	bbox_vector.clear();
	current_label_file.close();
}


string
get_path(string image_list)
{
	string s, path;
	vector<string> str_vector;

	istringstream iss(image_list);
	for (unsigned int i = 0; getline(iss, s, '/'); i++)
	{
		str_vector.push_back(s);
	}
	str_vector.pop_back();

	for (unsigned int i = 0; i < str_vector.size(); i++)
	{
		path += str_vector[i] + '/';
	}
	str_vector.clear();
	return path;
}


void
check_buton_pressed(int iKey, vector<string> image_name_vector, unsigned int &actual_image_position, string path)
{
	static unsigned int move_edge = 0;

	switch (iKey)
	{
		case 114:                      // R key 114 Save added rectangles to RED file
			consolidate_marked_bbox("RedTrafficLight");
			break;

		case 121:                      // Y  key 121 Save added rectangles to YELOW file
			consolidate_marked_bbox("YellowTrafficLight");
			break;

		case 103:                     // G key 103 Save added rectangles to GREEN file
			consolidate_marked_bbox("GreeTrafficLight");
			break;

		case 111:                       // O key 111 Save added rectangles to OFF file
			consolidate_marked_bbox("OffTrafficLight");
			break;

		case 112:                       // <P> key 112 set the previous mark to the current image, it does not work if the image is already marked
			set_previous_mark_to_image();
			break;

		case 110:                        // N key 110 Go to next image in the list
			save_to_file(path, image_name_vector[actual_image_position]);
			actual_image_position++;
			break;

		case 98:                        // B key 98 Go back one image in the list
			save_to_file(path, image_name_vector[actual_image_position]);
			if (actual_image_position > 0)
				actual_image_position--;
			break;

		case 49:                      // 49 <1> Select left vertical bbox edge
			move_edge = 1;
			break;

		case 50:                      // 50 <2> Select top horizontal bbox edge
			move_edge = 2;
			break;

		case 51:                      // 51 <3> Select right vertical bbox edge
			move_edge = 3;
			break;

		case 52:                      // 52 <4> Select bottom horizontal bbox edge
			move_edge = 4;
			break;

		case 48:                      // 48 <0> Go back to move the hole bbox
			move_edge = 0;
			break;

		case 119:                     // 65362 <W> Up
			if (move_edge == 0)
				move_bbox(0, -1, 0, -1);
			else if (move_edge == 2)
				move_bbox(0, -1, 0, 0);
			else if (move_edge == 4)
				move_bbox(0, 0, 0, -1);
			break;

		case 115:                     // 65364 <S> Down
			if (move_edge == 0)
				move_bbox(0, 1, 0, 1);
			else if (move_edge == 2)
				move_bbox(0, 1, 0, 0);
			else if (move_edge == 4)
				move_bbox(0, 0, 0, 1);
			break;

		case 97:                     // 65361 <L> Left
			if (move_edge == 0)
				move_bbox(-1, 0, -1, 0);
			else if (move_edge == 1)
				move_bbox(-1, 0, 0, 0);
			else if (move_edge == 3)
				move_bbox(0, 0, -1, 0);
			break;

		case 100:                     // 65363 <R> Right
			if (move_edge == 0)
				move_bbox(1, 0, 1, 0);
			else if (move_edge == 1)
				move_bbox(1, 0, 0, 0);
			else if (move_edge == 3)
				move_bbox(0, 0, 1, 0);
			break;

		case 113:                     // 113 <Q> Enlarge
			resize_bbox(1);
			break;

		case 101:                     // 101 <E> Reduce
			resize_bbox(-1);
			break;

		case 27:                        // ESC -> key 27 Exit program
			cout << "\nPROGRAM EXITED BY PRESSING <ESC> KEY!" << "\n\n";
			break;
	}
}


int
main(int argc, char** argv)
{
	int iKey = -1;
	unsigned int actual_image_position = 0;
	string path;
	vector<string> image_name_vector;

	if (argc != 2)
	{
		cerr << argv[0] << " <input_ImageList.txt>" << endl;
		return -1;
	}

	path = get_path(argv[1]);
	image_name_vector = open_file_read_all_image_names (argv[1]);

	namedWindow(window_name, WINDOW_AUTOSIZE);
	setMouseCallback(window_name, on_mouse);

	while (actual_image_position < image_name_vector.size() && iKey != 27)
	{
		cout << "Loading image :" << path + "images/" + image_name_vector[actual_image_position] << endl;
		image = imread(path + "images/" + image_name_vector[actual_image_position], 1);

		if (!image.empty())
		{
			imshow(window_name, image);
			open_image_label_file(path, image_name_vector[actual_image_position]);
			drawl_all_bbox();

			iKey = -1;												// 98  <B> go back one image in the list
			while (iKey != 98 && iKey != 110 && iKey != 27) 		// 110 <N> go to next image saving nothing in no file
			{														// 27  <ESC> go to next image saving nothing in no file
				iKey = waitKey(0);
				//cout << "IKEY " << iKey << endl;

				check_buton_pressed(iKey, image_name_vector, actual_image_position, path);
			}
		}
		else
		{
			could_not_open_error_message("IMAGE", path + "images/" + image_name_vector[actual_image_position]);
		}
	}
	bbox_vector.clear();
	previous_bbox_vector.clear();
	image_name_vector.clear();
	image.~Mat();
	image2.~Mat();
	destroyWindow(window_name);

	return EXIT_SUCCESS;
}
