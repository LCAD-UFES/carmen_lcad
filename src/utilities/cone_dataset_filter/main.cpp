/*******************GENERATE_GT***********************

 Compile:
 g++ -std=c++0x -o main main.cpp -W -Wall `pkg-config --cflags opencv` -O4 `pkg-config --libs opencv` -Wno-unused-variable

 *************************************************/

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/imgproc/imgproc_c.h"

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

double RESIZE = 0.25;
string window_name = "<O>Consolidade Person Anotation  <N>Next  <B>Back  <P>Reuse Previous Mark  <ESC>Exit";

void
could_not_open_error_message(string type, string name)
{
	cerr << "\n" <<
	"------------------------------------------------------------------------------------------" << endl <<
	"Failed! COLD NOT OPEN " << type << ": " << name << endl <<
	"------------------------------------------------------------------------------------------" << "\n\n";
}

void
draw_all_bbox()
{
	int r = 0, g = 0, b = 0;

	if (!image2.empty())
		image2.~Mat();

	image2 = image.clone();

	for (unsigned int i = 0; i < bbox_vector.size(); i++)
	{
		if (bbox_vector[i].state.compare("traffic_cone") == 0)
		{
			r = 0; g = 0; b = 220;
		}
		else
		{
			r = 200; g = 200; b = 200;
		}
		rectangle(image2, cvPoint(bbox_vector[i].x0, bbox_vector[i].y0), cvPoint(bbox_vector[i].x1, bbox_vector[i].y1), CV_RGB(r, g, b), 1);

		printf("\nimagem: %d\nx0=%d\ny0=%d\nx1=%d\ny1=%d\n", i, bbox_vector[i].x0, bbox_vector[i].y0, bbox_vector[i].x1, bbox_vector[i].y1);
	}
	imshow(window_name, image2);
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

string
get_image_name(string image_path)
{
	string s;

	istringstream iss(image_path);
	for (unsigned int i = 0; getline(iss, s, '/'); )
	{
		i++;
	}

	return s;
}

int
open_image_label_file(string image_name)
{
	int label_found = 0; // FALSE
	ifstream current_label_file;
	string line, aux;
	BBOX bbox;
	vector<string> line_vector;

	image_name = get_image_name(image_name);
	image_name.replace(image_name.size()-3, 3, "txt");
	image_name = "labels/" + image_name;
	cout << "Loading label: " << image_name << endl << endl;

	current_label_file.open(image_name.c_str());

	if (!current_label_file.is_open())
		return label_found;

	getline(current_label_file, line);
	while (!current_label_file.eof())
	{
		istringstream iss(line);
		for (unsigned int i = 0; getline(iss, aux, ' '); i++)
			line_vector.push_back(aux);

		#ifdef SPECIFIC_LABEL
		if (line_vector[0].compare(SPECIFIC_LABEL) == 0)   // Compare returns the number off different characteres
			label_found = 1;
		#endif

		if (line_vector[0].compare("DontCare"))      // If this is not a DontCare line
		{
			bbox.x0 = stoi(line_vector[4]) * RESIZE;
			bbox.y0 = stoi(line_vector[5]) * RESIZE;
			bbox.x1 = stoi(line_vector[6]) * RESIZE;
			bbox.y1 = stoi(line_vector[7]) * RESIZE;
			bbox.state = line_vector[0];
			bbox_vector.push_back(bbox);
		}
		line_vector.clear();
		getline(current_label_file, line);
	}

	return label_found;
}

void
save_to_file(string image_name)
{
	ofstream current_label_file;

	image_name = get_image_name(image_name);

	image_name.replace(image_name.size()-3, 3, "txt");
	image_name = "labels/" + image_name;

	current_label_file.open(image_name.c_str());

	cout << "Saving label file: " << image_name << endl;

	for (unsigned int i = 0; i < bbox_vector.size(); i++)
	{
		current_label_file << bbox_vector[i].state + ' ' + "0.00" + ' ' + "0" + ' ' + "0.00" + ' ' + to_string((int)(bbox_vector[i].x0/RESIZE)) + ".00" + ' ' + to_string((int)(bbox_vector[i].y0/RESIZE)) + ".00" + ' ' + to_string((int)(bbox_vector[i].x1/RESIZE)) +
				".00" + ' '	+ to_string((int)(bbox_vector[i].y1/RESIZE)) + ".00" + ' ' + "0.00" + ' ' + "0.00" + ' ' + "0.00" + ' ' + "0.00" + ' ' + "0.00" + ' ' + "0.00" + ' ' + "0" + "\n";
	}

	previous_bbox_vector = bbox_vector;
	bbox_vector.clear();
	current_label_file.close();
}

void
check_buton_pressed(int iKey, vector<string> image_name_vector, unsigned int &actual_image_position)
{
	static unsigned int move_edge = 0;

	switch (iKey)
	{
        case 110: // N key 110 Go to next image in the list
			save_to_file(image_name_vector[actual_image_position]);
			actual_image_position++;
			break;

		case 98: // B key 98 Go back one image in the list
			save_to_file(image_name_vector[actual_image_position]);
			if (actual_image_position > 0)
				actual_image_position--;
			break;

		case 27: // ESC -> key 27 Exit program
			cout << "\nPROGRAM EXITED BY PRESSING <ESC> KEY!" << "\n\n";
			break;
	}
}

int
main(int argc, char** argv)
{
	int iKey = -1;
	unsigned int actual_image_position = 0;
	vector<string> image_name_vector;

	if (argc != 2)
	{
		cerr << argv[0] << " <input_ImageList.txt>" << endl;
		return -1;
	}

    image_name_vector = open_file_read_all_image_names(argv[1]);

	namedWindow(window_name, WINDOW_AUTOSIZE);

    while (actual_image_position < image_name_vector.size() && iKey != 27)
    {
        cout << "Loading image: " << image_name_vector[actual_image_position] << endl;
		image = imread(image_name_vector[actual_image_position], 1);

        if (!image.empty())
		{
			resize(image, image, Size(image.cols * RESIZE, image.rows * RESIZE));

			imshow(window_name, image);
            int label_found = open_image_label_file(image_name_vector[actual_image_position]);

            draw_all_bbox();

            iKey = -1;												// 98 <B> go back one image in the list
			while (iKey != 98 && iKey != 110 && iKey != 27) 		// 110 <N> go to next image saving nothing in no file
			{														// 27 <ESC> go to next image saving nothing in no file
				iKey = waitKey(0) & 0xff;
				cout << "IKEY " << iKey << endl;
				check_buton_pressed(iKey, image_name_vector, actual_image_position);
			}
        }
        else
		{
			could_not_open_error_message("IMAGE", image_name_vector[actual_image_position]);
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
