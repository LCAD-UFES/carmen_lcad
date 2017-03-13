
#include <vector>
#include <cstdio>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;


char *output_filename;
vector<string> image_files;
vector<Rect> boxes;
int current_id = 0;
bool mouse_is_pressed = false;
Rect drawed_box;
Mat current_image;
int initial_x, initial_y;


char
draw_box(Mat m, Rect box, Scalar color, Rect box2 = Rect(-1,-1,-1,-1), Scalar color2 = Scalar(0,0,0))
{
	Mat n = m.clone();

	if (box.x >= 0 && box.y >= 0)
		rectangle(n, box, color, 2);

	if (box2.x >= 0 && box2.y >= 0)
		rectangle(n, box2, color2, 2);

	imshow("img", n);
	char c = waitKey(0);

	return c;
}


void
read_image_list(char *filename)
{
	char image_filename[1024];
	FILE *f = fopen(filename, "r");

	if (f == NULL)
		exit(printf("File '%s' not found!\n", filename));

	while (!feof(f))
	{
		Rect b;
		int n = fscanf(f, "\n%s %d %d %d %d", image_filename, &b.x, &b.y, &b.width, &b.height);

		if (n != 5)
			break;

		image_files.push_back(string(image_filename));
		boxes.push_back(b);
	}

	fclose(f);
}


void
save_images(char *filename)
{
	FILE *f = fopen(filename, "w");

	for (unsigned int i = 0; i < image_files.size(); i++)
	{
		fprintf(f, "%s %d %d %d %d\n", image_files[i].c_str(),
			boxes[i].x, boxes[i].y, boxes[i].width, boxes[i].height);
	}

	fclose(f);
}


void
mouse_callback(int event, int x, int y, int flag, void *param)
{
	(void)flag;
	(void)param;
	if (x < 0) x = 0;
	if (x >= current_image.cols) x = current_image.cols - 1;
	if (y < 0) y = 0;
	if (y >= current_image.rows) y = current_image.rows - 1;

	if (event == EVENT_LBUTTONDOWN)
	{
		drawed_box.x = x;
		drawed_box.y = y;
		drawed_box.height = 1;
		drawed_box.width = 1;

		initial_x = x;
		initial_y = y;

		mouse_is_pressed = true;
	}
	else if (event == CV_EVENT_MOUSEMOVE && mouse_is_pressed)
	{
		if (x >= initial_x)
			drawed_box.width = x - initial_x;
		else
		{
			drawed_box.x = x;
			drawed_box.width = initial_x - x;
		}

		if (y >= initial_y)
			drawed_box.height = y - initial_y;
		else
		{
			drawed_box.y = y;
			drawed_box.height = initial_y - y;
		}

		draw_box(current_image, boxes[current_id], Scalar(0, 0, 255), drawed_box, Scalar(0, 255, 0));
	}
	else if (event == EVENT_LBUTTONUP)
	{
		boxes[current_id] = drawed_box;

		drawed_box.x = -1;
		drawed_box.y = -1;

		initial_x = -1;
		initial_y = -1;

		save_images(output_filename);
		mouse_is_pressed = false;
		draw_box(current_image, boxes[current_id], Scalar(0, 0, 255));
	}
}


int
main(int argc, char **argv)
{
	if (argc < 4)
	{
		printf("Use %s <input-file> <output-file> <index>\n", argv[0]);
		exit(-1);
	}

	current_id = 0;
	read_image_list(argv[1]);
	current_id = atoi(argv[3]);

	namedWindow("img");
	setMouseCallback("img", mouse_callback, NULL);

	drawed_box.x = -1;
	drawed_box.y = -1;

	initial_x = -1;
	initial_y = -1;

	output_filename = argv[2];

	int prev = 0;

	while (1)
	{
		printf("Current image: %d\n", current_id);
		current_image = imread(image_files[current_id]);
		char c = draw_box(current_image, boxes[current_id], Scalar(0, 0, 255));

		switch (c)
		{
		case 'n':
			current_id++;
			if (current_id >= (int)image_files.size())
				current_id = 0;
			break;

		case 'p':
			current_id--;
			if (current_id < 0)
				current_id = image_files.size() - 1;
			break;

		case 'c':
			prev = current_id - 1;
			if (prev < 0) prev = image_files.size() - 1;
				boxes[current_id] = boxes[prev];
			break;

		case 'w':
			boxes[current_id].y -= 1;
			break;

		case 's':
			boxes[current_id].y += 1;
			break;

		case 'a':
			boxes[current_id].x -= 1;
			break;

		case 'd':
			boxes[current_id].x += 1;
			break;

		case (char)82: //up
			boxes[current_id].height += 1;
			boxes[current_id].y -= 1;
			break;

		case (char)81: //left
			boxes[current_id].width -= 1;
			break;

		case (char)84: //down
			boxes[current_id].height -= 1;
			boxes[current_id].y += 1;
			break;

		case (char)83: //right
			boxes[current_id].width += 1;
			break;
		}

		save_images(output_filename);
	}

	return 0;
}
