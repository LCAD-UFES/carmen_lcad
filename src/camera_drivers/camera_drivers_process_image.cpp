#include <iostream>
#include <vector>

#include "camera_drivers_process_image.hpp"

using namespace std;
using namespace cv;


void
load_undistort(char *camera_name, int undistort, int width, int height, int number_of_images, int &crop, vector<tCrop> &x_y_vector, Mat &MapX, Mat &MapY)
{
	tCrop crop_coordinates;
	char x_name[64], y_name[64], x_name_[64], y_name_[64];
	double _fx = .0, _fy = .0, _cu = .0, _cv = .0, _k1 = .0, _k2 = .0, _p1 = .0, _p2 = .0, _k3 = .0;
    Mat cameraMatrix, distCoeffs, newcameramtx, R1;

	carmen_param_allow_unfound_variables(1);
	carmen_param_t param_list_[] = 
	{
		{camera_name, (char*) "crop",             CARMEN_PARAM_ONOFF,  &crop,             0, NULL},
	};
	carmen_param_install_params(0, NULL, param_list_, sizeof(param_list_)/sizeof(param_list_[0]));

	if (crop)
	{
		carmen_param_allow_unfound_variables(0);
		for (int i = 1; i <= number_of_images; ++i)
		{
			sprintf(x_name,  "crop_x_%d", i);
			sprintf(y_name,  "crop_y_%d", i);
			sprintf(x_name_, "crop_width_%d", i);
			sprintf(y_name_, "crop_height_%d", i);

			carmen_param_t param_list__[] =
			{
				{camera_name, x_name,                  CARMEN_PARAM_INT, &(crop_coordinates.x), 0, NULL},
				{camera_name, y_name,                  CARMEN_PARAM_INT, &(crop_coordinates.y), 0, NULL},
				{camera_name, x_name_,                 CARMEN_PARAM_INT, &(crop_coordinates.width), 0, NULL},
				{camera_name, y_name_,                 CARMEN_PARAM_INT, &(crop_coordinates.height), 0, NULL},
			};
			carmen_param_install_params(0, NULL, param_list__, sizeof(param_list__)/sizeof(param_list__[0]));

			x_y_vector.push_back(crop_coordinates);
		}
	}

	if (undistort)
	{
		carmen_param_allow_unfound_variables(0);
		carmen_param_t param_list[] = 
		{
			{camera_name, (char*) "fx", CARMEN_PARAM_DOUBLE, &_fx, 0, NULL },
			{camera_name, (char*) "fy", CARMEN_PARAM_DOUBLE, &_fy, 0, NULL },
			{camera_name, (char*) "cu", CARMEN_PARAM_DOUBLE, &_cu, 0, NULL },
			{camera_name, (char*) "cv", CARMEN_PARAM_DOUBLE, &_cv, 0, NULL },
			{camera_name, (char*) "k1", CARMEN_PARAM_DOUBLE, &_k1, 0, NULL },
			{camera_name, (char*) "k2", CARMEN_PARAM_DOUBLE, &_k2, 0, NULL },
			{camera_name, (char*) "k3", CARMEN_PARAM_DOUBLE, &_k3, 0, NULL },
			{camera_name, (char*) "p1", CARMEN_PARAM_DOUBLE, &_p1, 0, NULL },
			{camera_name, (char*) "p2", CARMEN_PARAM_DOUBLE, &_p2, 0, NULL },
		};
		carmen_param_install_params(0, NULL, param_list, sizeof(param_list)/sizeof(param_list[0]));

		_fx = _fx*width;
		_fy = _fy*height;
		_cu = _cu*width;
		_cv = _cv*height;

		cameraMatrix = (Mat_<double>(3, 3) << _fx, .0, _cu, .0, _fy, _cv, .0, .0, 1.);
		newcameramtx = (Mat_<double>(3, 3) << _fx, .0, _cu, .0, _fy, _cv, .0, .0, 1.);
		distCoeffs = (Mat_<double>(5, 1) << _k1, _k2, _p1, _p2, _k3);
		R1 = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);

		initUndistortRectifyMap(cameraMatrix, distCoeffs, R1, newcameramtx, Size(width, height), CV_8U, MapX, MapY);
	}
}


int
initialize_camera_process(int image_index, char *camera_name, int undistort, camera_message *message,  
	int &crop, Rect &myROI, Mat &MapX, Mat &MapY)
{
	if (!camera_name)
	{
		printf("no camera informed: %s\n", camera_name);
		return 1;
	}

	int width, height;
	vector<tCrop> x_y_vector;

	width = message->images[image_index].width;
	height = message->images[image_index].height;

	load_undistort(camera_name, undistort, width, height, message->number_of_images, crop, x_y_vector, MapX, MapY);

	if(crop)
	{
		if (x_y_vector[image_index].x < 0)
			x_y_vector[image_index].x = 0;
		if (x_y_vector[image_index].y < 0)
			x_y_vector[image_index].y = 0;
		if (x_y_vector[image_index].width < 0)
			x_y_vector[image_index].width = 0;
		if (x_y_vector[image_index].height < 0)
			x_y_vector[image_index].height = 0;

		if (x_y_vector[image_index].x > width)
			x_y_vector[image_index].x = width;
		if (x_y_vector[image_index].y > height)
			x_y_vector[image_index].y = height;

		if ((x_y_vector[image_index].x + x_y_vector[image_index].width) > width)
			x_y_vector[image_index].width = width - x_y_vector[image_index].x;
		if ((x_y_vector[image_index].y + x_y_vector[image_index].height) > height)
			x_y_vector[image_index].height = height - x_y_vector[image_index].y;

		myROI = Rect(x_y_vector[image_index].x, x_y_vector[image_index].y, x_y_vector[image_index].width, x_y_vector[image_index].height);
	}

	return 0;
}



int
process_image(camera_message *message, int image_index, char*camera_name, int undistort, cv::Mat &src_image)
{
    static int crop = 0, first_time = 1, ret = 0;
    static Mat MapX, MapY;
    static Rect myROI;

	if ((message->images[image_index].height*message->images[image_index].width < 0) || (!message->images[image_index].raw_data))
		return 1;

	if (message->undistorted)
		undistort = 0;

	if ((image_index+1) > message->number_of_images)
	{
		printf("wrong image index: %d\n", image_index);
		return 1;
	}

    src_image = Mat(message->images[image_index].height, message->images[image_index].width, CV_8UC3, message->images[image_index].raw_data, 0);
	// src_image.convertTo(src_image, CV_32F, 1.0/255.0);

	if (first_time)
    {
        ret = initialize_camera_process(image_index, camera_name, undistort, message, crop, myROI, MapX, MapY);
        first_time = 0;
    }

	if (undistort && !ret)
		remap(src_image, src_image, MapX, MapY, INTER_LINEAR);

	if (crop)
	    src_image = src_image(myROI);

	return 0;
}
