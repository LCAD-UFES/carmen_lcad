#ifndef __MOVABLE_OBJECT_H__
#define __MOVABLE_OBJECT_H__

#include <carmen/carmen.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include <vector>
#include <string>
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <numeric>

using namespace std;
using namespace cv;

//Movable object related funcitons and structs --------
#define P_BUFF_SIZE 10

struct movable_object
{
	unsigned int obj_id; 
	int track_id;
	double velocity;
	double orientation;
	unsigned int x, y, w, h;
	double last_timestamp;
	bool active;
	double timestamp[P_BUFF_SIZE];
	double x_world[P_BUFF_SIZE];
	double y_world[P_BUFF_SIZE];
	unsigned int circular_idx;// should be changed only for update_world_position function
	double estimated_range;
	double lidar_range;
	double depth_range;
};

double 
slope_angle(const vector<double>& x, const vector<double>& y);

void 
update_world_position(movable_object* p, double new_x, double new_y, double new_timestamp);

void
update_movable_object_bbox(movable_object* p,short* bbox_vector);

double
get_movable_object_x(movable_object p);

double
get_movable_object_y(movable_object p);

movable_object 
create_movable_object(int track_id);

void
clean_movable_objects(double max_time);


#endif