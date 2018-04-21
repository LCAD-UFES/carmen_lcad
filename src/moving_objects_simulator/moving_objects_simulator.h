#ifndef MOVING_OBJECTS_SIMULATOR_H
#define MOVING_OBJECTS_SIMULATOR_H

#include <carmen/carmen.h>
#include <carmen/global.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/localize_ackerman_messages.h>
#include <carmen/moving_objects_messages.h>
#include <carmen/moving_objects_interface.h>
#include <carmen/map_server_interface.h>
#include <carmen/global_graphics.h>

#include <vector>
#include <map>


typedef struct {
	double x_global_pos;
	double y_global_pos;
	double timestamp;
	int id;
	char tipo[10];
	int oclusion;
	double alpha;
	double height;
	double width;
	double length;
	double pos_x_obj;
	double pos_y_obj;
	double l10;
	double orientation_obj;
	int l12;
	double pos_x_iara;
	double pos_y_iara;
	double l15;
	double orientation_iara;
	double velocity_obj;
} moving_object_data;

typedef struct {
	std::vector<moving_object_data> objects;
	double x_car;
	double y_car;
	double timestamp;
} timestamp_moving_objects;

typedef struct {
	int index;
	int publishing;
	std::vector<moving_object_data> objects;
} moving_objects_by_id_t;

double
euclidean_distance(double x1, double y1, double x2, double y2);

void
clear_obj_model_features(object_model_features_t &obj_model);

void
set_model(object_model_features_t &obj_model, int model_id, char *model_type, double width, double length, double height,
		double red, double green, double blue);

object_model_features_t
get_obj_model_features(int model_id);

void
set_object_models(std::vector<object_model_features_t> &obj_models);

void
publish_moving_objects_by_timestamp();

#endif
