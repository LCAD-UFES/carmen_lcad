#include "moving_objects_simulator.h"

extern std::vector<object_model_features_t> object_models;
extern int num_of_models;

double
euclidean_distance(double x1, double y1, double x2, double y2)
{
	double dist = sqrt( (x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) );

	return dist;
}


void
clear_obj_model_features(object_model_features_t &obj_model)
{
	obj_model.model_id = -1;
	obj_model.model_name = (char *)"";
	obj_model.geometry.width = 0.0;
	obj_model.geometry.length = 0.0;
	obj_model.geometry.height = 0.0;
	obj_model.red = 0.0;
	obj_model.green = 0.0;
	obj_model.blue = 0.0;
}


void
set_model(object_model_features_t &obj_model, int model_id, char *model_type, double width, double length, double height,
		double red, double green, double blue)
{
	obj_model.model_id = model_id;
	obj_model.model_name = model_type;
	obj_model.geometry.width = width;
	obj_model.geometry.length = length;
	obj_model.geometry.height = height;
	obj_model.red = red;
	obj_model.green = green;
	obj_model.blue = blue;
}


object_model_features_t
get_obj_model_features(int model_id)
{
	object_model_features_t obj_model;

	if (model_id >= 0 && model_id < int(object_models.size()))
		obj_model = object_models[model_id];
	else
		clear_obj_model_features(obj_model);

	return obj_model;
}


void
set_object_models(std::vector<object_model_features_t> &obj_models)
{
	object_model_features_t obj_class;

	/* 0) sedan */
	set_model(obj_class, 0, (char *)"car", 1.8, 4.4, 1.4, 1.0, 0.0, 0.8);
	obj_models.push_back(obj_class);

	/* 11) bike/motorbike 1 */
	set_model(obj_class, 11, (char *)"bike", 0.7, 2.2, 1.4, 0.0, 1.0, 1.0);
	obj_models.push_back(obj_class);

	/* 21) small truck */
	set_model(obj_class, 21, (char *)"truck", 2.2, 6.8, 2.6, 0.5, 0.5, 1.0);
	obj_models.push_back(obj_class);

	/* 31) bus (average estimative) */
	set_model(obj_class, 31, (char *)"bus", 2.9, 12.6, 3.5, 1.0, 1.0, 0.0);
	obj_models.push_back(obj_class);

	/* 41) pedestrian 1 */
	set_model(obj_class, 41, (char *)"pedestrian", 0.6, 0.6, 1.7, 0.0, 1.0, 0.0);
	obj_models.push_back(obj_class);

	num_of_models = int(obj_models.size());
}
