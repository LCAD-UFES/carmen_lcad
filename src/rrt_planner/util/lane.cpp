/*
 * street.cpp
 *
 *  Created on: 08/11/2012
 *      Author: romulo
 */

#include <stdio.h>

#include <kml/base/file.h>
#include <kml/engine.h>
#include <kml/dom.h>
#include <kml/engine/kml_file.h>
#include <carmen/carmen_gps_wrapper.h>

#include "publisher_util.h"
#include "lane.h"


static void get_placemarks(const kmldom::FeaturePtr& feature,
		vector<kmldom::PlacemarkPtr>* placemarks)
{
	if (kmldom::PlacemarkPtr placemark = kmldom::AsPlacemark(feature))
	{
		placemarks->push_back(placemark);
	}
	else if (const kmldom::ContainerPtr container = kmldom::AsContainer(
			feature))
	{
		for (size_t i = 0; i < container->get_feature_array_size(); ++i)
		{
			get_placemarks(container->get_feature_array_at(i), placemarks);
		}
	}
}

static void placemark_to_carmen_point_vector(
		vector<kmldom::PlacemarkPtr> &placemark_vector,
		vector<carmen_point_t> &carmen_poses)
{
	kmldom::CoordinatesPtr coord;
	double z, zone;
	int north;
	carmen_point_t point;

	for (unsigned int i = 0; i < placemark_vector.size(); i++)
	{
		if (placemark_vector[i]->has_geometry()
				&& placemark_vector[i]->get_geometry()->IsA(kmldom::Type_Point))
		{
			coord =
					kmldom::AsPoint(placemark_vector[i]->get_geometry())->get_coordinates();

			carmen_Gdc3_Utm(&point.y, &point.x, &z, &zone, &north,
					coord->get_coordinates_array_at(0).get_latitude(),
					coord->get_coordinates_array_at(0).get_longitude(), 0.0);

			point.y = -point.y;

			//				printf("%f %f\n", point.x, point.y);
			//			if(point.x > 7757650 && point.x < (7757600 + 150) && point.y > -363650 && point.y < (-363650 + 150))
			carmen_poses.push_back(point);
		}
	}

	printf("size %ld\n", carmen_poses.size());
}

static void get_street_points(vector<carmen_point_t> &carmen_poses,
		vector<carmen_point_t> &street_points)
{
	carmen_point_t last_point;
	double distance;
	double theta, last_theta;
	bool theta_initialized;

	if (carmen_poses.empty())
		return;

	last_point = carmen_poses.front();
	street_points.push_back(last_point);
	theta_initialized = false;

	for (unsigned int i = 0; i < carmen_poses.size(); i++)
	{
		distance = carmen_distance(&last_point, &carmen_poses[i]);

		if (distance > 1.0)
		{
			if (!theta_initialized)
			{
				last_theta = carmen_normalize_theta(
						atan2(carmen_poses[i].y - last_point.y,
								carmen_poses[i].x - last_point.x));
				theta_initialized = true;
				continue;
			}

			theta = carmen_normalize_theta(
					atan2(carmen_poses[i].y - last_point.y,
							carmen_poses[i].x - last_point.x));

			if (fabs(carmen_normalize_theta(last_theta - theta))
					> carmen_degrees_to_radians(0.25) || distance > 30.0)
			{
				last_point = carmen_poses[i];
				street_points.push_back(last_point);
				theta_initialized = false;
			}

		}

	}

}

static kmlengine::KmlFilePtr open_kml(const char *file_name)
{
	kmlengine::KmlFilePtr kml_file;
	string kml, errors;

	if (!kmlbase::File::ReadFileToString(file_name, &kml))
		carmen_die("Error reading KML file.");

	kml_file = kmlengine::KmlFile::CreateFromParse(kml, &errors);

	if (!kml_file)
		carmen_die("Error parsing KML file.");

	return kml_file;
}

//static void publish_points(vector<carmen_point_t> points)
//{
//	carmen_navigator_ackerman_plan_message msg;
//
//	msg.path_length = points.size();
//	msg.path = (carmen_ackerman_traj_point_t*) malloc(
//			sizeof(carmen_ackerman_traj_point_t) * msg.path_length);
//
//	for (int i = 0; i < msg.path_length; i++)
//	{
//		msg.path[i].x = points[i].x;
//		msg.path[i].y = points[i].y;
//	}
//
//	Publisher_Util::publish_navigator_ackerman_plan_message(msg);
//
//	free(msg.path);
//}

vector<carmen_point_t> Lane::get_street(const char *kml_path)
{
	kmlengine::KmlFilePtr kml_file;
	vector<kmldom::PlacemarkPtr> placemark_vector;
	vector<carmen_point_t> carmen_pose_vector, street_points;

	kml_file =	open_kml(kml_path);

	get_placemarks(kmlengine::GetRootFeature(kml_file->get_root()),
			&placemark_vector);

	placemark_to_carmen_point_vector(placemark_vector, carmen_pose_vector);

	get_street_points(carmen_pose_vector, street_points);

	carmen_pose_vector.clear();
	return street_points;

}

//int main2(int argc, char **argv)
//{
//	kmlengine::KmlFilePtr kml_file;
//
//	vector<kmldom::PlacemarkPtr> placemark_vector;
//	vector<carmen_point_t> carmen_pose_vector, street_points;
//
//	carmen_ipc_initialize(argc, argv);
//	carmen_param_check_version(argv[0]);
//
//	kml_file =	open_kml("/home/romulo/carmen/data/rndf/rndf_log_voltadaufes-20121003-01.kml");
//
//	get_placemarks(kmlengine::GetRootFeature(kml_file->get_root()),
//			&placemark_vector);
//
//	placemark_to_carmen_point_vector(placemark_vector, carmen_pose_vector);
//
//	get_street_points(carmen_pose_vector, street_points);
//
//	publish_points(street_points);
//	//publish_points(carmen_pose_vector);
//
//	printf("publicado\n");
//
//	carmen_ipc_dispatch();
//
//	return 0;
//}

