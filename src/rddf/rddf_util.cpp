#include <vector>
#include <float.h>
#include <iostream>
#include <signal.h>
#include <algorithm>
#include <sys/signal.h>

#include <carmen/carmen.h>
#include <carmen/writelog.h>
#include <carmen/carmen_stdio.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/carmen_gps_wrapper.h>
#include <carmen/collision_detection.h>

#include "rddf_util.h"
#include "rddf_index.h"

#include <fstream>

#include <kml/engine.h>
#include <kml/dom.h>

#include "g2o/types/slam2d/se2.h"

#include <locale.h>

using namespace std;
using namespace g2o;

kmldom::KmlFactory *factory;
kmldom::KmlPtr kml;
kmldom::DocumentPtr document;
kmldom::PlacemarkPtr placemark;
kmldom::LineStringPtr path;
kmldom::CoordinatesPtr waypoints;

typedef std::vector<kmldom::PlacemarkPtr> placemark_vector_t;
kmlengine::KmlFilePtr kml_file;

bool use_road_map = false;
char *carmen_rddf_filename = NULL;
int traffic_lights_camera = 3;

double distance_between_front_and_rear_axles;
double distance_between_front_car_and_front_wheels;
double turning_radius;
int carmen_rddf_num_poses_ahead_max;
double rddf_min_distance_between_waypoints;
int carmen_rddf_perform_loop;
double default_search_radius;
int dynamic_plot_state;
int rddf_play_use_truepos;
int road_mapper_kernel_size;
double maximum_curvature = numeric_limits<double>::max();

vector<carmen_annotation_t> annotation_read_from_file;


void
carmen_rddf_play_open_kml()
{
	factory = kmldom::KmlFactory::GetFactory();

	waypoints = factory->CreateCoordinates();

	path = factory->CreateLineString();
	path->set_coordinates(waypoints);
	path->set_altitudemode(kmldom::ALTITUDEMODE_CLAMPTOGROUND);
	path->set_tessellate(true);
	path->set_extrude(true);

	placemark = factory->CreatePlacemark();
	placemark->set_geometry(path);

	kmldom::IconStylePtr iconstyle = factory->CreateIconStyle();
	iconstyle->set_scale(1.1);
	iconstyle->set_color(kmlbase::Color32("ff00ff00"));

	kmldom::StylePtr normal = factory->CreateStyle();
	normal->set_id("normal");
	normal->set_iconstyle(iconstyle);

	iconstyle = factory->CreateIconStyle();
	iconstyle->set_scale(2.3);
	iconstyle->set_color(kmlbase::Color32("ff00ff00"));

	kmldom::StylePtr highlight = factory->CreateStyle();
	highlight->set_id("highlight");
	highlight->set_iconstyle(iconstyle);

	kmldom::StyleMapPtr stylemap = factory->CreateStyleMap();
	stylemap->set_id("stylemap");

	kmldom::PairPtr pair = factory->CreatePair();
	pair->set_key(kmldom::STYLESTATE_NORMAL);
	pair->set_styleurl("#normal");
	stylemap->add_pair(pair);

	pair = factory->CreatePair();
	pair->set_key(kmldom::STYLESTATE_HIGHLIGHT);
	pair->set_styleurl("#highlight");
	stylemap->add_pair(pair);

	document = factory->CreateDocument();
	document->add_feature(placemark);
	document->add_styleselector(normal);
	document->add_styleselector(stylemap);
	document->add_styleselector(highlight);

	kml = factory->CreateKml();
	kml->set_feature(document);
}


void
carmen_rddf_play_save_waypoints(char *carmen_rddf_filename)
{
	ofstream kml_file(carmen_rddf_filename, ios::out);

	if (!kml_file.is_open())
		carmen_die("Error opening KML file.");

	std::string xml = kmldom::SerializePretty(kml);
	kml_file << xml;

	kml_file.close();
}

void
carmen_rddf_play_save_waypoints_with_txt(char *carmen_rddf_filename)
{
	ofstream kml_file(carmen_rddf_filename, ios::out);

	if (!kml_file.is_open())
		carmen_die("Error opening KML file.");

	std::string xml = kmldom::SerializePretty(kml);
	kml_file << xml;

	kml_file.close();
}


void
carmen_rddf_play_add_waypoint(double latitude, double longitude)
{
	waypoints->add_latlng(latitude, longitude);
}


void
carmen_rddf_play_add_waypoint_speed(double latitude, double longitude, double max_speed, double driver_speed, double theta, double timestamp)
{
	char buffer[64];

	sprintf(buffer, "%.0lf", max_speed);
	std::string speed_string = buffer;

	sprintf(buffer, "%lf", theta);
	std::string theta_string = buffer;

	sprintf(buffer, "%lf", timestamp);
	std::string timestamp_string = buffer;

	sprintf(buffer, "%lf", driver_speed);
	std::string driver_speed_string = buffer;

	kmldom::CoordinatesPtr coord = factory->CreateCoordinates();
	coord->add_latlng(latitude, longitude);

	kmldom::PointPtr point = factory->CreatePoint();
	point->set_coordinates(coord);

	kmldom::PlacemarkPtr speedmark = factory->CreatePlacemark();
	speedmark->set_geometry(point);
	speedmark->set_name(speed_string);
	speedmark->set_styleurl("#stylemap");

	kmldom::DataPtr theta_value = factory->CreateData();
	theta_value->set_name("theta");
	theta_value->set_value(theta_string);

	kmldom::DataPtr timestamp_value = factory->CreateData();
	timestamp_value->set_name("timestamp");
	timestamp_value->set_value(timestamp_string);

	kmldom::DataPtr driver_speed_value = factory->CreateData();
	driver_speed_value->set_name("driver_velocity");
	driver_speed_value->set_value(driver_speed_string);

	kmldom::ExtendedDataPtr annotations = factory->CreateExtendedData();
	annotations->add_data(theta_value);
	annotations->add_data(timestamp_value);
	annotations->add_data(driver_speed_value);

	speedmark->set_extendeddata(annotations);
	document->add_feature(speedmark);
}


static void
SavePlacemarks(const kmldom::FeaturePtr& feature, placemark_vector_t* placemarks)
{
	if (kmldom::PlacemarkPtr placemark = kmldom::AsPlacemark(feature))
	{
		placemarks->push_back(placemark);
	}
	else if (const kmldom::ContainerPtr container = kmldom::AsContainer(feature))
	{
		for (size_t i = 0; i < container->get_feature_array_size(); ++i)
		{
			SavePlacemarks(container->get_feature_array_at(i), placemarks);
		}
	}
}


void
carmen_rddf_play_open_kml(const char *filename, placemark_vector_t *placemark_vector)
{
	std::string kml;
	std::string errors;

	if (!kmlbase::File::ReadFileToString(filename, &kml))
		carmen_die("Error reading KML file.");

	kml_file = kmlengine::KmlFile::CreateFromParse(kml, &errors);

	if (!kml_file)
		carmen_die("Error parsing KML file.");

	SavePlacemarks(kmlengine::GetRootFeature(kml_file->get_root()), placemark_vector);
}


int
carmen_rddf_play_copy_kml(kmldom::PlacemarkPtr waypoint, carmen_fused_odometry_message *message, int *waypoint_annotation)
{
	int north = 0;
	double zone = 0, x = 0, y = 0, z = 0;
	double latitude = 0, longitude = 0;
	double theta = 0, timestamp = 0;
	double driver_speed = 0;
	std::string waypoint_description;

	kmldom::PointPtr point;
	kmldom::CoordinatesPtr coord;
	kmldom::ExtendedDataPtr annotations;

	kmldom::DataPtr data;

	if (waypoint->has_geometry())
	{
		if (waypoint->get_geometry()->IsA(kmldom::Type_Point))
		{
			point = kmldom::AsPoint(waypoint->get_geometry());
			coord = point->get_coordinates();

			latitude = coord->get_coordinates_array_at(0).get_latitude();
			longitude = coord->get_coordinates_array_at(0).get_longitude();

			carmen_Gdc3_Utm(&x, &y, &z, &zone, &north, latitude, longitude, 0.0);
		}
		else
		{
			return 0;
		}

		message->pose.position.x = y;
		message->pose.position.y = -x;
		message->pose.position.z = z;

		std::string speed_string = waypoint->get_name();

		// OBS: o set da velocidade esta mais abaixo!
		// message->velocity.x = speed;
		message->velocity.y = 0;
		message->velocity.z = 0;

		(*waypoint_annotation) = RDDF_ANNOTATION_TYPE_NONE;

		waypoint_description = waypoint->get_description();

		if (waypoint_description.compare("") != 0)
		{
			if (waypoint_description.compare("human intervention") == 0)
				(*waypoint_annotation) = RDDF_ANNOTATION_TYPE_HUMAN_INTERVENTION;
			else
				exit(printf("Error: Unknown description annotation: \"%s\"\n", waypoint_description.c_str()));
		}

		if (waypoint->has_extendeddata())
		{
			annotations = waypoint->get_extendeddata();

			for (unsigned int i = 0; i < annotations->get_data_array_size(); i++)
			{
				data = annotations->get_data_array_at(i);

				if (data->has_name())
				{
					if (data->get_value().size() > 0)
					{
						if (std::string("theta").compare(data->get_name()) == 0)
						{
							std::string data_string = data->get_value();
							theta = atof( data_string.c_str() );
						}
						else if (std::string("timestamp").compare(data->get_name()) == 0)
						{
							std::string data_string = data->get_value();
							timestamp = atof( data_string.c_str() );
						}
						else if (std::string("driver_velocity").compare(data->get_name()) == 0)
						{
							std::string data_string = data->get_value();
							driver_speed = atof(data_string.c_str());
						}
					}
				}
			}
		}

		message->velocity.x = driver_speed;
		message->pose.orientation.roll= 0;
		message->pose.orientation.pitch= 0;
		message->pose.orientation.yaw = theta;
		message->gps_position_at_turn_on.x= 0;
		message->gps_position_at_turn_on.y= 0;
		message->gps_position_at_turn_on.z= 0;
		message->angular_velocity.roll= 0;
		message->angular_velocity.pitch= 0;
		message->angular_velocity.yaw= 0;
		message->timestamp = timestamp;
		message->phi= 0;
	}

	return 1;
}


carmen_rddf_waypoint *
carmen_rddf_play_load_rddf_from_file(char *rddf_filename, int *out_waypoint_vector_size)
{
	setlocale(LC_NUMERIC, "C");
	unsigned int i, n;
	carmen_fused_odometry_message message;

	if (strcmp(rddf_filename + (strlen(rddf_filename) - 3), "kml") == 0)
	{
		int annotation = 0;
		placemark_vector_t placemark_vector;

		carmen_rddf_play_open_kml(rddf_filename, &placemark_vector);
		carmen_rddf_waypoint *waypoint_vector = (carmen_rddf_waypoint *) calloc (sizeof(carmen_rddf_waypoint), placemark_vector.size());

		n = 0;

		for (i = 0; i < placemark_vector.size(); i++)
		{
			if (carmen_rddf_play_copy_kml(placemark_vector[i], &message, &annotation))
			{
				waypoint_vector[n].timestamp = message.timestamp;
				waypoint_vector[n].pose.x = message.pose.position.x;
				waypoint_vector[n].pose.y = message.pose.position.y;
				waypoint_vector[n].pose.theta = message.pose.orientation.yaw;
				waypoint_vector[n].driver_velocity = message.velocity.x;

				// TODO: aparentemente esse valor nao esta sendo salvo no rddf. No rddf_build, ele
				// esta sendo inicializado com o robot_max_velocity do carmen.ini.
				waypoint_vector[n].max_velocity = 2.0;

				n++;
			}
		}

		*out_waypoint_vector_size = n;
		return (waypoint_vector);
	}
	else
	{
		vector<carmen_fused_odometry_message> messages;
		FILE *fptr = fopen(rddf_filename, "r");

		while (!feof(fptr))
		{
			if (!fptr)
			{
				printf("Could not open file %s\n Exiting...\n", rddf_filename);
				exit(1);
			}
			memset(&message, 0, sizeof(message));

			fscanf(fptr, "%lf %lf %lf %lf %lf %lf\n",
				&(message.pose.position.x), &(message.pose.position.y),
				&(message.pose.orientation.yaw), &(message.velocity.x), &(message.phi),
				&(message.timestamp));

			messages.push_back(message);
		}
		fclose(fptr);

		carmen_rddf_waypoint *waypoint_vector = (carmen_rddf_waypoint *) calloc (sizeof(carmen_rddf_waypoint), messages.size());
		for (i = 0; i < messages.size(); i++)
		{
			waypoint_vector[i].timestamp = messages[i].timestamp;
			waypoint_vector[i].pose.x = messages[i].pose.position.x;
			waypoint_vector[i].pose.y = messages[i].pose.position.y;
			waypoint_vector[i].pose.theta = messages[i].pose.orientation.yaw;
			waypoint_vector[i].driver_velocity = messages[i].velocity.x;
			waypoint_vector[i].max_velocity = messages[i].velocity.x;
			waypoint_vector[i].phi = messages[i].phi;
		}

		*out_waypoint_vector_size = messages.size();
		return (waypoint_vector);
	}
}


void
carmen_rddf_play_save_rddf_to_file(char *rddf_filename, carmen_rddf_waypoint *waypoint_vector, int size)
{
	int i;
	if (strcmp(rddf_filename + (strlen(rddf_filename) - 3), "kml") == 0)
	{
		double latitude, longitude, altitude;
		double x, y, theta;

		carmen_rddf_play_open_kml();

		for (i = 0; i < size; i++)
		{
			x = waypoint_vector[i].pose.x;
			y = waypoint_vector[i].pose.y;
			theta = waypoint_vector[i].pose.theta;

			carmen_Utm_Gdc3(-y, x, 0.0, 24, 0.0, &latitude, &longitude, &altitude);
			carmen_rddf_play_add_waypoint(latitude, longitude);
			carmen_rddf_play_add_waypoint_speed(latitude, longitude, waypoint_vector[i].max_velocity, waypoint_vector[i].driver_velocity, theta, (double) waypoint_vector[i].timestamp);
		}

		carmen_rddf_play_save_waypoints(rddf_filename);
	}
	else
	{
		FILE *fptr = fopen(rddf_filename, "w");
		for (i = 0; i < size; i++)
		{
			fprintf(fptr, "%lf %lf %lf %lf %lf %lf\n",
					waypoint_vector[i].pose.x, waypoint_vector[i].pose.y,
					waypoint_vector[i].pose.theta, waypoint_vector[i].driver_velocity, waypoint_vector[i].phi,
					waypoint_vector[i].timestamp);
		}
		fclose(fptr);
	}
}


//bool
//carmen_rddf_play_annotation_is_forward(carmen_ackerman_traj_point_t robot_pose, carmen_ackerman_traj_point_t annotation_point)
//{
//	SE2 robot_pose_mat(robot_pose.x, robot_pose.y, robot_pose.theta);
//	SE2 annotation_point_mat(annotation_point.x, annotation_point.y, 0.0);
//	SE2 annotation_in_car_reference = robot_pose_mat.inverse() * annotation_point_mat;
//
//	if (annotation_in_car_reference[0] > 0.0)
//		return (true);
//	else
//		return (false);
//}


bool
carmen_rddf_play_annotation_is_forward(carmen_robot_and_trailer_traj_point_t robot_pose, carmen_vector_3D_t annotation_point)
{
	SE2 robot_pose_mat(robot_pose.x, robot_pose.y, robot_pose.theta);
	SE2 annotation_point_mat(annotation_point.x, annotation_point.y, 0.0);
	SE2 annotation_in_car_reference = robot_pose_mat.inverse() * annotation_point_mat;

	if (annotation_in_car_reference[0] > 0.0)
		return (true);
	else
		return (false);
}


bool
carmen_rddf_play_annotation_is_forward(carmen_robot_and_trailer_traj_point_t robot_pose, carmen_robot_and_trailer_traj_point_t annotation_point)
{
	SE2 robot_pose_mat(robot_pose.x, robot_pose.y, robot_pose.theta);
	SE2 annotation_point_mat(annotation_point.x, annotation_point.y, 0.0);
	SE2 annotation_in_car_reference = robot_pose_mat.inverse() * annotation_point_mat;

	if (annotation_in_car_reference[0] > 0.0)
		return (true);
	else
		return (false);
}


bool
carmen_rddf_play_annotation_is_forward(carmen_point_t robot_pose, carmen_vector_3D_t annotation_point)
{
	SE2 robot_pose_mat(robot_pose.x, robot_pose.y, robot_pose.theta);
	SE2 annotation_point_mat(annotation_point.x, annotation_point.y, 0.0);
	SE2 annotation_in_car_reference = robot_pose_mat.inverse() * annotation_point_mat;

	if (annotation_in_car_reference[0] > 0.0)
		return (true);
	else
		return (false);
}


int
carmen_rddf_play_pose_out_of_map_coordinates(carmen_point_t pose, carmen_map_p map)
{
	double x_min = map->config.x_origin;
	double x_max = map->config.x_origin + map->config.x_size * map->config.resolution;
	double y_min = map->config.y_origin;
	double y_max = map->config.y_origin + map->config.y_size * map->config.resolution;
	int out_of_map = (pose.x < x_min || pose.x >= x_max || pose.y < y_min || pose.y >= y_max);

	return (out_of_map);
}


void
carmen_rddf_play_clear_annotation_vector()
{
	for (unsigned int i = 0; i < annotation_read_from_file.size(); i++)
	{
		free (annotation_read_from_file[i].annotation_description);
	}

//	printf("%s\n", annotation_read_from_file[i].annotation_description);

	annotation_read_from_file.clear();
}


void
displace_car_pose_according_to_car_orientation(carmen_annotation_t *annotation, int direction = -1)
{
	carmen_robot_and_trailer_traj_point_t annotation_point;
	annotation_point.x = annotation->annotation_point.x;
	annotation_point.y = annotation->annotation_point.y;
	annotation_point.theta = annotation->annotation_orientation;
	double distance_car_pose_car_front = distance_between_front_and_rear_axles + distance_between_front_car_and_front_wheels;
	carmen_robot_and_trailer_pose_t new_annotation_point = carmen_collision_detection_displace_car_pose_according_to_car_orientation(
			(carmen_robot_and_trailer_traj_point_t *)&annotation_point, distance_car_pose_car_front * direction);
	annotation->annotation_point.x = new_annotation_point.x;
	annotation->annotation_point.y = new_annotation_point.y;
}


int
find_annotation_by_coordinates(carmen_vector_3D_t point, double range, int current_index = -1)
{
	int i = -1;

	for (i = annotation_read_from_file.size() - 1; i >= 0; i--)
		if ((DIST2D(annotation_read_from_file[i].annotation_point, point) < range) && (i != current_index))
			break;

	return i;
}


void
updade_annotation_vector(crud_t action, int old_index, carmen_annotation_t new_annotation)
{
	switch (action)
	{
		case CREATE_ACTION:
			annotation_read_from_file.push_back(new_annotation);
			break;
		case READ_ACTION:
			break;
		case UPDATE_ACTION:
			free(annotation_read_from_file[old_index].annotation_description);
			annotation_read_from_file[old_index] = new_annotation;
			break;
		case DELETE_ACTION:
			free(annotation_read_from_file[old_index].annotation_description);
			annotation_read_from_file.erase(annotation_read_from_file.begin() + old_index);
			break;
	}
}


void
carmen_rddf_play_updade_annotation_vector(crud_t action, carmen_annotation_t old_annotation, carmen_annotation_t new_annotation)
{
	int i_old = -1, i_new = -1;
	carmen_annotation_t old_annotation_d = old_annotation;
	carmen_annotation_t new_annotation_d = new_annotation;
	displace_car_pose_according_to_car_orientation(&old_annotation_d);
	displace_car_pose_according_to_car_orientation(&new_annotation_d);

	if (action == DELETE_ACTION || action == UPDATE_ACTION)
	{
		i_old = find_annotation_by_coordinates(old_annotation_d.annotation_point, 0.01);
		if (i_old < 0 || annotation_read_from_file[i_old].annotation_type != old_annotation_d.annotation_type ||
						 annotation_read_from_file[i_old].annotation_code != old_annotation_d.annotation_code)
		{
			carmen_warn("Annotation does not exist [carmen_rddf_play_updade_annotation_vector]: (%lf, %lf)  type %d  code %d\n",
					old_annotation.annotation_point.x, old_annotation.annotation_point.y,
					old_annotation.annotation_type, old_annotation.annotation_code);
			return;
		}
	}

	if (action == CREATE_ACTION || action == UPDATE_ACTION)
	{
		if (new_annotation.annotation_point.x == 0.0 || new_annotation.annotation_point.y == 0.0)
		{
			carmen_warn("Invalid annotation coordinates [carmen_rddf_play_updade_annotation_vector]: (%lf, %lf)  type %d  code %d\n",
					0.0, 0.0, new_annotation.annotation_type, new_annotation.annotation_code);
			return;
		}

		i_new = find_annotation_by_coordinates(new_annotation_d.annotation_point, 0.50, i_old);
		if (i_new >= 0)
		{
			carmen_annotation_t existing_annotation = annotation_read_from_file[i_new];
			displace_car_pose_according_to_car_orientation(&existing_annotation, 1);
			carmen_warn("Another annotation exists [carmen_rddf_play_updade_annotation_vector]: (%lf, %lf)  type %d  code %d\n",
					existing_annotation.annotation_point.x, existing_annotation.annotation_point.y,
					existing_annotation.annotation_type, existing_annotation.annotation_code);
			return;
		}

		if (new_annotation_d.annotation_description != NULL)
		{
			char *description = (char *) malloc(strlen(new_annotation_d.annotation_description) + 1);
			strcpy(description, new_annotation_d.annotation_description);
			new_annotation_d.annotation_description = description;
		}
	}

	updade_annotation_vector(action, i_old, new_annotation_d);
}


void
carmen_rddf_play_load_index(char *rddf_filename)
{
	int annotation = 0;
	carmen_fused_odometry_message message;
	placemark_vector_t placemark_vector;

	if (!carmen_rddf_index_exists(rddf_filename))
	{
		if (strcmp(rddf_filename + (strlen(rddf_filename) - 3), "kml") == 0)
		{
			carmen_rddf_play_open_kml(rddf_filename, &placemark_vector);

			for (unsigned int i = 0; i < placemark_vector.size(); i++)
			{
				if (carmen_rddf_play_copy_kml(placemark_vector[i], &message, &annotation))
					carmen_rddf_index_add(&message, 0, 0, annotation);
			}

			carmen_rddf_index_save(rddf_filename);
		}
		else
		{
			FILE *fptr = fopen(rddf_filename, "r");

			while (!feof(fptr))
			{
				memset(&message, 0, sizeof(message));

				fscanf(fptr, "%lf %lf %lf %lf %lf %lf\n",
					&(message.pose.position.x), &(message.pose.position.y),
					&(message.pose.orientation.yaw), &(message.velocity.x), &(message.phi),
					&(message.timestamp));

				carmen_rddf_index_add(&message, 0, 0, 0);
			}

			carmen_rddf_index_save(rddf_filename);
			fclose(fptr);
		}
	}

	carmen_rddf_load_index(rddf_filename);
}


void
carmen_rddf_play_load_annotation_file(char *carmen_annotation_filename)
{
	if (carmen_annotation_filename == NULL)
		return;

	FILE *f = fopen(carmen_annotation_filename, "r");
	if (f == NULL)
		return;

	//printf("---- Annotation file: %s\n", carmen_annotation_filename);

	carmen_rddf_play_clear_annotation_vector();

	char line[1024];

	while (fgets(line, 1023, f) != NULL)
	{
		if (line[0] == '#') // comment line
			continue;

		carmen_annotation_t annotation;
		char annotation_description[1024];
		if (sscanf(line, "%s %d %d %lf %lf %lf %lf\n",
				annotation_description,
				&annotation.annotation_type,
				&annotation.annotation_code,
				&annotation.annotation_orientation,
				&annotation.annotation_point.x,
				&annotation.annotation_point.y,
				&annotation.annotation_point.z) == 7)
		{
			annotation.annotation_description = (char *) calloc (1024, sizeof(char));
			strcpy(annotation.annotation_description, annotation_description);

			//printf("%s\t%d\t%d\t%lf\t%lf\t%lf\t%lf\n", annotation.annotation_description, annotation.annotation_type, annotation.annotation_code,
			//		annotation.annotation_orientation, annotation.annotation_point.x, annotation.annotation_point.y, annotation.annotation_point.z);


			//The annotation file's points (x,y) are placed at the front of the car
			//The annotation vector's points (x,y) are placed at the car's rear axle
			//The annotation orientation is the angle of the rddf orientation in radians
			//The value of annotation point z may have different meanings for different annotation types
			//For PEDESTRIAN_TRACK type z is the search radius for pedestrians in meters
			//For TRAFFIC_SIGN type z is the curvature of the rddf in radians/meter

			displace_car_pose_according_to_car_orientation(&annotation);
			annotation_read_from_file.push_back(annotation);
		}
	}

	fclose(f);
}


void
carmen_rddf_play_get_parameters(int argc, char** argv)
{
	carmen_param_t param_list[] =
	{
		{(char *) "robot", (char *) "distance_between_front_and_rear_axles", CARMEN_PARAM_DOUBLE, &distance_between_front_and_rear_axles, 0, NULL},
		{(char *) "robot", (char *) "distance_between_front_car_and_front_wheels", CARMEN_PARAM_DOUBLE, &distance_between_front_car_and_front_wheels, 0, NULL},
		{(char *) "robot", (char *) "turning_radius", CARMEN_PARAM_DOUBLE, &turning_radius, 0, NULL},
		{(char *) "rddf", (char *) "num_poses_ahead", CARMEN_PARAM_INT, &carmen_rddf_num_poses_ahead_max, 0, NULL},
		{(char *) "rddf", (char *) "min_distance_between_waypoints", CARMEN_PARAM_DOUBLE, &rddf_min_distance_between_waypoints, 0, NULL},
		{(char *) "rddf", (char *) "loop", CARMEN_PARAM_ONOFF, &carmen_rddf_perform_loop, 0, NULL},
		{(char *) "rddf", (char *) "default_search_radius", CARMEN_PARAM_DOUBLE, &default_search_radius, 0, NULL},
		{(char *) "rddf", (char *) "dynamic_plot_state", CARMEN_PARAM_INT, &dynamic_plot_state, 1, NULL}, /* param_edit para modificar */
		{(char *) "behavior_selector", (char *) "use_truepos", CARMEN_PARAM_ONOFF, &rddf_play_use_truepos, 0, NULL},
		{(char *) "road_mapper", (char *) "kernel_size", CARMEN_PARAM_INT, &road_mapper_kernel_size, 0, NULL},
	};

	int num_items = sizeof(param_list) / sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	if (turning_radius != 0.0)
		maximum_curvature = 1.0 / turning_radius;
}


char *
carmen_rddf_play_parse_input_command_line_parameters(int argc, char **argv)
{
	char *usage[] = { (char*) ("<rddf_filename> [<annotation_filename> [<traffic_lights_camera>]]"),
					  (char*) ("-use_road_map   [<annotation_filename> [<traffic_lights_camera>]]") };
	char *carmen_annotation_filename = NULL;
	for(int i = 0; i < argc; i++)
	{
		printf("%s ", argv[i]);
	}
	printf("\n");
	if(strcmp(argv[1], "--rddf") == 0)
	{
		carmen_rddf_filename = argv[2];
		carmen_annotation_filename = argv[3];
		printf("RDDF filename: %s.\n", carmen_rddf_filename);
		printf("Annotation filename: %s.\n", carmen_annotation_filename);
	}
	else
	{
		if (argc >= 2 && strcmp(argv[1], "-h") == 0)
		{
			printf("\nUsage 1: %s %s\nUsage 2: %s %s\n\nCarmen parameters:\n", argv[0], usage[0], argv[0], usage[1]);
			carmen_rddf_play_get_parameters(argc, argv); // display help and exit
		}
		if (argc < 2 || argc > 7)
			exit(printf("Error: Usage 1: %s %s\n       Usage 2: %s %s\n", argv[0], usage[0], argv[0], usage[1]));

		if (strcmp(argv[1], "-use_road_map") == 0)
		{
			use_road_map = true;
			printf("Road map option set.\n");
		}
		else
		{
			if(strcmp(argv[1], "--rddf") == 0)
				carmen_rddf_filename = argv[2];
			else
				carmen_rddf_filename = argv[1];
			printf("RDDF filename: %s.\n", carmen_rddf_filename);
		}

		if (argc >= 3)
			carmen_annotation_filename = argv[2];

		if (carmen_annotation_filename)
			printf("Annotation filename: %s.\n", carmen_annotation_filename);

		if (argc >= 4)
			traffic_lights_camera = atoi(argv[3]);

		printf("Traffic lights camera: %d.\n", traffic_lights_camera);
	}

	return (carmen_annotation_filename);
}


void
carmen_rddf_play_clear_rddf_loop_flag()
{
	carmen_rddf_perform_loop = 0;
}


void
compute_rectilinear_route_half_segment(vector<carmen_robot_and_trailer_traj_point_t> &rectilinear_route_segment,
		double size, carmen_annotation_t annotation, double theta, double step_size, bool reverse)
{
	if (reverse)
	{
		double distance = size;
		while (distance >= 0.0)
		{
			carmen_robot_and_trailer_traj_point_t point = { };
			double theta = carmen_normalize_theta(annotation.annotation_orientation + M_PI);
			point.x = annotation.annotation_point.x + distance * cos(theta);
			point.y = annotation.annotation_point.y + distance * sin(theta);
			point.theta = annotation.annotation_orientation;
			point.v = 1.0;
			rectilinear_route_segment.push_back(point);

			distance -= step_size;
		}
	}
	else
	{
		double distance = 0.0;
		while (distance < size)
		{
			carmen_robot_and_trailer_traj_point_t point = { };
			point.x = annotation.annotation_point.x + distance * cos(theta);
			point.y = annotation.annotation_point.y + distance * sin(theta);
			point.theta = annotation.annotation_orientation;
			point.v = 1.0;
			rectilinear_route_segment.push_back(point);

			distance += step_size;
		}
	}
}


vector<carmen_robot_and_trailer_traj_point_t>
carmen_rddf_compute_rectilinear_route_segment(carmen_annotation_t annotation, double size_front, double size_back, double step_size)
{
	vector<carmen_robot_and_trailer_traj_point_t> rectilinear_route_segment;

	double theta = annotation.annotation_orientation;
	compute_rectilinear_route_half_segment(rectilinear_route_segment, size_back, annotation, theta, step_size, true);
	compute_rectilinear_route_half_segment(rectilinear_route_segment, size_front, annotation, theta, step_size, false);

	return (rectilinear_route_segment);
}


int
carmen_rddf_index_of_point_within_rectlinear_route_segment(const vector<carmen_robot_and_trailer_traj_point_t> rectilinear_route_segment,
		carmen_robot_and_trailer_traj_point_t point)
{
	int j;

	for (j = 0; j < (int) (rectilinear_route_segment.size() - 1); j++)
	{
		int point_in_trajectory_is;
		carmen_get_point_nearest_to_trajectory(&point_in_trajectory_is, rectilinear_route_segment[j], rectilinear_route_segment[j + 1],
				point, 0.001);
		if (point_in_trajectory_is == POINT_WITHIN_SEGMENT)
			break;
	}

	if (j == (int) (rectilinear_route_segment.size() - 1))
		return (-1);
	else
		return (j);
}
