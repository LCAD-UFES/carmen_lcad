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

#include "rddf_util.h"

#include <fstream>
using namespace std;

#include <kml/engine.h>
#include <kml/dom.h>

#include "g2o/types/slam2d/se2.h"

#include <locale.h>

using namespace g2o;

kmldom::KmlFactory *factory;
kmldom::KmlPtr kml;
kmldom::DocumentPtr document;
kmldom::PlacemarkPtr placemark;
kmldom::LineStringPtr path;
kmldom::CoordinatesPtr waypoints;

typedef std::vector<kmldom::PlacemarkPtr> placemark_vector_t;
kmlengine::KmlFilePtr kml_file;


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


bool
carmen_rddf_play_annotation_is_forward(carmen_ackerman_traj_point_t robot_pose, carmen_ackerman_traj_point_t annotation_point)
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
carmen_rddf_play_annotation_is_forward(carmen_ackerman_traj_point_t robot_pose, carmen_vector_3D_t annotation_point)
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
