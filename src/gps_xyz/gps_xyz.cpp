 /*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation;
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <math.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <stdio.h>

#include <carmen/carmen.h>
#include <carmen/carmen_gps.h>
#include <carmen/gps_nmea_interface.h>
#include <carmen/gps_xyz_messages.h>
#include <carmen/gps_xyz_interface.h>
#include <carmen/xsens_mtig_interface.h>

#include <vector>
#include <algorithm>

#define GPS_MESSAGE_QUEUE_SIZE 5
#define GPS_1 1
#define GPS_2 2
#define SMALL_DELTA_T 0.02
#define REFERENCE_ANGLE 0.0

using namespace std;

typedef struct
{
	double x, y, theta, timestamp;
} graphslam_pose_t;

vector<graphslam_pose_t> graphslam_poses_opt;

vector<carmen_gps_xyz_message> gps_xyz_message_queue;

carmen_pose_3D_t sensor_board_1_pose;
carmen_pose_3D_t gps_pose_in_the_car;

carmen_base_ackerman_odometry_message base_ackerman_odometry_vector[BASE_ACKERMAN_ODOMETRY_VECTOR_SIZE];
int base_ackerman_odometry_index = -1;


double
get_angle_between_gpss(carmen_gps_xyz_message reach2, carmen_gps_xyz_message reach1)
{
	double angle = atan2(reach1.y - reach2.y, reach1.x - reach2.x) + REFERENCE_ANGLE;
	angle = carmen_normalize_theta(angle);

	return (angle);
}


bool
get_carmen_gps_gphdt_message(vector<carmen_gps_xyz_message> gps_xyz_message_queue, carmen_gps_gphdt_message *carmen_extern_gphdt_ptr)
{
	double angle = 1000.0;
	int i;

	if (gps_xyz_message_queue.size() > 0)
	{
		for (i = gps_xyz_message_queue.size() - 1; i >= 0; i--)
		{
			if (gps_xyz_message_queue[i].nr == GPS_1)
			{
				for (int j = gps_xyz_message_queue.size() - 1; j >= 0; j--)
				{
					if ((gps_xyz_message_queue[j].nr == GPS_2) && (fabs(gps_xyz_message_queue[j].utc - gps_xyz_message_queue[i].utc) < SMALL_DELTA_T))
					{
						angle = get_angle_between_gpss(gps_xyz_message_queue[j], gps_xyz_message_queue[i]);
						break;
					}
				}
				if (angle != 1000.0)
					break;
			}
		}
	}

	if (angle != 1000.0)
	{
		carmen_extern_gphdt_ptr->heading = angle;
		carmen_extern_gphdt_ptr->host = gps_xyz_message_queue[i].host;
		carmen_extern_gphdt_ptr->nr = gps_xyz_message_queue[i].nr;
		static double previous_timestamp = 0.0;
		if (gps_xyz_message_queue[i].timestamp != previous_timestamp)
			previous_timestamp = carmen_extern_gphdt_ptr->timestamp = gps_xyz_message_queue[i].timestamp;
		else
			return (false);
		carmen_extern_gphdt_ptr->valid = 1;

		return (true);
	}
	else
		return (false);
}


static int
gps_xyz_get_base_ackerman_odometry_index_by_timestamp(double timestamp)
{
	double min_diff, diff;
	int min_index;

	min_diff = DBL_MAX;
	min_index = -1;

	for (int i = 0; i < BASE_ACKERMAN_ODOMETRY_VECTOR_SIZE; i++)
	{
		diff = fabs(base_ackerman_odometry_vector[i].timestamp - timestamp);

		if (diff < min_diff)
		{
			min_diff = diff;
			min_index = i;
		}
	}

	return (min_index);
}
//////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Publishers                                                                                   //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


void
carmen_gps_xyz_publish_message(carmen_gps_xyz_message gps_xyz_message)
{
	IPC_RETURN_TYPE err = IPC_OK;

	err = IPC_publishData(CARMEN_GPS_XYZ_MESSAGE_NAME, &gps_xyz_message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_GPS_XYZ_MESSAGE_NAME);
}


static void
publish_xsens_xyz_message(carmen_xsens_xyz_message xsens_xyz_message)
{
	IPC_RETURN_TYPE err = IPC_publishData(CARMEN_XSENS_XYZ_MESSAGE_NAME, &xsens_xyz_message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_XSENS_XYZ_MESSAGE_NAME);
}


static void
publish_velodyne_gps_xyz_message(carmen_velodyne_gps_xyz_message velodyne_gps_xyz_message)
{
	IPC_RETURN_TYPE err = IPC_publishData(CARMEN_VELODYNE_GPS_XYZ_MESSAGE_NAME, &velodyne_gps_xyz_message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_VELODYNE_GPS_XYZ_MESSAGE_NAME);
}


static void
publish_carmen_gps_gphdt_message(carmen_gps_gphdt_message *carmen_extern_gphdt_ptr)
{
	IPC_RETURN_TYPE err = IPC_OK;

	if (carmen_extern_gphdt_ptr != NULL)
	{
		err = IPC_publishData (CARMEN_GPS_GPHDT_MESSAGE_NAME, carmen_extern_gphdt_ptr);
		carmen_test_ipc(err, "Could not publish", CARMEN_GPS_GPHDT_MESSAGE_NAME);
	}
}
//////////////////////////////////////////////////////////////////////////////////////////////////

#include <tf.h>

tf::Transformer tf_transformer;


static tf::Vector3
carmen_vector3_to_tf_vector3(carmen_vector_3D_t carmen_vector)
{
	tf::Vector3 tf_vector(carmen_vector.x, carmen_vector.y, carmen_vector.z);

	return (tf_vector);
}


static carmen_vector_3D_t
tf_vector3_to_carmen_vector3(tf::Vector3 tf_vector)
{
	carmen_vector_3D_t carmen_vector;
	carmen_vector.x = tf_vector.x();
	carmen_vector.y = tf_vector.y();
	carmen_vector.z = tf_vector.z();

	return (carmen_vector);
}


static tf::Quaternion
carmen_rotation_to_tf_quaternion(carmen_orientation_3D_t carmen_orientation)
{
	tf::Quaternion tf_quat(carmen_orientation.yaw, carmen_orientation.pitch, carmen_orientation.roll);

	return tf_quat;
}


static carmen_vector_3D_t
carmen_ackerman_interpolated_robot_position_at_time(carmen_vector_3D_t robot_pose, double dt, double v, double theta)
{
	carmen_vector_3D_t pose = robot_pose;
	int i;
	int steps = 1;
	double ds;

	ds = v * (dt / (double) steps);

	for (i = 0; i < steps; i++)
	{
		pose.x = pose.x + ds * cos(theta);
		pose.y = pose.y + ds * sin(theta);
	}

	return (pose);
}


carmen_vector_3D_t
get_gps_pose_from_car_pose(graphslam_pose_t car_pose, double theta, double v, double timestamp)
{
	tf::StampedTransform car_to_gps;
	tf_transformer.lookupTransform((char *) "/car", (char *) "/gps", tf::Time(0), car_to_gps);

	carmen_vector_3D_t car_position;
	car_position.x = car_pose.x;
	car_position.y = car_pose.y;
	car_position.z = 0.0;

	tf::Transform global_to_car;
	global_to_car.setOrigin(carmen_vector3_to_tf_vector3(car_position));
	global_to_car.setRotation(carmen_rotation_to_tf_quaternion({0.0, 0.0, theta}));

	tf::Transform global_to_gps = global_to_car * car_to_gps;

	carmen_vector_3D_t gps_position = tf_vector3_to_carmen_vector3(global_to_gps.getOrigin());

	gps_position = carmen_ackerman_interpolated_robot_position_at_time(gps_position, timestamp - car_pose.timestamp, v, theta);

	return (gps_position);
}


struct mystruct_comparer
{
    bool operator ()(graphslam_pose_t const& ms, double const timestamp) const
    {
        return ms.timestamp < timestamp;
    }
};


graphslam_pose_t *
get_nearest_graphslam_gps_pose_opt(double timestamp)
{
	vector<graphslam_pose_t>::iterator car_pose_iter = lower_bound(graphslam_poses_opt.begin(),
    		graphslam_poses_opt.end(),
			timestamp,
            mystruct_comparer());

	int base_ackerman_odometry_index = gps_xyz_get_base_ackerman_odometry_index_by_timestamp(timestamp);
	double v = 0.0;
	if (fabs(base_ackerman_odometry_vector[base_ackerman_odometry_index].timestamp - timestamp) < 0.2)
		v = base_ackerman_odometry_vector[base_ackerman_odometry_index].v;

	carmen_vector_3D_t gps_pose = get_gps_pose_from_car_pose(*car_pose_iter, (*car_pose_iter).theta, v, timestamp);
	static graphslam_pose_t pose;
	double delta_t = fabs((*car_pose_iter).timestamp - timestamp);
//	printf("%lf %lf - delta_t %lf, %lf ", (*car_pose_iter).timestamp, timestamp, delta_t, v);
    if (delta_t < 0.1)
    {
    	pose = {gps_pose.x, gps_pose.y, (*car_pose_iter).theta, timestamp};
    	return (&pose);
    }
    else
    	return (NULL);
}
//////////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Handlers                                                                                     //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


void
carmen_gps_gpgga_message_handler(carmen_gps_gpgga_message *gps_gpgga)
{
	double latitude = 0;
	double longitude = 0;
	carmen_gps_xyz_message gps_xyz_message;

//	static double previous_utc = -1.0;
//	static double previous_timestamp = -1.0;

//	if (gps_gpgga->utc == previous_utc)
//		return;

//	previous_utc = gps_gpgga->utc;
	gps_xyz_message.nr = gps_gpgga->nr;
	gps_xyz_message.utc = gps_gpgga->utc;
	gps_xyz_message.latitude = gps_gpgga->latitude;
	gps_xyz_message.latitude_dm = gps_gpgga->latitude_dm;
	gps_xyz_message.lat_orient = gps_gpgga->lat_orient;
	gps_xyz_message.longitude = gps_gpgga->longitude;
	gps_xyz_message.longitude_dm = gps_gpgga->longitude_dm;
	gps_xyz_message.long_orient = gps_gpgga->long_orient;
	gps_xyz_message.gps_quality = gps_gpgga->gps_quality;
	gps_xyz_message.num_satellites = gps_gpgga->num_satellites;
	gps_xyz_message.hdop = gps_gpgga->hdop;
	gps_xyz_message.sea_level = gps_gpgga->sea_level;
	gps_xyz_message.altitude = gps_gpgga->altitude;
	gps_xyz_message.geo_sea_level = gps_gpgga->geo_sea_level;
	gps_xyz_message.geo_sep = gps_gpgga->geo_sep;
	gps_xyz_message.data_age = gps_gpgga->data_age;

	if (gps_gpgga->lat_orient == 'S') latitude = -gps_gpgga->latitude;
	if (gps_gpgga->long_orient == 'W') longitude = -gps_gpgga->longitude;

// Transformando o z utilizando como altitude o sea_level
	Gdc_Coord_3d gdc = Gdc_Coord_3d(latitude, longitude, gps_gpgga->sea_level);

// Transformando o z utilizando como altitude a altitude mesmo - que esta vindo como zero
	//Gdc_Coord_3d gdc = Gdc_Coord_3d(latitude, longitude, gps_gpgga->altitude);
	Utm_Coord_3d utm;
	Gdc_To_Utm_Converter::Init();
	Gdc_To_Utm_Converter::Convert(gdc,utm);

	//gps_xyz_message.x = utm.x;
	//gps_xyz_message.y = utm.y;
	//gps_xyz_message.z = utm.z;

	gps_xyz_message.x = utm.y;
	gps_xyz_message.y = -utm.x;
	
	// *********************************************************************************************************************************************
	// @@@ Alberto: ATENCAO! ATENCAO! ATENCAO: estou zerando o z do gps que fiz pois ele varia muito. Quando tivermos um bom tem que parar de zerar!
	// gps_xyz_message.z = utm.z;
	// *********************************************************************************************************************************************
	gps_xyz_message.z = 0.0;
	gps_xyz_message.zone = (double)utm.zone;

	//printf("%lf %lf -> %lf %lf %lf\n", latitude, longitude, gps_xyz_message.x, gps_xyz_message.y, gps_xyz_message.z);

	if (utm.hemisphere_north == true)
		gps_xyz_message.hemisphere_north = 1;
	else
		gps_xyz_message.hemisphere_north = 0;

	gps_xyz_message.timestamp = gps_gpgga->timestamp;
	gps_xyz_message.host = carmen_get_host();

	static double first_timestamp = 0.0;
	if (first_timestamp == 0)
		first_timestamp = gps_xyz_message.timestamp;
	if (gps_xyz_message.timestamp - first_timestamp < 5.0) // wait for deep_vgl
		return;

	if ((gps_xyz_message.gps_quality < 4) && (graphslam_poses_opt.size() > 0))
	{
		graphslam_pose_t *graphslam_gps_pose = get_nearest_graphslam_gps_pose_opt(gps_xyz_message.timestamp);
		if (graphslam_gps_pose)
		{
			gps_xyz_message.gps_quality = 6;
			gps_xyz_message.x = graphslam_gps_pose->x;
			gps_xyz_message.y = graphslam_gps_pose->y;
		}
	}
//	printf("%lf %lf %d\n", gps_xyz_message.x, gps_xyz_message.y, gps_xyz_message.gps_quality);

	carmen_gps_xyz_publish_message(gps_xyz_message);

	if ((gps_xyz_message.nr == GPS_1) || (gps_xyz_message.nr == GPS_2))
		gps_xyz_message_queue.push_back(gps_xyz_message);

	if (gps_xyz_message_queue.size() > GPS_MESSAGE_QUEUE_SIZE)
		gps_xyz_message_queue.erase(gps_xyz_message_queue.begin());

	carmen_gps_gphdt_message carmen_extern_gphdt;
	if (get_carmen_gps_gphdt_message(gps_xyz_message_queue, &carmen_extern_gphdt))
		publish_carmen_gps_gphdt_message(&carmen_extern_gphdt);
}


static void
xsens_mtig_handler(carmen_xsens_mtig_message *message)
{
	carmen_xsens_xyz_message xsens_xyz_message;

	xsens_xyz_message.quat = message->quat;
	xsens_xyz_message.acc = message->acc;
	xsens_xyz_message.gyr = message->gyr;
	xsens_xyz_message.mag = message->mag;
	xsens_xyz_message.velocity = message->velocity;

	Gdc_Coord_3d gdc = Gdc_Coord_3d(message->latitude, message->longitude, message->height);
	Utm_Coord_3d utm;
	Gdc_To_Utm_Converter::Init();
	Gdc_To_Utm_Converter::Convert(gdc,utm);

	// The axis are changed to match the carmen frame of reference
	xsens_xyz_message.position.x = utm.y;
	xsens_xyz_message.position.y = -utm.x;
	xsens_xyz_message.position.z = utm.z;

	xsens_xyz_message.gps_fix = message->gps_fix;
	xsens_xyz_message.xkf_valid = message->xkf_valid;
	xsens_xyz_message.sensor_ID = message->sensor_ID;
	xsens_xyz_message.timestamp = message->timestamp;
	xsens_xyz_message.host = carmen_get_host();

	publish_xsens_xyz_message(xsens_xyz_message);
}


void
velodyne_gps_handler(carmen_velodyne_gps_message *message)
{
	carmen_velodyne_gps_xyz_message velodyne_gps_xyz_message;

	velodyne_gps_xyz_message.accel1_x = message->accel1_x;
	velodyne_gps_xyz_message.accel1_y = message->accel1_y;
	velodyne_gps_xyz_message.accel2_x = message->accel2_x;
	velodyne_gps_xyz_message.accel2_y = message->accel2_y;
	velodyne_gps_xyz_message.accel3_x = message->accel3_x;
	velodyne_gps_xyz_message.accel3_y = message->accel3_y;

	velodyne_gps_xyz_message.gyro1 = message->gyro1;
	velodyne_gps_xyz_message.gyro2 = message->gyro2;
	velodyne_gps_xyz_message.gyro3 = message->gyro3;

	velodyne_gps_xyz_message.temp1 = message->temp1;
	velodyne_gps_xyz_message.temp2 = message->temp2;
	velodyne_gps_xyz_message.temp3 = message->temp3;

	velodyne_gps_xyz_message.utc_time = message->utc_time;
	velodyne_gps_xyz_message.utc_date = message->utc_date;
	velodyne_gps_xyz_message.status = message->status;

	velodyne_gps_xyz_message.latitude = carmen_global_convert_degmin_to_double(message->latitude);
	velodyne_gps_xyz_message.latitude_hemisphere = message->latitude_hemisphere;
	velodyne_gps_xyz_message.longitude = carmen_global_convert_degmin_to_double(message->longitude);
	velodyne_gps_xyz_message.longitude_hemisphere = message->longitude_hemisphere;
	velodyne_gps_xyz_message.speed_over_ground = message->speed_over_ground;
	velodyne_gps_xyz_message.course_over_ground = message->course_over_ground;
	velodyne_gps_xyz_message.magnetic_variation_course = message->magnetic_variation_course;
	velodyne_gps_xyz_message.magnetic_variation_direction = message->magnetic_variation_direction;
	velodyne_gps_xyz_message.mode_indication = message->mode_indication;
	velodyne_gps_xyz_message.timestamp = message->timestamp;

	if (velodyne_gps_xyz_message.latitude_hemisphere == 'S')
		velodyne_gps_xyz_message.latitude = -velodyne_gps_xyz_message.latitude;
	if (velodyne_gps_xyz_message.longitude_hemisphere == 'W')
		velodyne_gps_xyz_message.longitude = -velodyne_gps_xyz_message.longitude;


	// Transformando o z utilizando como altitude o sea_level

	Gdc_Coord_3d gdc = Gdc_Coord_3d((double) velodyne_gps_xyz_message.latitude, (double)velodyne_gps_xyz_message.longitude, 0.0);
	Utm_Coord_3d utm;
	Gdc_To_Utm_Converter::Init();
	Gdc_To_Utm_Converter::Convert(gdc,utm);

	// The axis are changed to match the carmen frame of reference
	velodyne_gps_xyz_message.position.x = utm.y;
	velodyne_gps_xyz_message.position.y = -utm.x;
	velodyne_gps_xyz_message.position.z = utm.z;

	publish_velodyne_gps_xyz_message(velodyne_gps_xyz_message);
}


static void
base_ackerman_odometry_handler(carmen_base_ackerman_odometry_message *msg)
{
	base_ackerman_odometry_index = (base_ackerman_odometry_index + 1) % BASE_ACKERMAN_ODOMETRY_VECTOR_SIZE;
	base_ackerman_odometry_vector[base_ackerman_odometry_index] = *msg;
}


void
shutdown_module(int signo)
{
		carmen_ipc_disconnect();
	if (signo == SIGINT)
	{
		printf("gps_xyz: disconnected.\n");
		exit(0);
	}
}
//////////////////////////////////////////////////////////////////////////////////////////////////


void
read_poses_opt_data_file(char *poses_opt_file_name)
{
	FILE *f = fopen(poses_opt_file_name, "r");
	if (!f)
	{
		printf("Error: Could not open poses opt file %s\n", poses_opt_file_name);
		return;
	}

	printf("Reading poses opt file %s\n", poses_opt_file_name);
	while(!feof(f))
	{
		graphslam_pose_t pose_opt;
		fscanf(f, "%lf %lf %lf %lf\n", &pose_opt.x, &pose_opt.y, &pose_opt.theta, &pose_opt.timestamp);
		graphslam_poses_opt.push_back(pose_opt);
	}
	printf("%d poses read.\n", (int) graphslam_poses_opt.size());
}


void
initialize_tf_transforms()
{
	tf::Transform board_to_gps_pose;
	tf::Transform car_to_board_pose;

	tf::Time::init();

	// board pose with respect to the car
	car_to_board_pose.setOrigin(tf::Vector3(sensor_board_1_pose.position.x, sensor_board_1_pose.position.y, sensor_board_1_pose.position.z));
	car_to_board_pose.setRotation(tf::Quaternion(sensor_board_1_pose.orientation.yaw, sensor_board_1_pose.orientation.pitch, sensor_board_1_pose.orientation.roll)); 				// yaw, pitch, roll
	tf::StampedTransform car_to_board_transform(car_to_board_pose, tf::Time(0), "/car", "/board");
	tf_transformer.setTransform(car_to_board_transform, "car_to_board_transform");

	// gps pose with respect to the board
	board_to_gps_pose.setOrigin(tf::Vector3(gps_pose_in_the_car.position.x, gps_pose_in_the_car.position.y, gps_pose_in_the_car.position.z));
	board_to_gps_pose.setRotation(tf::Quaternion(gps_pose_in_the_car.orientation.yaw, gps_pose_in_the_car.orientation.pitch, gps_pose_in_the_car.orientation.roll)); 				// yaw, pitch, roll
	tf::StampedTransform board_to_gps_transform(board_to_gps_pose, tf::Time(0), "/board", "/gps");
	tf_transformer.setTransform(board_to_gps_transform, "board_to_gps_transform");
}


void
gps_xyz_read_parameters(int argc, char **argv)
{
	carmen_param_t param_list[] =
	{
		{(char *) "sensor_board_1", (char *) "x", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.x),	0, NULL},
		{(char *) "sensor_board_1", (char *) "y", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.y),	0, NULL},
		{(char *) "sensor_board_1", (char *) "z", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.z),	0, NULL},
		{(char *) "sensor_board_1", (char *) "roll", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.roll),0, NULL},
		{(char *) "sensor_board_1", (char *) "pitch", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.pitch),0, NULL},
		{(char *) "sensor_board_1", (char *) "yaw", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.yaw),	0, NULL},
		{(char *) "gps", (char *) "nmea_1_x",		CARMEN_PARAM_DOUBLE, &gps_pose_in_the_car.position.x,		1, NULL},
		{(char *) "gps", (char *) "nmea_1_y",		CARMEN_PARAM_DOUBLE, &gps_pose_in_the_car.position.y,		1, NULL},
		{(char *) "gps", (char *) "nmea_1_z",		CARMEN_PARAM_DOUBLE, &gps_pose_in_the_car.position.z,		1, NULL},
		{(char *) "gps", (char *) "nmea_1_roll",	CARMEN_PARAM_DOUBLE, &gps_pose_in_the_car.orientation.roll,		1, NULL},
		{(char *) "gps", (char *) "nmea_1_pitch",	CARMEN_PARAM_DOUBLE, &gps_pose_in_the_car.orientation.pitch,	1, NULL},
		{(char *) "gps", (char *) "nmea_1_yaw",		CARMEN_PARAM_DOUBLE, &gps_pose_in_the_car.orientation.yaw,		1, NULL},
	};

	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));
}


int
main(int argc, char *argv[])
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	signal(SIGINT, shutdown_module);

	if (argc == 2)
		read_poses_opt_data_file(argv[1]);

	gps_xyz_read_parameters(argc, argv);
	initialize_tf_transforms();

	carmen_gps_xyz_define_message();
	carmen_xsens_xyz_define_message();
	carmen_velodyne_gps_xyz_define_message();

	carmen_gps_subscribe_nmea_message(NULL, (carmen_handler_t) carmen_gps_gpgga_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_xsens_mtig_subscribe_message(NULL, (carmen_handler_t) xsens_mtig_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_velodyne_subscribe_gps_message(NULL, (carmen_handler_t) velodyne_gps_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_base_ackerman_subscribe_odometry_message(NULL, (carmen_handler_t) base_ackerman_odometry_handler, CARMEN_SUBSCRIBE_LATEST);

	/* Loop forever waiting for messages */
	carmen_ipc_dispatch();

	return(0);
}

