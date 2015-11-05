#include <carmen/carmen.h>
#include <carmen/base_ackerman_interface.h>
#include <carmen/gps_xyz_interface.h>

#include "fused_odometry.h"
#include "fused_odometry_kalman.h"
#include "xsens_xyz_handler.h"
#include "car_odometry_handler.h"
#include <carmen/fused_odometry_interface.h>

static car_odometry_handler car_handler;

static carmen_fused_odometry_parameters *fused_odometry_parameters;

extern sensor_vector_xsens_xyz **xsens_sensor_vector;
extern int xsens_sensor_vector_index;

extern sensor_vector_xsens_xyz **gps_sensor_vector;
extern int gps_sensor_vector_index;

extern sensor_vector_xsens_xyz **xsens_mti_sensor_vector;
extern int xsens_mti_sensor_vector_index;
extern xsens_xyz_handler xsens_handler;


static void 
car_odometry_message_handler(carmen_base_ackerman_odometry_message *car_odometry_message)
{
	carmen_point_t k_pose;
	if (xsens_handler.initial_state_initialized)
	{
		k_pose = kalman_filter_correct_pose(gps_sensor_vector, xsens_mti_sensor_vector, gps_sensor_vector_index, xsens_mti_sensor_vector_index, car_odometry_message, fused_odometry_parameters->axis_distance);

		carmen_fused_odometry_message fused_odometry_message;

		fused_odometry_message.phi = car_odometry_message->phi;
		fused_odometry_message.velocity.x = car_odometry_message->v;
		fused_odometry_message.pose.orientation.yaw = k_pose.theta;
		fused_odometry_message.pose.orientation.pitch = 0.0;//xsens_mti_sensor_vector[xsens_mti_sensor_vector_index]->orientation.pitch;
		fused_odometry_message.pose.orientation.roll = 0.0;//xsens_mti_sensor_vector[xsens_mti_sensor_vector_index]->orientation.roll;
		fused_odometry_message.pose.position.z = 0.0;
		fused_odometry_message.pose.position.y = k_pose.y;
		fused_odometry_message.pose.position.x = k_pose.x;
		fused_odometry_message.timestamp = car_odometry_message->timestamp;
		fused_odometry_message.xsens_yaw_bias = 0.0;
		fused_odometry_message.host = carmen_get_host();

		IPC_publishData(CARMEN_FUSED_ODOMETRY_NAME, &fused_odometry_message);
	}
}


static void 
subscribe_messages(void)
{	
	carmen_base_ackerman_subscribe_odometry_message(NULL, (carmen_handler_t) car_odometry_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


static void
unsubscribe_messages(void)
{
	//carmen_base_ackerman_unsubscribe_odometry_message((carmen_handler_t)car_odometry_message_handler);
}


car_odometry_handler *
create_car_odometry_handler(xsens_xyz_handler *xsens_handler, carmen_fused_odometry_parameters *parameters)
{
	fused_odometry_parameters = parameters;
	
	car_handler.xsens_handler = xsens_handler;
	
	subscribe_messages();

	return (&car_handler);
}


void 
reset_car_odometry_handler(car_odometry_handler *car_handler __attribute__ ((unused)))
{
}


void 
destroy_car_odometry_handler(car_odometry_handler *car_handler __attribute__ ((unused)))
{
	unsubscribe_messages();
}
