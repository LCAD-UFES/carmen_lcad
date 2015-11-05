#include <carmen/carmen.h>
#include <carmen/gps_xyz_interface.h>

#include "fused_odometry.h"
#include "fused_odometry_messages.h"
#include "fused_odometry_kalman.h"
#include "xsens_xyz_handler.h"
#include "car_odometry_handler.h"

static car_odometry_handler car_handler;

static carmen_fused_odometry_parameters *fused_odometry_parameters;

extern sensor_vector_xsens_xyz **xsens_sensor_vector;
extern int xsens_sensor_vector_index;
extern int kalman_filter;

static carmen_fused_odometry_control 
create_car_odometry_control(carmen_base_ackerman_odometry_message *car_odometry_message)
{	
	carmen_fused_odometry_control ut;

	ut.v = car_odometry_message->v;
	ut.v_z = 0.0;
	ut.phi = car_odometry_message->phi;
	ut.v_pitch = car_handler.xsens_handler->ang_velocity.pitch;
	ut.z = 0.0;
	ut.pitch = car_handler.xsens_handler->orientation.pitch;
	ut.roll = car_handler.xsens_handler->orientation.roll;

	ut.gps_available = 1;

	return (ut);
}


static void 
car_odometry_message_handler(carmen_base_ackerman_odometry_message *car_odometry_message)
{
	carmen_fused_odometry_control ut = create_car_odometry_control(car_odometry_message);

	set_fused_odometry_control_vector(ut);

	if (kalman_filter)
	{
		carmen_point_t k_pose;
		k_pose = kalman_filter_correct_pose(xsens_sensor_vector, xsens_sensor_vector_index, car_odometry_message, fused_odometry_parameters->axis_distance);

		carmen_fused_odometry_message fused_odometry_message;

		fused_odometry_message.pose.orientation.yaw = k_pose.theta;
		fused_odometry_message.pose.position.y = k_pose.y;
		fused_odometry_message.pose.position.x = k_pose.x;
		fused_odometry_message.timestamp = car_odometry_message->timestamp;

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
