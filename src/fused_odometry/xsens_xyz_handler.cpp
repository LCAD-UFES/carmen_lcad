#include <carmen/carmen.h>
#include <carmen/gps_xyz_interface.h>
#include <tf.h>

#include "fused_odometry.h"
#include "xsens_xyz_handler.h"


xsens_xyz_handler xsens_handler;
	
static tf::Transformer transformer(false);

static char *car_tf_name = (char *) "/car";
static char *xsens_tf_name = (char *) "/xsens";
static char *board_tf_name = (char *) "/board";
static char *gps_tf_name = (char *) "/gps";

static carmen_fused_odometry_parameters *fused_odometry_parameters;

static int reinitialized_gps = 0;

sensor_vector_xsens_xyz **xsens_sensor_vector = NULL;
int xsens_sensor_vector_index = 0;

extern int kalman_filter;


int
is_global_pos_initialized()
{
	return (xsens_handler.initial_state_initialized);
}


static carmen_vector_3D_t 
get_car_acceleration_from_message(carmen_xsens_xyz_message *xsens_xyz_message)
{
	// Gravity is not being removed, should it be?

	carmen_vector_3D_t acceleration = xsens_xyz_message->acc;
	
	return acceleration;
}


static carmen_vector_3D_t 
get_car_acceleration_from_message(carmen_xsens_global_quat_message *xsens_mti_message)
{
	// Gravity is not being removed, should it be?

	carmen_vector_3D_t acceleration;

	acceleration.x = xsens_mti_message->m_acc.x;
	acceleration.y = xsens_mti_message->m_acc.y;
	acceleration.z = xsens_mti_message->m_acc.z;
	
	return acceleration;
}


static tf::Quaternion 
carmen_quaternion_to_tf_quaternion(carmen_quaternion_t carmen_quaternion)
{
	tf::Quaternion tf_quat(carmen_quaternion.q1, carmen_quaternion.q2, carmen_quaternion.q3, carmen_quaternion.q0);

	return tf_quat;
}


static carmen_orientation_3D_t 
get_carmen_orientation_from_tf_transform(tf::Transform transform)
{
	carmen_orientation_3D_t orientation;
	tf::Matrix3x3(transform.getRotation()).getEulerYPR(orientation.yaw, orientation.pitch, orientation.roll);
	
	return orientation;
}


static carmen_orientation_3D_t 
get_orientation_car_reference_from_message(tf::Quaternion xsens_reading)
{
	tf::StampedTransform xsens_to_car;
	transformer.lookupTransform(xsens_tf_name, car_tf_name, tf::Time(0), xsens_to_car);

	tf::Transform xsens_matrix;
	xsens_matrix.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	xsens_matrix.setRotation(xsens_reading);
	
	tf::Transform car_pose = xsens_matrix * xsens_to_car;
	
	carmen_orientation_3D_t orientation = get_carmen_orientation_from_tf_transform(car_pose);

	return orientation;
}


static carmen_orientation_3D_t 
get_car_orientation_from_message(carmen_xsens_xyz_message *xsens_xyz_message)
{	
	tf::Quaternion tf_quat_xsens = carmen_quaternion_to_tf_quaternion(xsens_xyz_message->quat);
	
	carmen_orientation_3D_t orientation = get_orientation_car_reference_from_message(tf_quat_xsens);

	return orientation;
}


static carmen_orientation_3D_t 
get_car_orientation_from_message(carmen_xsens_global_quat_message *xsens_mti_message)
{	
	tf::Quaternion tf_quat_xsens(xsens_mti_message->quat_data.m_data[1], xsens_mti_message->quat_data.m_data[2], xsens_mti_message->quat_data.m_data[3], xsens_mti_message->quat_data.m_data[0]);
	
	carmen_orientation_3D_t orientation = get_orientation_car_reference_from_message(tf_quat_xsens);

	return orientation;
}


static tf::Quaternion 
carmen_rotation_to_tf_quaternion(carmen_orientation_3D_t carmen_orientation)
{
	tf::Quaternion tf_quat(carmen_orientation.yaw, carmen_orientation.pitch, carmen_orientation.roll);

	return tf_quat;
}


static carmen_orientation_3D_t 
get_car_ang_velocity_from_message(carmen_xsens_xyz_message *xsens_xyz_message)
{
	carmen_orientation_3D_t gyr_orientation;
	gyr_orientation.roll = xsens_xyz_message->gyr.x;
	gyr_orientation.pitch = xsens_xyz_message->gyr.y;
	gyr_orientation.yaw = xsens_xyz_message->gyr.z;

	tf::Quaternion tf_quat_xsens = carmen_rotation_to_tf_quaternion(gyr_orientation);
	
	carmen_orientation_3D_t ang_velocity = get_orientation_car_reference_from_message(tf_quat_xsens);

	return ang_velocity;
}


static carmen_orientation_3D_t 
get_car_ang_velocity_from_message(carmen_xsens_global_quat_message *xsens_mti_message)
{
	carmen_orientation_3D_t gyr_orientation;
	gyr_orientation.roll = xsens_mti_message->m_gyr.x;
	gyr_orientation.pitch = xsens_mti_message->m_gyr.y;
	gyr_orientation.yaw = xsens_mti_message->m_gyr.z;

	tf::Quaternion tf_quat_xsens = carmen_rotation_to_tf_quaternion(gyr_orientation);
	
	carmen_orientation_3D_t ang_velocity = get_orientation_car_reference_from_message(tf_quat_xsens);

	return ang_velocity;
}


static tf::Vector3 
carmen_vector3_to_tf_vector3(carmen_vector_3D_t carmen_vector)
{
	tf::Vector3 tf_vector(carmen_vector.x, carmen_vector.y, carmen_vector.z);

	return tf_vector;
}


static carmen_vector_3D_t 
tf_vector3_to_carmen_vector3(tf::Vector3 tf_vector)
{
	carmen_vector_3D_t carmen_vector;
	carmen_vector.x = tf_vector.x();
	carmen_vector.y = tf_vector.y();
	carmen_vector.z = tf_vector.z();
	
	return carmen_vector;
}


static carmen_vector_3D_t 
get_car_position_from_message(carmen_xsens_xyz_message *xsens_xyz_message)
{
	tf::StampedTransform xsens_to_car;
	transformer.lookupTransform(xsens_tf_name, car_tf_name, tf::Time(0), xsens_to_car);
	
	tf::Transform global_to_xsens;
	global_to_xsens.setOrigin(carmen_vector3_to_tf_vector3(xsens_xyz_message->position));
	global_to_xsens.setRotation(carmen_quaternion_to_tf_quaternion(xsens_xyz_message->quat));
	
	tf::Transform global_to_car = global_to_xsens * xsens_to_car;

	carmen_vector_3D_t car_position = tf_vector3_to_carmen_vector3(global_to_car.getOrigin());
	
	return car_position;
}


static carmen_vector_3D_t 
get_car_position_from_message(carmen_gps_xyz_message *gps_xyz_message)
{
	tf::StampedTransform gps_to_car;
	transformer.lookupTransform(gps_tf_name, car_tf_name, tf::Time(0), gps_to_car);
	
	carmen_vector_3D_t gps_position;
	gps_position.x = gps_xyz_message->x;
	gps_position.y = gps_xyz_message->y;
	gps_position.z = gps_xyz_message->z;

	tf::Transform global_to_gps;
	global_to_gps.setOrigin(carmen_vector3_to_tf_vector3(gps_position));
	tf::Quaternion zero_quat(0.0, 0.0, 0.0);
	global_to_gps.setRotation(zero_quat);
	
	tf::Transform global_to_car = global_to_gps * gps_to_car;

	carmen_vector_3D_t car_position = tf_vector3_to_carmen_vector3(global_to_car.getOrigin());
	
	return car_position;
}


static sensor_vector_xsens_xyz * 
create_sensor_vector_xsens_xyz(carmen_xsens_xyz_message *xsens_xyz_message)
{
	sensor_vector_xsens_xyz *sensor_vector = (sensor_vector_xsens_xyz*)malloc(sizeof(sensor_vector_xsens_xyz));

	sensor_vector->acceleration = get_car_acceleration_from_message(xsens_xyz_message);
	sensor_vector->orientation = get_car_orientation_from_message(xsens_xyz_message);
	sensor_vector->ang_velocity = get_car_ang_velocity_from_message(xsens_xyz_message);
	sensor_vector->position = get_car_position_from_message(xsens_xyz_message);
	sensor_vector->timestamp = xsens_xyz_message->timestamp;

	return sensor_vector;
}


static sensor_vector_xsens_xyz * 
create_sensor_vector_xsens_mti(carmen_xsens_global_quat_message *xsens_mti_message, carmen_gps_xyz_message *gps_xyz_message)
{
	sensor_vector_xsens_xyz *sensor_vector = (sensor_vector_xsens_xyz *) malloc(sizeof(sensor_vector_xsens_xyz));

	sensor_vector->acceleration = get_car_acceleration_from_message(xsens_mti_message);
	sensor_vector->orientation = get_car_orientation_from_message(xsens_mti_message);
	sensor_vector->ang_velocity = get_car_ang_velocity_from_message(xsens_mti_message);
	sensor_vector->position = get_car_position_from_message(gps_xyz_message);
	sensor_vector->timestamp = xsens_mti_message->timestamp;

	return sensor_vector;
}


static sensor_vector_xsens_xyz * 
create_sensor_vector_gps_xyz(carmen_gps_xyz_message *gps_xyz_message)
{
	sensor_vector_xsens_xyz *sensor_vector = (sensor_vector_xsens_xyz *) malloc(sizeof(sensor_vector_xsens_xyz));

	carmen_vector_3D_t zero = {0.0, 0.0, 0.0};
	carmen_orientation_3D_t zero_ori = {0.0, 0.0, 0.0};

	sensor_vector->acceleration = zero;
	sensor_vector->orientation = zero_ori;
	sensor_vector->ang_velocity = zero_ori;
	sensor_vector->position = get_car_position_from_message(gps_xyz_message);
	sensor_vector->timestamp = gps_xyz_message->timestamp;

	return sensor_vector;
}


static sensor_vector_xsens_xyz *
create_sensor_vector_gps_hdt(carmen_gps_gphdt_message *gps_hdt_message)
{
	sensor_vector_xsens_xyz *sensor_vector = (sensor_vector_xsens_xyz *) malloc(sizeof(sensor_vector_xsens_xyz));

	carmen_vector_3D_t zero = {0.0, 0.0, 0.0};
	carmen_orientation_3D_t zero_ori = {0.0, 0.0, 0.0};
	carmen_orientation_3D_t heading = {0.0, 0.0, gps_hdt_message->heading};

	sensor_vector->acceleration = zero;
	sensor_vector->orientation = heading;
	sensor_vector->ang_velocity = zero_ori;
	sensor_vector->position = zero;
	sensor_vector->timestamp = gps_hdt_message->timestamp;

	return sensor_vector;
}


static void 
destroy_sensor_vector_xsens_xyz(sensor_vector_xsens_xyz *sensor_vector)
{
	free(sensor_vector);
}


static carmen_fused_odometry_state_vector 
create_state_vector_from_message(carmen_xsens_xyz_message *xsens_xyz)
{
	sensor_vector_xsens_xyz *sensor_vector = create_sensor_vector_xsens_xyz(xsens_xyz);

	carmen_vector_3D_t zero = {0.0, 0.0, 0.0};

	carmen_fused_odometry_state_vector sv;

	sv.pose.position = sensor_vector->position;
	sv.xsens_yaw_bias = 0.0;
	sv.pose.orientation = sensor_vector->orientation;
	sv.velocity = zero;
	sv.ang_velocity = sensor_vector->ang_velocity;
	sv.phi = 0.0;
	sv.timestamp = sensor_vector->timestamp;

	destroy_sensor_vector_xsens_xyz(sensor_vector);

	return sv;
}

static carmen_fused_odometry_state_vector 
create_state_vector_from_message(carmen_gps_xyz_message *gps_xyz_message)
{
	sensor_vector_xsens_xyz *sensor_vector = create_sensor_vector_gps_xyz(gps_xyz_message);

	carmen_vector_3D_t zero = {0.0, 0.0, 0.0};
	carmen_orientation_3D_t zero_ori = {0.0, 0.0, 0.0};

	carmen_fused_odometry_state_vector sv;

	sv.pose.position = sensor_vector->position;
	sv.xsens_yaw_bias = 0.0;
	sv.pose.orientation = zero_ori;
	sv.velocity = zero;
	sv.ang_velocity = zero_ori;
	sv.phi = 0.0;
	sv.timestamp = sensor_vector->timestamp;

	destroy_sensor_vector_xsens_xyz(sensor_vector);

	return sv;
}


carmen_point_t
get_std_error(xsens_xyz_handler *xsens_handler, carmen_fused_odometry_parameters *fused_odometry_parameters)
{
	carmen_point_t std;

	if (xsens_handler->extra_gps == 2)
	{
		if (xsens_handler->gps_xyz.gps_quality == 4)
		{
			std.x = fused_odometry_parameters->xsens_gps_x_std_error / 5.0;
			std.y = fused_odometry_parameters->xsens_gps_y_std_error / 5.0;
			std.theta = fused_odometry_parameters->xsens_yaw_std_error;// / 5.0;
		}
		else if (xsens_handler->gps_xyz.gps_quality == 5)
		{
			std.x = fused_odometry_parameters->xsens_gps_x_std_error / 2.0;
			std.y = fused_odometry_parameters->xsens_gps_y_std_error / 2.0;
			std.theta = fused_odometry_parameters->xsens_yaw_std_error;// / 2.0;
		}
		else
		{
			std.x = fused_odometry_parameters->xsens_gps_x_std_error;
			std.y = fused_odometry_parameters->xsens_gps_y_std_error;
			std.theta = fused_odometry_parameters->xsens_yaw_std_error;
		}
	}
	else
	{
		std.x = fused_odometry_parameters->xsens_gps_x_std_error;
		std.y = fused_odometry_parameters->xsens_gps_y_std_error;
		// @@@ Alberto: Nao se deve usar xsens_yaw_std_error porque o erro real esta associado ao bias
		std.theta = fused_odometry_parameters->xsens_maximum_yaw_bias;
	}

	std.x *= 1.0 + xsens_handler->gps_performance_degradation;
	std.y *= 1.0 + xsens_handler->gps_performance_degradation;
	std.theta *= 1.0 + xsens_handler->gps_performance_degradation + xsens_handler->gps_orientation_performance_degradation;

	return (std);
}


void
globalpos_ackerman_initialize_from_xsens(carmen_fused_odometry_state_vector initial_state, double timestamp)
{
	carmen_point_t mean;

	mean.x = initial_state.pose.position.x;
	mean.y = initial_state.pose.position.y;
	mean.theta = initial_state.pose.orientation.yaw;// - 0.35;

	carmen_point_t std;
	std.x = fused_odometry_parameters->xsens_gps_x_std_error;
	std.y = fused_odometry_parameters->xsens_gps_y_std_error;
	std.theta = fused_odometry_parameters->xsens_yaw_std_error;

	carmen_localize_ackerman_initialize_gaussian_time_command(mean, std, timestamp);

	init_particles(initial_state, fused_odometry_parameters);
}


void 
initialize_states(carmen_xsens_xyz_message *xsens_xyz)
{
	static int first_time = 1;
	static double first_timestamp = 0.0;
	
	if (first_time)
	{
		first_timestamp = carmen_get_time();
		first_time = 0;
	}
	
	if (carmen_get_time() > (first_timestamp + 2.0))
	{
		if (xsens_handler.extra_gps)
		{
			if (xsens_handler.gps_xyz.gps_quality)
			{
				carmen_fused_odometry_state_vector initial_state;
				initial_state = create_state_vector_from_message(&xsens_handler.gps_xyz);
				if (xsens_handler.gps_hdt.valid && (xsens_handler.extra_gps == 2)) // Trimble
				{
					carmen_orientation_3D_t orientation = {0.0, 0.0, xsens_handler.gps_hdt.heading};
					initial_state.pose.orientation = orientation;
				}
				else
					initial_state.pose.orientation = get_car_orientation_from_message(xsens_xyz);

				globalpos_ackerman_initialize_from_xsens(initial_state, xsens_xyz->timestamp);

				xsens_handler.initial_state_initialized = 1;
			}
		}
		else
		{
			if (xsens_xyz->gps_fix)
			{
				carmen_fused_odometry_state_vector initial_state = create_state_vector_from_message(xsens_xyz);

				globalpos_ackerman_initialize_from_xsens(initial_state, xsens_xyz->timestamp);

				xsens_handler.initial_state_initialized = 1;
			}
		}
	}
}


void 
initialize_states(carmen_xsens_global_quat_message *xsens_mti)
{
	static int first_time = 1;
	static double first_timestamp = 0.0;
	
	if (first_time)
	{
		first_timestamp = carmen_get_time();
		first_time = 0;
	}
	
	if (carmen_get_time() > (first_timestamp + 2.0))
	{
		if (xsens_handler.gps_xyz.gps_quality)
		{
			carmen_fused_odometry_state_vector initial_state = create_state_vector_from_message(&xsens_handler.gps_xyz);
			if (xsens_handler.gps_hdt.valid && (xsens_handler.extra_gps == 2)) // Trimble
			{
				carmen_orientation_3D_t orientation = {0.0, 0.0, xsens_handler.gps_hdt.heading};
				initial_state.pose.orientation = orientation;
			}
			else
				initial_state.pose.orientation = get_car_orientation_from_message(xsens_mti);

			globalpos_ackerman_initialize_from_xsens(initial_state, xsens_mti->timestamp);

			xsens_handler.initial_state_initialized = 1;
		}
	}
}


static int 
check_time_difference(double timestamp)
{
	static double last_timestamp = 0.0;

	if (last_timestamp == 0.0)
	{
		last_timestamp = timestamp;
		return 0;
	}
	else if (fabs(timestamp - last_timestamp) > 3.0)	// Large time difference, filter need to be restarted
	{
		last_timestamp = timestamp;	
		return 1;
	}

	last_timestamp = timestamp;
	return 0;	
}


static double 
measure_weight_orientation(carmen_fused_odometry_state_vector xt, void *zt_v, carmen_fused_odometry_parameters *fused_odometry_parameters)
{
	if (xsens_handler.extra_gps == 2) // Trimble
		return (1.0); // Ignore orientation from xsens

	sensor_vector_xsens_xyz *zt = (sensor_vector_xsens_xyz *) zt_v;
	carmen_point_t std = get_std_error(&xsens_handler, fused_odometry_parameters);

	double diff_yaw = carmen_normalize_theta(xt.pose.orientation.yaw - (zt->orientation.yaw + xt.xsens_yaw_bias));
	double weight_yaw = exp(-(diff_yaw * diff_yaw / (2.0 * std.theta * std.theta)));

	return weight_yaw;
}


static double 
measure_weight_orientation_hdt(carmen_fused_odometry_state_vector xt, void *zt_v, carmen_fused_odometry_parameters *fused_odometry_parameters)
{
	sensor_vector_xsens_xyz *zt = (sensor_vector_xsens_xyz *) zt_v;
	carmen_point_t std = get_std_error(&xsens_handler, fused_odometry_parameters);

	double diff_yaw = carmen_normalize_theta(xt.pose.orientation.yaw - zt->orientation.yaw);
	double weight_yaw = exp(-(diff_yaw * diff_yaw / (2.0 * std.theta * std.theta)));

	return weight_yaw;
}


static double
measure_weight_position(carmen_fused_odometry_state_vector xt, void *zt_v, carmen_fused_odometry_parameters *fused_odometry_parameters)
{			
	sensor_vector_xsens_xyz *zt = (sensor_vector_xsens_xyz *) zt_v;

	double diff_x = xt.pose.position.x - zt->position.x;
	double diff_y = xt.pose.position.y - zt->position.y;
	//double diff_z = xt.pose.position.z - zt->position.z;
	
	carmen_point_t std = get_std_error(&xsens_handler, fused_odometry_parameters);
	double weight_x = exp(-((diff_x * diff_x) / (2.0 * std.x * std.x)));
	double weight_y = exp(-((diff_y * diff_y) / (2.0 * std.y * std.y)));
	//double weight_z = exp(-((diff_z * diff_z) / (2.0 * fused_odometry_parameters->xsens_gps_z_std_error * fused_odometry_parameters->xsens_gps_z_std_error)));
	
	double weight = (weight_x * weight_y); // * weight_z);

	return weight;
}


static double 
measure_weight_particle(carmen_fused_odometry_state_vector xt, void *sensor_vector, carmen_fused_odometry_parameters *fused_odometry_parameters)
{
	sensor_vector_xsens_xyz *zt = (sensor_vector_xsens_xyz *) sensor_vector;

	double weight_orientation = measure_weight_orientation(xt, zt, fused_odometry_parameters);
	double weight_position = measure_weight_position(xt, zt, fused_odometry_parameters);	
	
	double weight = weight_orientation * weight_position;

	return weight;

}


static void 
xsens_xyz_message_handler(carmen_xsens_xyz_message *xsens_xyz)
{
	sensor_vector_xsens_xyz *sensor_vector = create_sensor_vector_xsens_xyz(xsens_xyz);

	xsens_sensor_vector_index = (xsens_sensor_vector_index + 1) % XSENS_SENSOR_VECTOR_SIZE;

	if (xsens_sensor_vector[xsens_sensor_vector_index] != NULL)
		destroy_sensor_vector_xsens_xyz(xsens_sensor_vector[xsens_sensor_vector_index]);

	xsens_sensor_vector[xsens_sensor_vector_index] = sensor_vector;

	// Must check if timestamp has changed due to log file jump on log playback
	if (check_time_difference(xsens_xyz->timestamp))
	{
		carmen_fused_odometry_initialize(fused_odometry_parameters);
		initialize_states(xsens_xyz);
	
		return;
	}

	if (!xsens_handler.initial_state_initialized)
	{
		initialize_states(xsens_xyz);
		return;
	}

	xsens_handler.orientation = sensor_vector->orientation; 	// Used by create_car_odometry_control()
	xsens_handler.ang_velocity = sensor_vector->ang_velocity; 	// Used by create_car_odometry_control()

	if (!kalman_filter)
	{
		if (xsens_handler.gps_performance_changed)
		{
			xsens_handler.gps_performance_degradation = 10.0;
			xsens_handler.gps_performance_changed = 0;
		}
		if (xsens_handler.gps_orientation_performance_changed)
		{
			xsens_handler.gps_orientation_performance_degradation = 10.0;
			xsens_handler.gps_orientation_performance_changed = 0;
		}
		prediction(sensor_vector->timestamp, fused_odometry_parameters);
		correction(measure_weight_particle, sensor_vector, fused_odometry_parameters);
		publish_fused_odometry();
		xsens_handler.gps_performance_degradation *= 0.98; // fator de decaimento
		xsens_handler.gps_orientation_performance_degradation *= 0.98; // fator de decaimento
	}
	xsens_handler.last_xsens_message_timestamp = xsens_xyz->timestamp;
}


int
different_poses(carmen_vector_3D_t pose1, carmen_vector_3D_t pose2)
{
	double delta_x, delta_y, distance;
	
	delta_x = pose1.x - pose2.x;
	delta_y = pose1.y - pose2.y;
	
	distance = sqrt(delta_x * delta_x + delta_y * delta_y);
	
	if (distance > 0.0)
		return (1);
	else
		return (0); 
}


double
compute_yaw_from_two_poses(carmen_vector_3D_t pose1, carmen_vector_3D_t pose2)
{
	return (atan2((pose1.y - pose2.y), pose1.x - pose2.x));
}


double
compute_yaw_from_two_poses(carmen_point_t *pose1, carmen_point_t *pose2)
{
	return (atan2((pose1->y - pose2->y), pose1->x - pose2->x));
}


double
get_yaw_estimate_from_gps_poses(sensor_vector_xsens_xyz **xsens_sensor_vector, int xsens_sensor_vector_index)
{
	int i;
	double num_samples;
	int index;
	double average_x, average_y, yaw;
	carmen_vector_3D_t last_position;
	
	index = xsens_sensor_vector_index;
	average_x = average_y = 0.0;
	last_position = xsens_sensor_vector[index]->position;
	num_samples = 0.0;
	for (i = 0; i < XSENS_SENSOR_VECTOR_SIZE; i++)
	{
		if (different_poses(last_position, xsens_sensor_vector[index]->position))
		{
			yaw = compute_yaw_from_two_poses(last_position, xsens_sensor_vector[index]->position);
			average_x += cos(yaw);
			average_y += sin(yaw);
			
			num_samples += 1.0;
			
			if (num_samples > 10.0)
				break;
		}
		last_position = xsens_sensor_vector[index]->position;
		index--;
		if (index < 0)
			index = XSENS_SENSOR_VECTOR_SIZE - 1;

		if (xsens_sensor_vector[index] == NULL)
			break;
	}
	//printf("num_samples = %lf, i = %d\n", num_samples, i);
	if (num_samples > 10.0)
		return (atan2(average_y, average_x));
	else
		return (10.0);
}


void
set_best_yaw_estimate(sensor_vector_xsens_xyz **xsens_sensor_vector, int xsens_sensor_vector_index)
{
	carmen_fused_odometry_control *ut;
	double weight_factor;
	double xsens_yaw, gps_yaw;
	double x, y;
	
	xsens_yaw = xsens_sensor_vector[xsens_sensor_vector_index]->orientation.yaw;
	gps_yaw = get_yaw_estimate_from_gps_poses(xsens_sensor_vector, xsens_sensor_vector_index);

	ut = get_fused_odometry_control_vector();
	weight_factor = 1.0 / (1.0 + exp(-((fabs(ut->v) - 1.0) * 1.0)));
	// weight_factor = 0.0;
	
	if (gps_yaw != 10.0)
	{
		x = weight_factor * cos(gps_yaw) + (1.0 - weight_factor) * cos(xsens_yaw);
		y = weight_factor * sin(gps_yaw) + (1.0 - weight_factor) * sin(xsens_yaw);
		
		xsens_sensor_vector[xsens_sensor_vector_index]->orientation.yaw = atan2(y, x);
	}
		
	// printf("index = %d, xsens yaw before = %lf, xsens yaw after = %lf, gps yaw = %lf\n", xsens_sensor_vector_index, xsens_yaw, xsens_sensor_vector[xsens_sensor_vector_index]->orientation.yaw, gps_yaw);
}


static void 
xsens_mti_message_handler(carmen_xsens_global_quat_message *xsens_mti)
{
	if (!xsens_handler.initial_state_initialized)
	{
		initialize_states(xsens_mti);
		return;
	}

	// Must check if timestamp has changed due to log file jump on log playback

	if (reinitialized_gps)//if (check_time_difference(xsens_mti->timestamp))
	{
		carmen_fused_odometry_initialize(fused_odometry_parameters);
		initialize_states(xsens_mti);
		reinitialized_gps = 0;

		return;
	}

	xsens_sensor_vector_index = (xsens_sensor_vector_index + 1) % XSENS_SENSOR_VECTOR_SIZE;
	if (xsens_sensor_vector[xsens_sensor_vector_index] != NULL)
		destroy_sensor_vector_xsens_xyz(xsens_sensor_vector[xsens_sensor_vector_index]);

	sensor_vector_xsens_xyz *sensor_vector = create_sensor_vector_xsens_mti(xsens_mti, &xsens_handler.gps_xyz);
	xsens_sensor_vector[xsens_sensor_vector_index] = sensor_vector;
	
	set_best_yaw_estimate(xsens_sensor_vector, xsens_sensor_vector_index);

	xsens_handler.orientation = sensor_vector->orientation; 	// Used by create_car_odometry_control()
	xsens_handler.ang_velocity = sensor_vector->ang_velocity; 	// Used by create_car_odometry_control()

	if (!kalman_filter)
	{
		if (xsens_handler.gps_performance_changed)
		{
			xsens_handler.gps_performance_degradation = 50.0;
			xsens_handler.gps_performance_changed = 0;
		}
		if (xsens_handler.gps_orientation_performance_changed)
		{
			xsens_handler.gps_orientation_performance_degradation = 40.0;
			xsens_handler.gps_orientation_performance_changed = 0;
		}
		prediction(sensor_vector->timestamp, fused_odometry_parameters);
		correction(measure_weight_orientation, sensor_vector, fused_odometry_parameters);
		publish_fused_odometry();
		xsens_handler.gps_performance_degradation *= 0.98; // fator de decaimento
		xsens_handler.gps_orientation_performance_degradation *= 0.98; // fator de decaimento
	}
	xsens_handler.last_xsens_message_timestamp = xsens_mti->timestamp;
}


static void
detect_gps_performance_change(carmen_gps_xyz_message *message)
{
	static carmen_point_t previous_position = {0.0, 0.0, 0.0};

	if (!xsens_handler.initial_state_initialized)
	{
		previous_position.x = message->x;
		previous_position.y = message->y;
		return;
	}

	if (message->nr == 1)
	{
		if (xsens_handler.gps_xyz.gps_quality != message->gps_quality)
			xsens_handler.gps_performance_changed = 1;
		else
		{
			carmen_point_t current_position = {message->x, message->y, 0.0};
			carmen_fused_odometry_control *ut = get_fused_odometry_control_vector();
			double distance = carmen_distance(&previous_position, &current_position);
			if (distance > (0.25 + 2.0 * (ut->v * 1.0 /20.0))) // andou 2 vezes mais que o esperado
				xsens_handler.gps_performance_changed = 1;
			else
			{
				double car_yaw = get_fused_odometry()->pose.orientation.yaw;
				double gps_yaw = compute_yaw_from_two_poses(&previous_position, &current_position);
				double orthogonal_distance = distance * sin(car_yaw - gps_yaw);
				if (orthogonal_distance > 0.35)
					xsens_handler.gps_performance_changed = 1;
			}
		}
		previous_position.x = message->x;
		previous_position.y = message->y;
	}
}


static void
detect_gps_performance_change(carmen_gps_gphdt_message *message)
{
	static double previous_heading = 0.0;

	if (!xsens_handler.initial_state_initialized)
	{
		previous_heading = message->heading;
		return;
	}

	if ((xsens_handler.gps_hdt.valid != message->valid) ||
		(fabs(previous_heading - message->heading)) > carmen_degrees_to_radians(15.0))
		xsens_handler.gps_orientation_performance_changed = 1;

	previous_heading = message->heading;
}


static void 
gps_hdt_message_handler(carmen_gps_gphdt_message *message)
{
	detect_gps_performance_change(message);

	xsens_handler.gps_hdt = *message;

	if (!xsens_handler.initial_state_initialized)
		return;

//	if (check_time_difference(message->timestamp))
//	{
//		initialized_gps = 1;
//	}

	sensor_vector_xsens_xyz *sensor_vector = create_sensor_vector_gps_hdt(message);

	if (!kalman_filter)
	{
		if ((message->valid) && (xsens_handler.gps_xyz.gps_quality == 4))
			correction(measure_weight_orientation_hdt, sensor_vector, fused_odometry_parameters);
	}

	destroy_sensor_vector_xsens_xyz(sensor_vector);
}


static int
message_from_best_gps(carmen_gps_xyz_message *message)
{
	if ((xsens_handler.extra_gps == 0) || (xsens_handler.extra_gps == 1))
	{
		if (message->nr == 1) // GPS Trimble
			xsens_handler.extra_gps = 2;
		else
			xsens_handler.extra_gps = 1;
	}

	if (xsens_handler.extra_gps == 2)
	{
		if (message->nr == 1)
		{
			xsens_handler.gps_xyz = *message;
			return (1);
		}
		else
			return (0);
	}
	else
	{
		xsens_handler.gps_xyz = *message;
		return (1);
	}
}


static void
carmen_gps_xyz_message_handler(carmen_gps_xyz_message *message)
{
	detect_gps_performance_change(message);

	if (!message_from_best_gps(message))
		return;

	if (!xsens_handler.initial_state_initialized)
		return;

	if (check_time_difference(message->timestamp))
	{
		reinitialized_gps = 1;
	}

	sensor_vector_xsens_xyz *sensor_vector = create_sensor_vector_gps_xyz(message);

	if (!kalman_filter)
	{
		if (message->gps_quality)
			correction(measure_weight_position, sensor_vector, fused_odometry_parameters);
	}
	
	destroy_sensor_vector_xsens_xyz(sensor_vector);
}


static void
initialize_transformations(xsens_xyz_handler *xsens_handler)
{
	tf::Time::init();

	tf::Transform xsens_position_on_board;	
	xsens_position_on_board.setOrigin(carmen_vector3_to_tf_vector3(xsens_handler->xsens_pose.position));
	xsens_position_on_board.setRotation(carmen_rotation_to_tf_quaternion(xsens_handler->xsens_pose.orientation));
	tf::StampedTransform board_to_xsens_transform(xsens_position_on_board, tf::Time(0), board_tf_name, xsens_tf_name);
	transformer.setTransform(board_to_xsens_transform, "board_to_xsens_transform");

	tf::Transform gps_position_on_board;	
	gps_position_on_board.setOrigin(carmen_vector3_to_tf_vector3(xsens_handler->gps_pose_in_the_car.position));
	gps_position_on_board.setRotation(carmen_rotation_to_tf_quaternion(xsens_handler->gps_pose_in_the_car.orientation));
	tf::StampedTransform board_to_gps_transform(gps_position_on_board, tf::Time(0), board_tf_name, gps_tf_name);
	transformer.setTransform(board_to_gps_transform, "board_to_gps_transform");

	tf::Transform board_position_on_car;	
	board_position_on_car.setOrigin(carmen_vector3_to_tf_vector3(xsens_handler->sensor_board_pose.position));
	board_position_on_car.setRotation(carmen_rotation_to_tf_quaternion(xsens_handler->sensor_board_pose.orientation));
	tf::StampedTransform car_to_board_transform(board_position_on_car, tf::Time(0), car_tf_name, board_tf_name);
	transformer.setTransform(car_to_board_transform, "car_to_board_transform");
}


static void 
subscribe_messages(void)
{
	carmen_xsens_subscribe_xsens_global_quat_message(NULL, (carmen_handler_t) xsens_mti_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_xsens_xyz_subscribe_message(NULL, (carmen_handler_t) xsens_xyz_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_gps_xyz_subscribe_message(NULL, (carmen_handler_t) carmen_gps_xyz_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_gps_subscribe_nmea_hdt_message(NULL, (carmen_handler_t) gps_hdt_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


static void 
unsubscribe_messages(void)
{
	carmen_xsens_xyz_unsubscribe_message((carmen_handler_t)xsens_xyz_message_handler);
}


static void 
initialize_carmen_parameters(xsens_xyz_handler *xsens_handler, int argc, char **argv)
{
	int num_items;

	carmen_param_t param_list[] = 
	{
		{(char*)"sensor_board_1", (char*)"x",		CARMEN_PARAM_DOUBLE, &xsens_handler->sensor_board_pose.position.x,	0, NULL},
		{(char*)"sensor_board_1", (char*)"y",		CARMEN_PARAM_DOUBLE, &xsens_handler->sensor_board_pose.position.y,	0, NULL},
		{(char*)"sensor_board_1", (char*)"z",		CARMEN_PARAM_DOUBLE, &xsens_handler->sensor_board_pose.position.z,	0, NULL},
		{(char*)"sensor_board_1", (char*)"roll",	CARMEN_PARAM_DOUBLE, &xsens_handler->sensor_board_pose.orientation.roll,0, NULL},
		{(char*)"sensor_board_1", (char*)"pitch",	CARMEN_PARAM_DOUBLE, &xsens_handler->sensor_board_pose.orientation.pitch,0, NULL},
		{(char*)"sensor_board_1", (char*)"yaw",		CARMEN_PARAM_DOUBLE, &xsens_handler->sensor_board_pose.orientation.yaw,	0, NULL},

		{(char*)"xsens", (char*)"x",			CARMEN_PARAM_DOUBLE, &xsens_handler->xsens_pose.position.x,		0, NULL},
		{(char*)"xsens", (char*)"y",			CARMEN_PARAM_DOUBLE, &xsens_handler->xsens_pose.position.y,		0, NULL},
		{(char*)"xsens", (char*)"z",			CARMEN_PARAM_DOUBLE, &xsens_handler->xsens_pose.position.z,		0, NULL},
		{(char*)"xsens", (char*)"roll",			CARMEN_PARAM_DOUBLE, &xsens_handler->xsens_pose.orientation.roll,	0, NULL},
		{(char*)"xsens", (char*)"pitch",		CARMEN_PARAM_DOUBLE, &xsens_handler->xsens_pose.orientation.pitch,	0, NULL},
		{(char*)"xsens", (char*)"yaw",			CARMEN_PARAM_DOUBLE, &xsens_handler->xsens_pose.orientation.yaw,	0, NULL},

		{(char*)"gps_nmea", (char*)"x",			CARMEN_PARAM_DOUBLE, &xsens_handler->gps_pose_in_the_car.position.x,		0, NULL},
		{(char*)"gps_nmea", (char*)"y",			CARMEN_PARAM_DOUBLE, &xsens_handler->gps_pose_in_the_car.position.y,		0, NULL},
		{(char*)"gps_nmea", (char*)"z",			CARMEN_PARAM_DOUBLE, &xsens_handler->gps_pose_in_the_car.position.z,		0, NULL},
		{(char*)"gps_nmea", (char*)"roll",		CARMEN_PARAM_DOUBLE, &xsens_handler->gps_pose_in_the_car.orientation.roll,		0, NULL},
		{(char*)"gps_nmea", (char*)"pitch",		CARMEN_PARAM_DOUBLE, &xsens_handler->gps_pose_in_the_car.orientation.pitch,	0, NULL},
		{(char*)"gps_nmea", (char*)"yaw",		CARMEN_PARAM_DOUBLE, &xsens_handler->gps_pose_in_the_car.orientation.yaw,		0, NULL},

	};
	
	num_items = sizeof(param_list) / sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);
}


xsens_xyz_handler *
create_xsens_xyz_handler(int argc, char **argv, carmen_fused_odometry_parameters *parameters)
{
	fused_odometry_parameters = parameters;
	
	if (xsens_sensor_vector == NULL)
		xsens_sensor_vector = (sensor_vector_xsens_xyz **) calloc(XSENS_SENSOR_VECTOR_SIZE, sizeof(sensor_vector_xsens_xyz *));

	memset(&xsens_handler, 0, sizeof(xsens_handler));

	initialize_carmen_parameters(&xsens_handler, argc, argv);
	initialize_transformations(&xsens_handler);
	subscribe_messages();

	return (&xsens_handler);	
}


void 
reset_xsens_xyz_handler(xsens_xyz_handler *xsens_handler)
{
	xsens_handler->initial_state_initialized = 0;
}


void 
destroy_xsens_xyz_handler(xsens_xyz_handler *xsens_handler __attribute__ ((unused)))
{
	unsubscribe_messages();
}
