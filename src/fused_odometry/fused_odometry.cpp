
#include <carmen/carmen.h>
#include <carmen/xsens_interface.h>
#include <carmen/gps_xyz_interface.h>
#include <carmen/visual_odometry_interface.h>
#include <carmen/map_server_interface.h>
#include <carmen/localize_ackerman_core.h>

#include "fused_odometry.h"
#include "xsens_xyz_handler.h"
#include "visual_odometry_handler.h"
#include "car_odometry_handler.h"
#include "fused_odometry_interface.h"

#include <tf.h>


// Fused odometry state
int num_particles;
carmen_fused_odometry_particle *xt = NULL;
carmen_fused_odometry_particle *_xt = NULL;
carmen_fused_odometry_message fused_odometry_message;

carmen_fused_odometry_control ut;


static carmen_fused_odometry_particle
sample_motion_model(carmen_fused_odometry_particle x_t_1, carmen_fused_odometry_control ut, double dt, carmen_fused_odometry_parameters *fused_odometry_parameters)
{
	double L = fused_odometry_parameters->axis_distance;
	carmen_fused_odometry_particle x_t = x_t_1;	
	
	x_t.state.velocity.x = ut.v + 
			carmen_gaussian_random(0.0,
					fused_odometry_parameters->velocity_noise_velocity * ut.v * ut.v +
					fused_odometry_parameters->velocity_noise_phi * ut.phi * ut.phi);

	x_t.state.velocity.y = 0.0;
	x_t.state.velocity.z = 0.0;
	//x_t.state.velocity.z = ut.v_z + carmen_gaussian_random(0.0, alpha5 * ut.v_z * ut.v_z);// + alpha6 * ut.v_pitch * ut.v_pitch);

	x_t.state.phi = ut.phi + carmen_gaussian_random(0.0,
					fused_odometry_parameters->phi_noise_phi * ut.phi * ut.phi +
					fused_odometry_parameters->phi_noise_velocity * ut.v * ut.v);

	x_t.state.ang_velocity.roll = 0.0;
	// x_t.state.ang_velocity.pitch = 0.0;
	// x_t.state.ang_velocity.pitch = ut.v_pitch + carmen_gaussian_random(0.0,
	//				fused_odometry_parameters->pitch_v_noise_pitch_v * ut.v_pitch * ut.v_pitch +
	//				fused_odometry_parameters->pitch_v_noise_velocity * ut.v * ut.v);

	x_t.state.ang_velocity.yaw = 0.0;

	// This will limit the wheel position to the maximum angle the car can turn
	if (x_t.state.phi > fused_odometry_parameters->maximum_phi)
	{
		x_t.state.phi = fused_odometry_parameters->maximum_phi;
	}
	else if (x_t.state.phi < -fused_odometry_parameters->maximum_phi)
	{
		x_t.state.phi = -fused_odometry_parameters->maximum_phi;
	}

	//sensor_vector_imu imu_zt = create_sensor_vector_imu(xsens_matrix_message);
	//x_t.state.pose.orientation.roll = imu_zt.orientation.roll;
	x_t.state.pose.orientation.roll = ut.roll;
	x_t.state.pose.orientation.pitch = ut.pitch;
	//x_t.state.pose.orientation.pitch = x_t_1.state.pose.orientation.pitch + x_t.state.ang_velocity.pitch * dt;
	//x_t.state.pose.orientation.pitch = 0.0;	

	x_t.state.pose.orientation.yaw = x_t_1.state.pose.orientation.yaw + (x_t.state.velocity.x * tan(x_t.state.phi) / L) * dt;
	x_t.state.pose.orientation.yaw = carmen_normalize_theta(x_t.state.pose.orientation.yaw);
	if (0)//(fabs(ut.v) > fused_odometry_parameters->minimum_speed_for_correction) && ut.gps_available)
	{
		x_t.state.xsens_yaw_bias = x_t_1.state.xsens_yaw_bias + carmen_gaussian_random(0.0, fused_odometry_parameters->xsens_yaw_bias_noise);
		if (x_t.state.xsens_yaw_bias > fused_odometry_parameters->xsens_maximum_yaw_bias)
		{
			x_t.state.xsens_yaw_bias = fused_odometry_parameters->xsens_maximum_yaw_bias;
		}
		else if (x_t.state.xsens_yaw_bias < -fused_odometry_parameters->xsens_maximum_yaw_bias)
		{
			x_t.state.xsens_yaw_bias = -fused_odometry_parameters->xsens_maximum_yaw_bias;
		}
	}
	else
		x_t.state.xsens_yaw_bias = 0.0;
	
	tf::Vector3 displacement_car_reference(x_t.state.velocity.x * dt, x_t.state.velocity.y * dt, x_t.state.velocity.z * dt);

	tf::Transform car_to_global;
	car_to_global.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	car_to_global.setRotation(tf::Quaternion(x_t.state.pose.orientation.yaw, x_t.state.pose.orientation.pitch, x_t.state.pose.orientation.roll));

	tf::Vector3 displacement_global_reference = car_to_global * displacement_car_reference;

	tf::Vector3 x_t_global_position(x_t_1.state.pose.position.x, x_t_1.state.pose.position.y, x_t_1.state.pose.position.z);
	x_t_global_position = x_t_global_position + displacement_global_reference;

	double xy_uncertainty_due_to_grid_resolution = (0.2 / 4.0) * (0.2 / 4.0);
	double yaw_uncertainty_due_to_grid_resolution = asin((0.2 / 4.0)) * asin((0.2 / 4.0));
	x_t.state.pose.position.x = x_t_global_position.getX() + carmen_gaussian_random(0.0, xy_uncertainty_due_to_grid_resolution);
	x_t.state.pose.position.y = x_t_global_position.getY() + carmen_gaussian_random(0.0, xy_uncertainty_due_to_grid_resolution);
	x_t.state.pose.position.z = 0.0; //x_t_global_position.getZ();

	x_t.state.pose.orientation.yaw = carmen_normalize_theta(x_t.state.pose.orientation.yaw + carmen_gaussian_random(0.0, yaw_uncertainty_due_to_grid_resolution));

	x_t.weight = x_t_1.weight;

	return (x_t);
}


carmen_fused_odometry_particle
sample_motion_model_simple(carmen_fused_odometry_particle x_t_1, carmen_fused_odometry_control ut, double dt, carmen_fused_odometry_parameters *fused_odometry_parameters)
{
	double L = fused_odometry_parameters->axis_distance;
	carmen_fused_odometry_particle x_t = x_t_1;

	x_t.state.velocity.x = ut.v +
			carmen_gaussian_random(0.0,
					fused_odometry_parameters->velocity_noise_velocity * ut.v * ut.v +
					fused_odometry_parameters->velocity_noise_phi * ut.phi * ut.phi);
	x_t.state.velocity.y = 0.0;
	x_t.state.velocity.z = 0.0;

	x_t.state.phi = ut.phi + carmen_gaussian_random(0.0,
					fused_odometry_parameters->phi_noise_phi * ut.phi * ut.phi +
					fused_odometry_parameters->phi_noise_velocity * ut.v * ut.v);

	x_t.state.ang_velocity.roll = 0.0;
	x_t.state.ang_velocity.yaw = 0.0;

	// This will limit the wheel position to the maximum angle the car can turn
	if (x_t.state.phi > fused_odometry_parameters->maximum_phi)
	{
		x_t.state.phi = fused_odometry_parameters->maximum_phi;
	}
	else if (x_t.state.phi < -fused_odometry_parameters->maximum_phi)
	{
		x_t.state.phi = -fused_odometry_parameters->maximum_phi;
	}

	x_t.state.pose.orientation.roll = ut.roll;
	x_t.state.pose.orientation.pitch = ut.pitch;

	x_t.state.xsens_yaw_bias = 0.0;

	x_t.state.pose.position.x += dt * ut.v * cos(x_t_1.state.pose.orientation.yaw);
	x_t.state.pose.position.y += dt * ut.v * sin(x_t_1.state.pose.orientation.yaw);
	x_t.state.pose.position.z = 0.0;
	x_t.state.pose.orientation.yaw = x_t_1.state.pose.orientation.yaw + (x_t.state.velocity.x * tan(x_t.state.phi) / L) * dt;
	x_t.state.pose.orientation.yaw = carmen_normalize_theta(x_t.state.pose.orientation.yaw);

	x_t.weight = x_t_1.weight;

	return (x_t);
}


static void 
resample(carmen_fused_odometry_particle *xt)
{		
	double sum_weights = 0.0;
	int m;

	for (m = 0; m < num_particles; m++)
		sum_weights += xt[m].weight;

	double invM = sum_weights / (double) num_particles; // To avoid problems with a Measurament Model that does not returns probability 1, we sum the weights up
	double r = invM * carmen_uniform_random(0.0, 1.0);
	double c = xt[0].weight;

	for (m = 0; m < num_particles; m++)
		_xt[m] = xt[m];

	int i = 0; // The book appears to have a bug: i should start with zero.
	for (m = 1; m <= num_particles; m++)
	{
		double U = r + (double)(m - 1) * invM;
		while (U > c)
		{
			i = i + 1;
			c = c + _xt[i].weight;
		}
		xt[m - 1] = _xt[i];
	}
}


static void 
low_variance_sampler(carmen_fused_odometry_particle *xt, carmen_fused_odometry_parameters *fused_odometry_parameters)
{		
/*	int m;
	double sum_weights = 0.0;
	double sum_sqr_weight = 0.0;
	//double Neff;

	for (m = 0; m < num_particles; m++)
		sum_weights += xt[m].weight; // @@@ Alberto: O peso das particulas nao esta de acordo com a probabilidade; por isso tem que normalizar.

	for (m = 0; m < num_particles; m++)
		sum_sqr_weight += (xt[m].weight / sum_weights) * (xt[m].weight / sum_weights);

	Neff = 1.0 / sum_sqr_weight;

	//printf("Neff = %lf\n", Neff);
	if (Neff < (num_particles * 0.90)) // Selective resampling: see Grisetti, Stachniss and Burgard
	{	// Nao estamos utilizando selective resample por causa de nao estarmos computando o peso das paticulas precisamente
*/
	if (fabs(ut.v) > fused_odometry_parameters->minimum_speed_for_correction)
	{
		resample(xt);
	}
}


void 
prediction(double timestamp, carmen_fused_odometry_parameters *fused_odometry_parameters)
{
	int i;
	
	for (i = 0; i < num_particles; i++)
	{	
		double dt = timestamp - xt[i].state.timestamp;

		xt[i] = sample_motion_model(xt[i], ut, dt, fused_odometry_parameters);
		xt[i].state.timestamp = timestamp;
	}

}


void 
correction(double (*weight_func)(carmen_fused_odometry_state_vector, void *, carmen_fused_odometry_parameters *), void *sensor_vector, carmen_fused_odometry_parameters *fused_odometry_parameters)
{
	int i;
	
	for (i = 0; i < num_particles; i++)
	{	
		xt[i].weight = weight_func(xt[i].state, sensor_vector, fused_odometry_parameters);
		xt[i].weight_type = 0;
	}

	low_variance_sampler(xt, fused_odometry_parameters);
}


void 
set_fused_odometry_control_vector(carmen_fused_odometry_control c)
{
	ut = c;
}


carmen_fused_odometry_control *
get_fused_odometry_control_vector()
{
	return (&ut);
}


carmen_fused_odometry_message *
get_fused_odometry()
{
	return (&fused_odometry_message);
}


void 
add_gps_samples_into_particle_pool(carmen_gps_xyz_message *gps_xyz)
{
	int i;
	
	for (i = 0; i < num_particles; i = i + 400)
	{	
		xt[i].state.pose.position.x = gps_xyz->x;
		xt[i].state.pose.position.y = gps_xyz->y;
		xt[i].state.pose.position.z = gps_xyz->z;
	}
}
